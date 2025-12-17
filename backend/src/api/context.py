"""API router for context-restricted chat endpoints."""
from fastapi import APIRouter, HTTPException
from typing import Optional
import logging
import uuid
from datetime import datetime

from src.models.schemas import ContextRestrictedQueryRequest, ContextRestrictedQueryResponse, Chunk, Citation
from src.utils.validation import validate_query_length
from src.utils.exceptions import QueryValidationError, LowConfidenceError
from src.services.retrieval import RetrieverService
from src.services.generation import GenerationService
from src.services.citations import CitationService
from src.services.audit_logger import AuditLoggerService
from src.config import settings

logger = logging.getLogger(__name__)
router = APIRouter()

# Global service instances for MVP (shared with query.py)
from src.api.query import retriever_service, generation_service, citation_service, audit_logger_service

@router.post("/context-restricted", response_model=ContextRestrictedQueryResponse)
async def context_restricted_query_endpoint(request: ContextRestrictedQueryRequest):
    """Handle queries restricted to a specific passage context."""
    start_time = datetime.utcnow()
    query_id = str(uuid.uuid4())

    try:
        # Validate query and passage
        validate_query_length(request.query, min_length=1, max_length=1000)
        if not request.selected_passage or len(request.selected_passage.strip()) < 50:
            raise QueryValidationError("Selected passage too short to analyze (min 50 characters)")

        # Log the query
        if audit_logger_service:
            await audit_logger_service.log_query(request.query, source_mode="context-restricted")

        # Create a chunk from the selected passage
        passage_chunk = Chunk(
            text=request.selected_passage,
            page_number=1,  # For context-restricted, we'll use page 1 as default
            section="Selected Passage",
            score=1.0  # Full confidence since user provided the context
        )
        chunks = [passage_chunk]

        # Generate answer from the passage
        answer = ""
        generation_confidence = 0.0
        if generation_service and chunks:
            generation_result = await generation_service.generate_answer(chunks, request.query)
            answer = generation_result["answer"]
            generation_confidence = generation_result["confidence"]

        # Calculate final confidence (based on generation since context is provided)
        final_confidence = generation_confidence

        # Process citations
        citations = []
        if citation_service and chunks:
            # Build citation objects
            citations = await citation_service.build_citation_objects(chunks)

            # Inject citations into answer
            answer = await citation_service.inject_citations(answer, chunks)

        # Determine status based on confidence
        status = "success"
        if final_confidence < 0.60:
            status = "out_of_scope"
            answer = "Cannot be fully answered from your selected passage. Would you like to expand to the full chapter?"

        # Create response with context-restricted mode
        response = ContextRestrictedQueryResponse(
            answer=answer,
            sources=citations,
            confidence=final_confidence,
            confidence_level="high" if final_confidence >= 0.8 else "medium" if final_confidence >= 0.6 else "low",
            status=status,
            source_mode="context-restricted"
        )

        # Log the answer
        if audit_logger_service:
            answer_id = str(uuid.uuid4())
            await audit_logger_service.log_answer(
                query_id,
                f"Context-restricted query: {request.query}",
                answer,
                answer,
                final_confidence,
                [c.dict() for c in citations]
            )

        # Calculate and log processing time
        processing_time_ms = (datetime.utcnow() - start_time).total_seconds() * 1000
        if audit_logger_service:
            await audit_logger_service.log_retrieval(
                query_id,
                request.query,
                len(chunks),
                [c.dict() for c in chunks],
                processing_time_ms
            )

        return response

    except QueryValidationError as e:
        logger.error(f"Context-restricted query validation error: {e}")
        if audit_logger_service:
            await audit_logger_service.log_operation({
                "query_id": query_id,
                "error": str(e),
                "error_type": "QueryValidationError"
            })
        raise HTTPException(status_code=400, detail=str(e))

    except LowConfidenceError as e:
        logger.warning(f"Low confidence error: {e}")
        response = ContextRestrictedQueryResponse(
            answer="Cannot be fully answered from your selected passage. Would you like to expand to the full chapter?",
            sources=[],
            confidence=0.0,
            confidence_level="low",
            status="out_of_scope",
            source_mode="context-restricted"
        )
        return response

    except Exception as e:
        logger.error(f"Context-restricted query endpoint error: {e}")
        if audit_logger_service:
            await audit_logger_service.log_operation({
                "query_id": query_id,
                "error": str(e),
                "error_type": "InternalServerError"
            })
        raise HTTPException(status_code=500, detail="Internal server error")