"""API router for chat query endpoints."""
from fastapi import APIRouter, Depends, HTTPException
from typing import Optional
import logging
import uuid
from datetime import datetime

from src.models.schemas import QueryRequest, QueryResponse, Chunk
from src.utils.validation import validate_query_length
from src.utils.exceptions import QueryValidationError, LowConfidenceError, TimeoutError
from src.services.retrieval import RetrieverService
from src.services.generation import GenerationService
from src.services.citations import CitationService
from src.services.audit_logger import AuditLoggerService
from src.config import settings
from src.utils.tracing import get_trace_id

logger = logging.getLogger(__name__)
router = APIRouter()

# Global service instances for MVP
retriever_service: Optional[RetrieverService] = None
generation_service: Optional[GenerationService] = None
citation_service: Optional[CitationService] = None
audit_logger_service: Optional[AuditLoggerService] = None

def initialize_services():
    """Initialize global service instances."""
    global retriever_service, generation_service, citation_service, audit_logger_service

    # In a real implementation, these would be initialized with proper API keys
    retriever_service = RetrieverService(
        qdrant_url=settings.QDRANT_URL,
        cohere_key=settings.COHERE_API_KEY
    )
    generation_service = GenerationService(
        anthropic_api_key=settings.ANTHROPIC_API_KEY
    )
    citation_service = CitationService()
    audit_logger_service = AuditLoggerService()

# Initialize services on startup
initialize_services()

@router.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Handle natural language queries and return answers with citations."""
    start_time = datetime.utcnow()
    query_id = str(uuid.uuid4())

    try:
        # Validate query
        validate_query_length(request.query, min_length=1, max_length=1000)

        # Log the query
        if audit_logger_service:
            await audit_logger_service.log_query(request.query, source_mode="standard")

        # Process with retrieval service
        chunks = []
        retrieval_confidence = 0.0
        if retriever_service:
            chunks = await retriever_service.retrieve_chunks(request.query)
            chunks, retrieval_confidence = await retriever_service.rank_by_confidence(chunks)

        # Generate answer
        answer = ""
        generation_confidence = 0.0
        if generation_service and chunks:
            generation_result = await generation_service.generate_answer(chunks, request.query)
            answer = generation_result["answer"]
            generation_confidence = generation_result["confidence"]

        # Calculate final confidence
        final_confidence = 0.0
        if generation_service and retrieval_confidence > 0 and generation_confidence > 0:
            final_confidence = await generation_service.estimate_confidence(
                retrieval_confidence,
                {"confidence": generation_confidence}
            )

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
            answer = "This question is not covered in the Physical AI textbook"

        # Create response
        response = QueryResponse(
            answer=answer,
            sources=citations,
            confidence=final_confidence,
            confidence_level="high" if final_confidence >= 0.8 else "medium" if final_confidence >= 0.6 else "low",
            status=status
        )

        # Log the answer
        if audit_logger_service:
            answer_id = str(uuid.uuid4())
            await audit_logger_service.log_answer(
                query_id,
                f"Query: {request.query}",
                answer,
                answer,
                final_confidence,
                [c.dict() for c in citations]
            )

        # Calculate and log retrieval time
        retrieval_time_ms = (datetime.utcnow() - start_time).total_seconds() * 1000
        if audit_logger_service:
            await audit_logger_service.log_retrieval(
                query_id,
                request.query,
                len(chunks),
                [c.dict() for c in chunks],
                retrieval_time_ms
            )

        return response

    except QueryValidationError as e:
        logger.error(f"Query validation error: {e}")
        if audit_logger_service:
            await audit_logger_service.log_operation({
                "query_id": query_id,
                "error": str(e),
                "error_type": "QueryValidationError"
            })
        raise HTTPException(status_code=400, detail=str(e))

    except LowConfidenceError as e:
        logger.warning(f"Low confidence error: {e}")
        response = QueryResponse(
            answer="This question is not covered in the Physical AI textbook",
            sources=[],
            confidence=0.0,
            confidence_level="low",
            status="out_of_scope"
        )
        return response

    except TimeoutError as e:
        logger.error(f"Query timeout error: {e}")
        if audit_logger_service:
            await audit_logger_service.log_operation({
                "query_id": query_id,
                "error": str(e),
                "error_type": "TimeoutError"
            })
        raise HTTPException(status_code=504, detail="Query processing timed out")

    except Exception as e:
        logger.error(f"Query endpoint error: {e}")
        if audit_logger_service:
            await audit_logger_service.log_operation({
                "query_id": query_id,
                "error": str(e),
                "error_type": "InternalServerError"
            })
        raise HTTPException(status_code=500, detail="Internal server error")