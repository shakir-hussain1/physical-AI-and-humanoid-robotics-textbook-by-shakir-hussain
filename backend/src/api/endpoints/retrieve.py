"""Retrieval endpoint for direct search without LLM generation."""

import time

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse

from ..errors import InvalidKError, QueryTooLongError, RetrievalError, ValidationError
from ..models import RetrievalRequest, RetrievalResponse, RetrievalResult
from ..validators import validate_k_range, validate_query_length
from ...utils.logging import get_logger

router = APIRouter(prefix="/api", tags=["retrieval"])
logger = get_logger()


@router.post("/retrieve", response_model=RetrievalResponse, status_code=200)
async def retrieve_endpoint(request: Request, retrieval_request: RetrievalRequest) -> RetrievalResponse:
    """
    Retrieve raw search results without LLM generation.

    POST /api/retrieve
    - Accepts: query, k (number of results)
    - Returns: list of retrieved documents with metadata
    - Errors: 400 (invalid input), 503 (retrieval unavailable)
    """
    request_id = getattr(request.state, "request_id", "unknown")
    start_time = time.time()

    try:
        # Import retrieval service dynamically (initialized in lifespan)
        from ..main import retrieval_service

        if not retrieval_service:
            logger.error("Retrieval service not initialized", extra={"request_id": request_id})
            raise RetrievalError("Retrieval service is not available")

        # Validate request
        valid, error_msg = validate_query_length(retrieval_request.query)
        if not valid:
            logger.warning(
                f"Query validation failed: {error_msg}",
                extra={"request_id": request_id},
            )
            raise QueryTooLongError()

        valid, error_msg = validate_k_range(retrieval_request.k)
        if not valid:
            logger.warning(
                f"K validation failed: {error_msg}",
                extra={"request_id": request_id},
            )
            raise InvalidKError(retrieval_request.k)

        logger.info(
            "Retrieval started",
            extra={
                "request_id": request_id,
                "query_length": len(retrieval_request.query),
                "k": retrieval_request.k,
            },
        )

        # Call retrieval service
        search_results = retrieval_service.search(
            query_text=retrieval_request.query,
            k=retrieval_request.k,
        )

        # Format results
        latency_ms = int((time.time() - start_time) * 1000)

        results = [
            RetrievalResult(
                id=result.get("id", f"chunk_{i}"),
                text=result.get("text", ""),
                similarity_score=result.get("similarity_score", 0.0),
                metadata={
                    "url": result.get("metadata", {}).get("url", ""),
                    "page_title": result.get("metadata", {}).get("page_title", ""),
                    "chunk_index": result.get("metadata", {}).get("chunk_index", i),
                },
            )
            for i, result in enumerate(search_results)
        ]

        response = RetrievalResponse(
            results=results,
            count=len(results),
            latency_ms=latency_ms,
        )

        logger.info(
            "Retrieval completed",
            extra={
                "request_id": request_id,
                "latency_ms": latency_ms,
                "results_count": len(results),
            },
        )

        return response

    except QueryTooLongError:
        raise
    except InvalidKError:
        raise
    except RetrievalError:
        raise
    except Exception as e:
        error_type = type(e).__name__
        logger.error(
            f"Retrieval failed: {str(e)}",
            extra={
                "request_id": request_id,
                "error_type": error_type,
            },
            exc_info=True,
        )

        raise RetrievalError(f"Unable to retrieve documents: {str(e)}")
