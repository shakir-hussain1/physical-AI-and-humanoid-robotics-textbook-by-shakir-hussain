"""Global exception handlers for FastAPI."""

from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse
import logging

from src.utils.exceptions import QueryValidationError, LowConfidenceError, TimeoutError, RetrieverError, GenerationError, HallucinationsDetectedError
from src.utils.constants import ERROR_INVALID_QUERY, ERROR_LOW_CONFIDENCE, ERROR_SERVER_ERROR, ERROR_TIMEOUT, MSG_INVALID_QUERY, MSG_LOW_CONFIDENCE, MSG_TIMEOUT

logger = logging.getLogger(__name__)


def setup_exception_handlers(app: FastAPI):
    """Setup all exception handlers for the FastAPI app."""

    @app.exception_handler(QueryValidationError)
    async def query_validation_error_handler(request: Request, exc: QueryValidationError):
        logger.warning(f"Query validation error: {str(exc)}")
        return JSONResponse(
            status_code=status.HTTP_400_BAD_REQUEST,
            content={
                "error": {
                    "code": ERROR_INVALID_QUERY,
                    "message": str(exc),
                    "details": None,
                }
            },
        )

    @app.exception_handler(LowConfidenceError)
    async def low_confidence_error_handler(request: Request, exc: LowConfidenceError):
        logger.warning(f"Low confidence: {str(exc)}")
        return JSONResponse(
            status_code=status.HTTP_200_OK,
            content={
                "answer_id": None,
                "answer": MSG_LOW_CONFIDENCE,
                "sources": [],
                "confidence": 0.0,
                "confidence_level": "low",
                "status": "out_of_scope",
            },
        )

    @app.exception_handler(TimeoutError)
    async def timeout_error_handler(request: Request, exc: TimeoutError):
        logger.warning(f"Timeout error: {str(exc)}")
        return JSONResponse(
            status_code=status.HTTP_504_GATEWAY_TIMEOUT,
            content={
                "error": {
                    "code": ERROR_TIMEOUT,
                    "message": MSG_TIMEOUT,
                    "details": None,
                }
            },
        )

    @app.exception_handler(RetrieverError)
    async def retriever_error_handler(request: Request, exc: RetrieverError):
        logger.error(f"Retriever error: {str(exc)}")
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": {
                    "code": "RETRIEVER_ERROR",
                    "message": "Error in retrieval service",
                    "details": None,
                }
            },
        )

    @app.exception_handler(GenerationError)
    async def generation_error_handler(request: Request, exc: GenerationError):
        logger.error(f"Generation error: {str(exc)}")
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": {
                    "code": "GENERATION_ERROR",
                    "message": "Error in generation service",
                    "details": None,
                }
            },
        )

    @app.exception_handler(HallucinationsDetectedError)
    async def hallucinations_error_handler(request: Request, exc: HallucinationsDetectedError):
        logger.warning(f"Hallucinations detected: {str(exc)}")
        return JSONResponse(
            status_code=status.HTTP_200_OK,
            content={
                "answer_id": None,
                "answer": "Response contains potential hallucinations and was rejected.",
                "sources": [],
                "confidence": 0.0,
                "confidence_level": "low",
                "status": "hallucination_rejected",
            },
        )

    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": {
                    "code": ERROR_SERVER_ERROR,
                    "message": "Internal server error",
                    "details": None,
                }
            },
        )
