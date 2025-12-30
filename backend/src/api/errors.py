"""Custom exception and error handling for API."""

from fastapi import HTTPException, status
from typing import Any, Dict, Optional


class APIError(HTTPException):
    """Base API error."""

    def __init__(
        self,
        detail: str,
        status_code: int = 500,
        error_code: str = "internal_error",
        extra: Optional[Dict[str, Any]] = None,
    ):
        """Initialize API error."""
        super().__init__(status_code=status_code, detail=detail)
        self.error_code = error_code
        self.extra = extra or {}

    def to_dict(self, request_id: str) -> Dict[str, Any]:
        """Convert error to response dict."""
        response = {
            "error": self.error_code,
            "message": self.detail,
            "request_id": request_id,
        }
        response.update(self.extra)
        return response


class ValidationError(APIError):
    """Request validation error (400)."""

    def __init__(self, detail: str, field: Optional[str] = None):
        """Initialize validation error."""
        super().__init__(
            detail=detail,
            status_code=status.HTTP_400_BAD_REQUEST,
            error_code="validation_error",
            extra={"field": field} if field else {},
        )


class QueryTooLongError(ValidationError):
    """Query exceeds maximum length."""

    def __init__(self, max_length: int = 10000):
        """Initialize query length error."""
        super().__init__(
            detail=f"Query exceeds maximum length of {max_length} characters",
            field="query",
        )


class InvalidHistoryError(ValidationError):
    """Invalid conversation history format."""

    def __init__(self, detail: str):
        """Initialize history error."""
        super().__init__(detail=detail, field="conversation_history")


class InvalidKError(ValidationError):
    """Invalid k parameter."""

    def __init__(self, k: int, min_k: int = 1, max_k: int = 20):
        """Initialize k validation error."""
        super().__init__(
            detail=f"k must be between {min_k} and {max_k}, got {k}",
            field="k",
        )


class ServiceUnavailableError(APIError):
    """Service temporarily unavailable (503)."""

    def __init__(self, service: str = "Knowledge base"):
        """Initialize service unavailable error."""
        super().__init__(
            detail=f"{service} is temporarily unavailable. Please try again later.",
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            error_code="service_unavailable",
            extra={"service": service},
        )


class RequestTimeoutError(APIError):
    """Request timeout (504)."""

    def __init__(self, timeout_seconds: int = 30):
        """Initialize timeout error."""
        super().__init__(
            detail=f"Request took too long (exceeded {timeout_seconds} seconds). Please try again.",
            status_code=status.HTTP_504_GATEWAY_TIMEOUT,
            error_code="request_timeout",
            extra={"timeout_seconds": timeout_seconds},
        )


class LLMError(APIError):
    """LLM API error (502)."""

    def __init__(self, detail: str = "Unable to generate response"):
        """Initialize LLM error."""
        super().__init__(
            detail=detail,
            status_code=status.HTTP_502_BAD_GATEWAY,
            error_code="llm_error",
        )


class RetrievalError(APIError):
    """Retrieval service error (503)."""

    def __init__(self, detail: str = "Unable to retrieve documents"):
        """Initialize retrieval error."""
        super().__init__(
            detail=detail,
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            error_code="retrieval_error",
        )


class AgentError(APIError):
    """Agent processing error (502)."""

    def __init__(self, detail: str = "Unable to process query"):
        """Initialize agent error."""
        super().__init__(
            detail=detail,
            status_code=status.HTTP_502_BAD_GATEWAY,
            error_code="agent_error",
        )


class OutOfDomainError(APIError):
    """Query is out of domain (400)."""

    def __init__(self, detail: str = "This question is outside the scope of the available knowledge base"):
        """Initialize out of domain error."""
        super().__init__(
            detail=detail,
            status_code=status.HTTP_400_BAD_REQUEST,
            error_code="out_of_domain",
        )
