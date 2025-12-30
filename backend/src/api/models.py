"""Pydantic models for request/response validation."""

from datetime import datetime
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field, field_validator


# ============================================================================
# Request Models
# ============================================================================


class MessageModel(BaseModel):
    """Conversation message model."""

    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")
    timestamp: Optional[str] = Field(
        default_factory=lambda: datetime.utcnow().isoformat() + "Z",
        description="ISO8601 timestamp",
    )

    @field_validator("role")
    @classmethod
    def validate_role(cls, v: str) -> str:
        """Validate role is user or assistant."""
        if v not in ["user", "assistant"]:
            raise ValueError("Role must be 'user' or 'assistant'")
        return v


class QueryRequest(BaseModel):
    """Query endpoint request model."""

    query: str = Field(..., description="User query text", max_length=10000)
    conversation_history: Optional[List[MessageModel]] = Field(
        default=None, description="Previous conversation messages"
    )
    user_role: str = Field(default="student", description="User role: student, teacher, or researcher")

    @field_validator("query")
    @classmethod
    def validate_query(cls, v: str) -> str:
        """Validate query is not empty."""
        if not v or not v.strip():
            raise ValueError("Query cannot be empty")
        return v.strip()

    @field_validator("user_role")
    @classmethod
    def validate_user_role(cls, v: str) -> str:
        """Validate user role."""
        if v not in ["student", "teacher", "researcher"]:
            raise ValueError("User role must be one of: student, teacher, researcher")
        return v


class ContextQueryRequest(BaseModel):
    """Context query endpoint request model."""

    selected_text: str = Field(..., description="Selected text from document", max_length=5000)
    query: str = Field(..., description="User query text", max_length=10000)
    conversation_history: Optional[List[MessageModel]] = Field(
        default=None, description="Previous conversation messages"
    )

    @field_validator("selected_text")
    @classmethod
    def validate_selected_text(cls, v: str) -> str:
        """Validate selected text is not empty."""
        if not v or not v.strip():
            raise ValueError("Selected text cannot be empty")
        return v.strip()

    @field_validator("query")
    @classmethod
    def validate_query(cls, v: str) -> str:
        """Validate query is not empty."""
        if not v or not v.strip():
            raise ValueError("Query cannot be empty")
        return v.strip()


class RetrievalRequest(BaseModel):
    """Retrieval endpoint request model."""

    query: str = Field(..., description="Search query", max_length=10000)
    k: int = Field(default=5, description="Number of results to return", ge=1, le=20)

    @field_validator("query")
    @classmethod
    def validate_query(cls, v: str) -> str:
        """Validate query is not empty."""
        if not v or not v.strip():
            raise ValueError("Query cannot be empty")
        return v.strip()


# ============================================================================
# Response Models
# ============================================================================


class SourceInfo(BaseModel):
    """Source document information."""

    url: str = Field(..., description="Source URL")
    page_title: str = Field(..., description="Document page title")
    relevance_score: float = Field(..., description="Relevance score [0-1]", ge=0.0, le=1.0)
    chunk_index: int = Field(..., description="Chunk index in document", ge=0)


class QueryResponse(BaseModel):
    """Query endpoint response model."""

    answer: str = Field(..., description="Generated answer")
    sources: List[SourceInfo] = Field(default=[], description="Source documents")
    confidence: str = Field(..., description="Confidence level: high, medium, or low")
    metadata: Dict[str, Any] = Field(
        default_factory=dict, description="Additional metadata (latency_ms, grounding, follow_ups, etc.)"
    )
    timestamp: str = Field(default_factory=lambda: datetime.utcnow().isoformat() + "Z", description="Response timestamp")

    @field_validator("confidence")
    @classmethod
    def validate_confidence(cls, v: str) -> str:
        """Validate confidence level."""
        if v not in ["high", "medium", "low"]:
            raise ValueError("Confidence must be one of: high, medium, low")
        return v


class RetrievalResult(BaseModel):
    """Individual retrieval result."""

    id: str = Field(..., description="Unique chunk ID")
    text: str = Field(..., description="Chunk text content")
    similarity_score: float = Field(..., description="Similarity score [0-1]", ge=0.0, le=1.0)
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Chunk metadata (url, page_title, etc.)")


class RetrievalResponse(BaseModel):
    """Retrieval endpoint response model."""

    results: List[RetrievalResult] = Field(..., description="Retrieved documents")
    count: int = Field(..., description="Number of results returned", ge=0)
    latency_ms: int = Field(..., description="Query latency in milliseconds", ge=0)


class HealthComponentStatus(BaseModel):
    """Health status of a component."""

    name: str = Field(..., description="Component name")
    status: str = Field(..., description="Status: ready, degraded, or error")
    details: Optional[str] = Field(None, description="Additional status details")


class HealthResponse(BaseModel):
    """Health check response model."""

    status: str = Field(..., description="Overall status: healthy, degraded, or error")
    timestamp: str = Field(default_factory=lambda: datetime.utcnow().isoformat() + "Z", description="Check timestamp")
    components: Dict[str, str] = Field(..., description="Component status map")

    @field_validator("status")
    @classmethod
    def validate_status(cls, v: str) -> str:
        """Validate overall status."""
        if v not in ["healthy", "degraded", "error"]:
            raise ValueError("Status must be one of: healthy, degraded, error")
        return v


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(..., description="Error code/type")
    message: str = Field(..., description="Human-readable error message")
    request_id: str = Field(..., description="Request ID for debugging")
    timestamp: str = Field(default_factory=lambda: datetime.utcnow().isoformat() + "Z", description="Error timestamp")


# ============================================================================
# Validation Utilities
# ============================================================================


def validate_query_length(query: str, max_length: int = 10000) -> tuple[bool, Optional[str]]:
    """Validate query length."""
    if not query or not query.strip():
        return False, "Query cannot be empty"
    if len(query) > max_length:
        return False, f"Query exceeds maximum length of {max_length} characters"
    return True, None


def validate_history_format(history: Optional[List[Dict[str, str]]]) -> tuple[bool, Optional[str]]:
    """Validate conversation history format."""
    if history is None:
        return True, None

    if not isinstance(history, list):
        return False, "Conversation history must be a list"

    for i, msg in enumerate(history):
        if not isinstance(msg, dict):
            return False, f"History item {i} must be a dict"
        if "role" not in msg or "content" not in msg:
            return False, f"History item {i} must have 'role' and 'content' fields"
        if msg["role"] not in ["user", "assistant"]:
            return False, f"History item {i} has invalid role: {msg['role']}"

    return True, None


def validate_k_range(k: int, min_k: int = 1, max_k: int = 20) -> tuple[bool, Optional[str]]:
    """Validate k parameter for retrieval."""
    if k < min_k or k > max_k:
        return False, f"k must be between {min_k} and {max_k}, got {k}"
    return True, None
