"""Pydantic request/response schemas for RAG Chatbot API."""
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid


class Chunk(BaseModel):
    """Schema for text chunks used in retrieval."""
    chunk_id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique chunk identifier")
    text: str = Field(..., description="Chunk text content")
    page_number: int = Field(..., description="Page number where chunk appears")
    section: str = Field(..., description="Section heading")
    score: Optional[float] = Field(None, description="Relevance score from retrieval")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata")


class Citation(BaseModel):
    """Schema for citation information."""
    chunk_id: str = Field(..., description="Referenced chunk ID")
    excerpt: str = Field(..., description="Excerpt text from source")
    page_number: int = Field(..., description="Page number of citation")
    section_heading: str = Field(..., description="Section where citation appears")
    source_confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence in source")


class QueryRequest(BaseModel):
    """Request schema for POST /chat/query endpoint."""
    query: str = Field(..., min_length=1, max_length=1000, description="User question")
    conversation_id: Optional[str] = Field(None, description="Optional conversation ID for multi-turn (future)")


class ContextRestrictedQueryRequest(BaseModel):
    """Request schema for POST /chat/context-restricted endpoint."""
    query: str = Field(..., min_length=1, max_length=1000, description="User question")
    selected_passage: str = Field(..., min_length=50, max_length=5000, description="User-selected text passage")


class QueryResponse(BaseModel):
    """Response schema for POST /chat/query endpoint."""
    answer_id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique answer identifier")
    answer: str = Field(..., description="Generated response with inline citations")
    sources: List[Citation] = Field(..., description="List of cited sources")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence score (0.0-1.0)")
    confidence_level: str = Field(..., description="Qualitative confidence level (high/medium/low)")
    status: str = Field(..., description="Answer status (success/partial/out_of_scope/error)")


class ContextRestrictedQueryResponse(QueryResponse):
    """Response schema for POST /chat/context-restricted endpoint."""
    source_mode: str = Field(default="context-restricted", description="Answer restricted to user-selected passage")


class IngestRequest(BaseModel):
    """Request schema for POST /content/ingest endpoint."""
    chapter_id: str = Field(..., description="Unique chapter identifier")


class IngestResponse(BaseModel):
    """Response schema for POST /content/ingest endpoint."""
    ingest_id: str = Field(..., description="Job ID for status polling")
    status: str = Field(..., description="Ingest status (queued/processing/completed/failed)")
    message: str = Field(..., description="Status message")
    chunks_created: Optional[int] = Field(None, description="Number of chunks created (populated when completed)")


class HealthResponse(BaseModel):
    """Response schema for GET /health endpoint."""
    status: str = Field(..., description="System status (healthy/degraded/unhealthy)")
    timestamp: str = Field(..., description="ISO 8601 timestamp")
    services: dict = Field(..., description="Individual service health status")


class ErrorDetail(BaseModel):
    """Error response detail."""
    code: str = Field(..., description="Error code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[str] = Field(None, description="Additional context for debugging")


class ErrorResponse(BaseModel):
    """Error response schema."""
    error: ErrorDetail = Field(..., description="Error information")