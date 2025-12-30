"""Data models for RAG agent query processing and responses."""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Literal
from datetime import datetime


@dataclass
class Message:
    """Conversation message for history tracking."""

    role: Literal["user", "assistant"]
    content: str
    timestamp: Optional[str] = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().isoformat()


@dataclass
class SourceInfo:
    """Information about a source chunk used in response."""

    url: str
    page_title: str
    relevance_score: float
    chunk_index: int
    chunk_text: Optional[str] = None

    def __post_init__(self):
        if not 0 <= self.relevance_score <= 1:
            raise ValueError(f"relevance_score must be in [0, 1], got {self.relevance_score}")


@dataclass
class RetrievedChunk:
    """Chunk retrieved from vector database."""

    id: str
    text: str
    similarity_score: float
    metadata: Dict[str, str]

    def __post_init__(self):
        if not 0 <= self.similarity_score <= 1:
            raise ValueError(f"similarity_score must be in [0, 1], got {self.similarity_score}")


@dataclass
class Intent:
    """Parsed intent from user query."""

    query_text: str
    query_type: Literal["factual", "conceptual", "how_to", "clarification", "out_of_scope"]
    primary_topic: Optional[str]
    scope: Optional[str]
    confidence: float = 0.8


@dataclass
class Query:
    """User query with optional conversation context."""

    text: str
    conversation_history: List[Message] = field(default_factory=list)
    user_role: Literal["student", "teacher", "researcher"] = "student"
    context_scope: Optional[str] = None


@dataclass
class AgentResponse:
    """Structured response from RAG agent."""

    answer: str
    sources: List[SourceInfo] = field(default_factory=list)
    confidence: Literal["high", "medium", "low"] = "medium"
    metadata: Dict[str, any] = field(default_factory=dict)
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())

    def to_dict(self) -> Dict:
        """Convert to dictionary for serialization."""
        return {
            "answer": self.answer,
            "sources": [
                {
                    "url": s.url,
                    "page_title": s.page_title,
                    "relevance_score": s.relevance_score,
                    "chunk_index": s.chunk_index,
                }
                for s in self.sources
            ],
            "confidence": self.confidence,
            "metadata": self.metadata,
            "timestamp": self.timestamp,
        }
