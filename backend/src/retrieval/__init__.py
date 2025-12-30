"""Data Retrieval & Pipeline Validation Service

This module provides retrieval functionality for the RAG system, including:
- Vector similarity search against stored embeddings
- Metadata validation and content verification
- Deterministic test query suite
- Comprehensive validation reporting
"""

from .retrieval_service import RetrievalService
from .validator import MetadataValidator, ContentVerifier
from .qdrant_client import QdrantClient
from .embedding_service import EmbeddingService

__all__ = [
    "RetrievalService",
    "MetadataValidator",
    "ContentVerifier",
    "QdrantClient",
    "EmbeddingService",
]

__version__ = "1.0.0"
