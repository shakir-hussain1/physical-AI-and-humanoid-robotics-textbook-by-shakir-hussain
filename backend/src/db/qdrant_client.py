"""Qdrant vector database client for semantic search."""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from typing import List, Dict, Optional
import logging
import uuid

from src.config import settings

logger = logging.getLogger(__name__)

# Initialize Qdrant client
qdrant_client = QdrantClient(url=settings.QDRANT_URL)


def init_qdrant_collection():
    """Initialize Qdrant collection if not exists."""
    try:
        # Check if collection exists
        try:
            qdrant_client.get_collection("chunks")
            logger.info("Qdrant collection 'chunks' already exists")
            return
        except:
            pass

        # Create collection for embeddings (768 dimensions for Cohere)
        qdrant_client.create_collection(
            collection_name="chunks",
            vectors_config=VectorParams(
                size=768,  # Cohere embed-english-v3.0 uses 768 dimensions
                distance=Distance.COSINE,
            ),
        )
        logger.info("Qdrant collection 'chunks' created successfully")
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant collection: {e}")
        raise


def upsert_chunk(
    chunk_id: str,
    text: str,
    embedding: List[float],
    page_number: int,
    section_heading: str,
    document_id: str,
) -> bool:
    """Upsert a chunk into Qdrant."""
    try:
        point = PointStruct(
            id=int(uuid.UUID(chunk_id).int % (2**63)),  # Convert UUID to int for Qdrant
            vector=embedding,
            payload={
                "chunk_id": chunk_id,
                "text": text,
                "page_number": page_number,
                "section": section_heading,
                "document_id": document_id,
            },
        )
        qdrant_client.upsert(
            collection_name="chunks",
            points=[point],
        )
        return True
    except Exception as e:
        logger.error(f"Failed to upsert chunk to Qdrant: {e}")
        return False


def search_chunks(
    query_embedding: List[float],
    top_k: int = 5,
    score_threshold: float = 0.0,
) -> List[Dict]:
    """Search for similar chunks using vector similarity."""
    try:
        results = qdrant_client.search(
            collection_name="chunks",
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=score_threshold,
        )

        chunks = []
        for result in results:
            chunks.append({
                "chunk_id": result.payload["chunk_id"],
                "text": result.payload["text"],
                "page_number": result.payload["page_number"],
                "section": result.payload["section"],
                "document_id": result.payload["document_id"],
                "score": result.score,
            })

        return chunks
    except Exception as e:
        logger.error(f"Failed to search chunks in Qdrant: {e}")
        return []


def delete_document_chunks(document_id: str) -> bool:
    """Delete all chunks for a document."""
    try:
        qdrant_client.delete(
            collection_name="chunks",
            points_selector={
                "filter": {
                    "must": [
                        {
                            "key": "document_id",
                            "match": {"value": document_id},
                        }
                    ]
                }
            },
        )
        logger.info(f"Deleted chunks for document {document_id}")
        return True
    except Exception as e:
        logger.error(f"Failed to delete chunks: {e}")
        return False


def check_qdrant_connection() -> bool:
    """Check if Qdrant is accessible."""
    try:
        qdrant_client.get_collections()
        return True
    except Exception as e:
        logger.error(f"Qdrant connection check failed: {e}")
        return False
