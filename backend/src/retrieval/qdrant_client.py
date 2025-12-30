"""Qdrant Vector Database Client Wrapper

Provides retrieval-specific methods for querying vectors and metadata from Qdrant.
"""

import logging
from typing import List, Dict, Optional, Tuple
from qdrant_client import QdrantClient as QdrantBaseClient
from qdrant_client.models import PointStruct

logger = logging.getLogger(__name__)


class QdrantClient:
    """Wrapper around qdrant-client with retrieval-specific operations."""

    def __init__(self, url: str, api_key: str, timeout: int = 30):
        """Initialize Qdrant client.

        Args:
            url: Qdrant Cloud URL
            api_key: Qdrant API key
            timeout: Request timeout in seconds (default 30)
        """
        self.url = url
        self.api_key = api_key
        self.timeout = timeout
        self.client = QdrantBaseClient(url=url, api_key=api_key, timeout=timeout)
        logger.info(f"Qdrant client initialized: {url}")

    def search_similar(
        self,
        collection_name: str,
        query_embedding: List[float],
        k: int = 5,
        score_threshold: Optional[float] = None
    ) -> List[Dict]:
        """Execute similarity search against vectors in collection.

        Args:
            collection_name: Name of the Qdrant collection
            query_embedding: 1024-dimensional query embedding vector
            k: Number of top results to return (default 5)
            score_threshold: Optional minimum similarity score threshold [0, 1]

        Returns:
            List of dictionaries with keys:
            - id: Vector ID
            - similarity_score: Cosine similarity score [0, 1]
            - metadata: Dict with url, page_title, chunk_index

        Raises:
            ValueError: If collection doesn't exist or query_embedding dimension mismatch
            ConnectionError: If Qdrant connection fails
        """
        try:
            # Use query_points method with embedding vector
            response = self.client.query_points(
                collection_name=collection_name,
                query=query_embedding,  # Pass embedding as list of floats
                limit=k,
                score_threshold=score_threshold,
                with_payload=True,
                with_vectors=False,
            )

            output = []
            # QueryResponse contains points list
            if hasattr(response, 'points'):
                for result in response.points:
                    metadata = {}
                    if hasattr(result, 'payload') and result.payload:
                        # Payload might have metadata nested or at top level
                        if "metadata" in result.payload:
                            metadata = result.payload["metadata"]
                        else:
                            metadata = result.payload

                    output.append({
                        "id": result.id,
                        "similarity_score": result.score,
                        "metadata": metadata,
                    })

            logger.info(f"Similarity search returned {len(output)}/{k} results")
            return output

        except Exception as e:
            logger.error(f"Similarity search failed: {str(e)}")
            raise

    def get_vectors_by_filter(
        self,
        collection_name: str,
        url_filter: Optional[str] = None,
        limit: int = 100
    ) -> List[Dict]:
        """Retrieve vectors with optional URL filtering.

        Args:
            collection_name: Name of the Qdrant collection
            url_filter: Optional URL to filter by (substring match)
            limit: Maximum number of vectors to return (default 100)

        Returns:
            List of vector dictionaries with id, metadata, embedding

        Raises:
            ConnectionError: If Qdrant connection fails
        """
        try:
            points = self.client.scroll(
                collection_name=collection_name,
                limit=limit,
                with_vectors=False,
                with_payload=True,
            )

            results = []
            for point in points[0]:  # scroll returns (points, next_page_offset)
                metadata = point.payload.get("metadata", {}) if point.payload else {}

                # Filter by URL if specified
                if url_filter and url_filter not in metadata.get("url", ""):
                    continue

                results.append({
                    "id": point.id,
                    "metadata": metadata,
                })

            logger.info(f"Retrieved {len(results)} vectors (limit: {limit})")
            return results

        except Exception as e:
            logger.error(f"Filter retrieval failed: {str(e)}")
            raise

    def get_collection_stats(self, collection_name: str) -> Dict:
        """Get collection statistics and health metrics.

        Args:
            collection_name: Name of the Qdrant collection

        Returns:
            Dictionary with:
            - total_vectors: Total vector count
            - vector_dimension: Vector dimensions (should be 1024)
            - memory_usage_bytes: Approximate memory usage
            - status: Collection status (e.g., "active")

        Raises:
            ConnectionError: If Qdrant connection fails
        """
        try:
            collection = self.client.get_collection(collection_name)

            stats = {
                "total_vectors": collection.points_count,
                "vector_dimension": collection.config.params.vectors.size,
                "memory_usage_bytes": getattr(collection, "memory_usage", 0),
                "status": collection.status.value if hasattr(collection.status, 'value') else str(collection.status),
            }

            logger.info(f"Collection stats: {stats['total_vectors']} vectors, {stats['vector_dimension']}D")
            return stats

        except Exception as e:
            logger.error(f"Failed to get collection stats: {str(e)}")
            raise

    def health_check(self) -> Tuple[bool, str]:
        """Check Qdrant service health.

        Returns:
            Tuple of (is_healthy: bool, status_message: str)
        """
        try:
            health = self.client.get_telemetry()
            return True, "Qdrant service is healthy"
        except Exception as e:
            logger.error(f"Health check failed: {str(e)}")
            return False, f"Health check failed: {str(e)}"
