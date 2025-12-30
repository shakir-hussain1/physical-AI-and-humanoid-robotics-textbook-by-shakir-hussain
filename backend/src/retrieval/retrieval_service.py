"""Main Retrieval Service for RAG pipeline.

Provides high-level API for querying, validating, and reporting on retrieved vectors.
"""

import logging
import time
from typing import List, Dict, Optional
from .qdrant_client import QdrantClient
from .embedding_service import EmbeddingService
from .validator import MetadataValidator, ContentVerifier, RetrievalValidator
from .config import RetrievalConfig

logger = logging.getLogger(__name__)


class RetrievalService:
    """Main retrieval service orchestrating all retrieval operations."""

    def __init__(self, config: RetrievalConfig):
        """Initialize retrieval service with Qdrant and Cohere clients.

        Args:
            config: RetrievalConfig instance with required credentials

        Raises:
            ValueError: If configuration is invalid
        """
        self.config = config

        # Initialize clients
        self.qdrant = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            timeout=config.request_timeout,
        )

        self.embedding_service = EmbeddingService(
            api_key=config.cohere_api_key,
            model=config.cohere_model,
        )

        logger.info("RetrievalService initialized successfully")

    def search(self, query: str, k: Optional[int] = None) -> Dict:
        """Execute similarity search for a query.

        Args:
            query: Query text to search for
            k: Number of top results to return (default: config.default_k)

        Returns:
            Dictionary with:
            - query: Original query text
            - results: List of results with id, score, metadata
            - latency_ms: Query execution time in milliseconds
            - status: "success" or "error"
            - error: Error message if failed

        Raises:
            ValueError: If query is invalid or k is out of bounds
        """
        if not query or not query.strip():
            return {
                "query": query,
                "results": [],
                "status": "error",
                "error": "Query cannot be empty",
                "latency_ms": 0,
            }

        k = k or self.config.default_k
        if k < 1 or k > self.config.max_k:
            return {
                "query": query,
                "results": [],
                "status": "error",
                "error": f"k must be between 1 and {self.config.max_k}",
                "latency_ms": 0,
            }

        start_time = time.time()

        try:
            # Generate embedding for query
            query_embedding = self.embedding_service.embed_query_text(query)

            # Execute similarity search
            results = self.qdrant.search_similar(
                collection_name=self.config.qdrant_collection,
                query_embedding=query_embedding,
                k=k,
            )

            latency_ms = (time.time() - start_time) * 1000

            return {
                "query": query,
                "results": results,
                "result_count": len(results),
                "status": "success",
                "latency_ms": round(latency_ms, 2),
            }

        except Exception as e:
            latency_ms = (time.time() - start_time) * 1000
            logger.error(f"Search failed for query '{query}': {str(e)}")
            return {
                "query": query,
                "results": [],
                "status": "error",
                "error": str(e),
                "latency_ms": round(latency_ms, 2),
            }

    def batch_search(self, queries: List[str], k: Optional[int] = None) -> List[Dict]:
        """Execute similarity search for multiple queries.

        Args:
            queries: List of query texts
            k: Number of top results per query (default: config.default_k)

        Returns:
            List of search result dictionaries (one per query)
        """
        if not queries:
            logger.warning("batch_search called with empty query list")
            return []

        results = []
        for query in queries:
            result = self.search(query, k=k)
            results.append(result)

        logger.info(f"Batch search completed for {len(queries)} queries")
        return results

    def validate_retrieval(self, query: str, k: Optional[int] = None) -> Dict:
        """Execute query and validate results for correctness.

        Args:
            query: Query text to validate
            k: Number of results to validate

        Returns:
            Dictionary with:
            - query: Original query
            - results: Retrieved results
            - validation: Validation report with pass/fail status
            - status: "success" or "error"
        """
        if not query or not query.strip():
            return {
                "query": query,
                "status": "error",
                "error": "Query cannot be empty",
            }

        k = k or self.config.default_k

        try:
            # Execute search
            search_result = self.search(query, k=k)

            if search_result["status"] != "success":
                return {
                    "query": query,
                    "status": "error",
                    "error": search_result.get("error", "Search failed"),
                }

            results = search_result["results"]

            # Validate results
            validation_report = RetrievalValidator.validate_query_results(
                query=query,
                results=results,
                k=k,
            )

            # Validate metadata consistency for each result
            for i, result in enumerate(results):
                metadata = result.get("metadata", {})
                is_consistent, errors = MetadataValidator.validate_metadata_consistency(metadata)
                if not is_consistent:
                    validation_report["issues"].extend(
                        [f"Result {i}: {err}" for err in errors]
                    )
                    validation_report["is_valid"] = False

            return {
                "query": query,
                "results": results,
                "validation": validation_report,
                "status": "success",
                "latency_ms": search_result.get("latency_ms", 0),
            }

        except Exception as e:
            logger.error(f"Validation failed for query '{query}': {str(e)}")
            return {
                "query": query,
                "status": "error",
                "error": str(e),
            }

    def get_stats(self) -> Dict:
        """Get collection statistics and service health.

        Returns:
            Dictionary with:
            - total_vectors: Total vector count in collection
            - vector_dimension: Dimensionality of vectors (should be 1024)
            - memory_usage_bytes: Memory used by collection
            - status: Collection status
            - qdrant_health: Qdrant service health status
            - service_status: Overall service status
        """
        try:
            # Get collection stats
            collection_stats = self.qdrant.get_collection_stats(
                self.config.qdrant_collection
            )

            # Check Qdrant health
            qdrant_healthy, health_msg = self.qdrant.health_check()

            return {
                "collection": self.config.qdrant_collection,
                "total_vectors": collection_stats.get("total_vectors", 0),
                "vector_dimension": collection_stats.get("vector_dimension", 0),
                "memory_usage_bytes": collection_stats.get("memory_usage_bytes", 0),
                "status": collection_stats.get("status", "unknown"),
                "qdrant_health": "healthy" if qdrant_healthy else "unhealthy",
                "service_status": "operational" if qdrant_healthy else "degraded",
            }

        except Exception as e:
            logger.error(f"Failed to get stats: {str(e)}")
            return {
                "service_status": "error",
                "error": str(e),
            }
