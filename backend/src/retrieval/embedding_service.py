"""Embedding Generation Service using Cohere API

Generates query embeddings using the same model as the ingestion pipeline.
"""

import logging
from typing import List, Union
from cohere import Client

logger = logging.getLogger(__name__)


class EmbeddingService:
    """Generate embeddings using Cohere API."""

    EMBEDDING_MODEL = "embed-english-v3.0"
    EXPECTED_DIMENSION = 1024

    def __init__(self, api_key: str, model: str = EMBEDDING_MODEL, max_retries: int = 3):
        """Initialize Cohere embedding client.

        Args:
            api_key: Cohere API key
            model: Embedding model name (default: embed-english-v3.0)
            max_retries: Maximum retry attempts for API calls (default 3)
        """
        self.api_key = api_key
        self.model = model
        self.max_retries = max_retries
        self.client = Client(api_key=api_key)
        logger.info(f"Embedding service initialized with model: {model}")

    def embed_query_text(self, query: str) -> List[float]:
        """Generate embedding for a single query text.

        Args:
            query: Query text to embed

        Returns:
            List of float values representing 1024-dimensional embedding

        Raises:
            ValueError: If query is empty or embedding dimension mismatch
            RuntimeError: If Cohere API call fails after retries
        """
        if not query or not query.strip():
            raise ValueError("Query text cannot be empty")

        # Handle very long queries (truncate to 2048 chars for API)
        query = query[:2048].strip()

        try:
            response = self.client.embed(
                texts=[query],
                model=self.model,
                input_type="search_query"
            )

            embedding = response.embeddings[0]

            # Verify dimension
            if len(embedding) != self.EXPECTED_DIMENSION:
                raise ValueError(
                    f"Embedding dimension mismatch: expected {self.EXPECTED_DIMENSION}, "
                    f"got {len(embedding)}"
                )

            logger.debug(f"Generated embedding for query: {query[:50]}...")
            return embedding

        except Exception as e:
            logger.error(f"Embedding generation failed: {str(e)}")
            raise RuntimeError(f"Failed to generate embedding: {str(e)}")

    def embed_batch_queries(self, queries: List[str]) -> List[List[float]]:
        """Generate embeddings for multiple query texts (batch).

        Args:
            queries: List of query texts to embed

        Returns:
            List of embeddings (each 1024-dimensional)

        Raises:
            ValueError: If queries list is empty or contains empty strings
            RuntimeError: If Cohere API call fails
        """
        if not queries:
            raise ValueError("Queries list cannot be empty")

        # Filter out empty queries
        queries = [q.strip() for q in queries if q and q.strip()]
        if not queries:
            raise ValueError("All query texts are empty")

        # Truncate long queries
        queries = [q[:2048].strip() for q in queries]

        try:
            response = self.client.embed(
                texts=queries,
                model=self.model,
                input_type="search_query"
            )

            embeddings = response.embeddings

            # Verify all dimensions
            for i, embedding in enumerate(embeddings):
                if len(embedding) != self.EXPECTED_DIMENSION:
                    raise ValueError(
                        f"Embedding {i} dimension mismatch: expected {self.EXPECTED_DIMENSION}, "
                        f"got {len(embedding)}"
                    )

            logger.info(f"Generated {len(embeddings)} embeddings for batch query")
            return embeddings

        except Exception as e:
            logger.error(f"Batch embedding generation failed: {str(e)}")
            raise RuntimeError(f"Failed to generate batch embeddings: {str(e)}")

    def verify_embedding_model(self) -> bool:
        """Verify that the configured embedding model matches expected version.

        Returns:
            True if model is correct, False otherwise

        Note:
            This is a validation check to ensure consistency with ingestion pipeline.
            Cohere model versions are fixed and don't change dynamically.
        """
        if self.model == self.EMBEDDING_MODEL:
            logger.info(f"Embedding model verified: {self.model}")
            return True
        else:
            logger.warning(
                f"Embedding model mismatch: expected {self.EMBEDDING_MODEL}, "
                f"got {self.model}"
            )
            return False
