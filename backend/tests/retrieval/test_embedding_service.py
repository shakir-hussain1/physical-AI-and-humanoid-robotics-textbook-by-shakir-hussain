"""Unit tests for EmbeddingService."""

import pytest
from backend.src.retrieval.embedding_service import EmbeddingService


# Mock Cohere client for testing
class MockCohereClient:
    """Mock Cohere client for testing without API calls."""

    def embed(self, texts, model, input_type):
        """Mock embed method returning 1024-dim vectors."""
        # Return mock embeddings with correct dimensions
        embeddings = []
        for _ in texts:
            embeddings.append([0.1] * 1024)  # 1024-dimensional vector

        class MockResponse:
            pass

        response = MockResponse()
        response.embeddings = embeddings
        return response


@pytest.fixture
def mock_embedding_service():
    """Create EmbeddingService with mock client."""
    service = EmbeddingService(api_key="test-key")
    service.client = MockCohereClient()
    return service


class TestEmbeddingService:
    """Test EmbeddingService functionality."""

    def test_embed_query_returns_1024_dims(self, mock_embedding_service):
        """Test that single query embedding returns 1024-dimensional vector."""
        embedding = mock_embedding_service.embed_query_text("test query")
        assert isinstance(embedding, list)
        assert len(embedding) == 1024

    def test_embed_query_handles_empty_query(self, mock_embedding_service):
        """Test that empty query raises ValueError."""
        with pytest.raises(ValueError):
            mock_embedding_service.embed_query_text("")

    def test_embed_query_handles_whitespace_only(self, mock_embedding_service):
        """Test that whitespace-only query raises ValueError."""
        with pytest.raises(ValueError):
            mock_embedding_service.embed_query_text("   ")

    def test_embed_query_handles_special_chars(self, mock_embedding_service):
        """Test that query with special characters is handled correctly."""
        embedding = mock_embedding_service.embed_query_text("ROS 2.0 @learning!")
        assert isinstance(embedding, list)
        assert len(embedding) == 1024

    def test_embed_batch_queries_returns_all(self, mock_embedding_service):
        """Test that batch embedding returns one vector per query."""
        queries = ["query1", "query2", "query3"]
        embeddings = mock_embedding_service.embed_batch_queries(queries)
        assert len(embeddings) == 3
        assert all(len(emb) == 1024 for emb in embeddings)

    def test_embed_batch_handles_empty_list(self, mock_embedding_service):
        """Test that empty batch raises ValueError."""
        with pytest.raises(ValueError):
            mock_embedding_service.embed_batch_queries([])

    def test_embed_batch_filters_empty_strings(self, mock_embedding_service):
        """Test that batch embedding filters out empty strings."""
        queries = ["valid", "  ", ""]
        # Should work because one valid query is present
        embeddings = mock_embedding_service.embed_batch_queries(queries)
        # Only one valid query provided
        assert len(embeddings) == 1

    def test_verify_embedding_model(self, mock_embedding_service):
        """Test model verification returns True for correct model."""
        is_valid = mock_embedding_service.verify_embedding_model()
        assert is_valid is True
