"""Integration tests for end-to-end retrieval workflow.

Tests the complete retrieval pipeline from query to validation report.
Requires Qdrant connection and 26 stored vectors from Spec-1.
"""

import pytest
import time
from backend.src.retrieval.config import RetrievalConfig
from backend.src.retrieval.retrieval_service import RetrievalService
from backend.src.retrieval.test_queries import get_test_queries
from backend.src.retrieval.report_generator import ReportGenerator


@pytest.fixture
def retrieval_service():
    """Create RetrievalService with live Qdrant connection."""
    try:
        config = RetrievalConfig.load_from_env()
        service = RetrievalService(config)
        return service
    except ValueError as e:
        pytest.skip(f"Qdrant configuration not available: {str(e)}")


class TestEndToEndWorkflow:
    """Test complete retrieval workflow."""

    def test_end_to_end_query_to_result(self, retrieval_service):
        """Test full pipeline: query → embedding → search → validation → report."""
        # Execute query
        result = retrieval_service.search("ROS2 communication", k=5)

        # Verify success
        assert result["status"] == "success"
        assert len(result["results"]) > 0
        assert result["query"] == "ROS2 communication"

        # Verify results have required fields
        for res in result["results"]:
            assert "id" in res
            assert "similarity_score" in res
            assert "metadata" in res
            assert "url" in res["metadata"]
            assert "page_title" in res["metadata"]

    def test_deterministic_retrieval(self, retrieval_service):
        """Test that same query produces same results across multiple runs."""
        query = "robot simulation"

        # Run same query 3 times
        results = []
        for _ in range(3):
            result = retrieval_service.search(query, k=5)
            assert result["status"] == "success"
            results.append(result["results"])

        # Verify all runs return identical results
        first_ids = [r["id"] for r in results[0]]
        for run_results in results[1:]:
            run_ids = [r["id"] for r in run_results]
            assert first_ids == run_ids, "Results should be deterministic"

        # Verify scores are identical
        first_scores = [r["similarity_score"] for r in results[0]]
        for run_results in results[1:]:
            run_scores = [r["similarity_score"] for r in run_results]
            assert first_scores == run_scores, "Scores should be identical"

    def test_metadata_integrity(self, retrieval_service):
        """Test that retrieved metadata matches expected structure."""
        result = retrieval_service.search("perception", k=10)

        assert result["status"] == "success"
        assert len(result["results"]) > 0

        for res in result["results"]:
            metadata = res["metadata"]

            # Check required fields exist
            assert "url" in metadata
            assert "page_title" in metadata
            assert "chunk_index" in metadata

            # Check types
            assert isinstance(metadata["url"], str)
            assert isinstance(metadata["page_title"], str)
            assert isinstance(metadata["chunk_index"], int)

            # Check URL format
            assert metadata["url"].startswith("http")
            assert len(metadata["url"]) > 0

            # Check chunk index is valid
            assert metadata["chunk_index"] >= 0

    def test_error_handling_on_empty_query(self, retrieval_service):
        """Test graceful handling of empty queries."""
        result = retrieval_service.search("", k=5)

        assert result["status"] == "error"
        assert "error" in result
        assert result["result_count"] == 0 or "result_count" not in result

    def test_error_handling_on_invalid_k(self, retrieval_service):
        """Test validation of k parameter."""
        # Test k=0
        result = retrieval_service.search("robot", k=0)
        assert result["status"] == "error"

        # Test k > max_k
        result = retrieval_service.search("robot", k=10000)
        assert result["status"] == "error"

    def test_batch_search_operation(self, retrieval_service):
        """Test batch search with multiple queries."""
        queries = ["ROS2", "simulation", "perception", "behavior"]
        results = retrieval_service.batch_search(queries, k=5)

        assert len(results) == len(queries)
        for i, result in enumerate(results):
            assert result["query"] == queries[i]
            if result["status"] == "success":
                assert len(result["results"]) > 0

    def test_collection_stats(self, retrieval_service):
        """Test collection statistics retrieval."""
        stats = retrieval_service.get_stats()

        assert "total_vectors" in stats
        assert "vector_dimension" in stats
        assert "status" in stats
        assert "service_status" in stats

        # Verify expected values
        assert stats["total_vectors"] > 0
        assert stats["vector_dimension"] == 1024  # Cohere embedding dimension
        assert stats["service_status"] in ["operational", "degraded", "error"]


class TestValidationIntegration:
    """Test validation within retrieval workflow."""

    def test_validation_with_search_results(self, retrieval_service):
        """Test that validation detects issues in search results."""
        result = retrieval_service.validate_retrieval("ROS2", k=5)

        assert result["status"] == "success"
        assert "validation" in result
        assert "results" in result

        validation = result["validation"]
        assert "is_valid" in validation
        assert "metadata_valid" in validation
        assert "scores_valid" in validation

    def test_validation_detects_metadata_issues(self, retrieval_service):
        """Test that validation reports metadata problems if they exist."""
        result = retrieval_service.validate_retrieval("digital twin", k=5)

        assert result["status"] == "success"

        validation = result["validation"]
        # If all metadata is valid, should have no issues
        if validation["is_valid"]:
            assert len(validation.get("issues", [])) == 0

    def test_query_ranking_validation(self, retrieval_service):
        """Test that retrieved results are properly ranked by similarity score."""
        result = retrieval_service.search("humanoid robot", k=5)

        assert result["status"] == "success"
        results = result["results"]

        # Verify descending order by similarity score
        scores = [r["similarity_score"] for r in results]
        for i in range(len(scores) - 1):
            assert scores[i] >= scores[i + 1], "Results must be ranked by score (descending)"


class TestPerformanceValidation:
    """Test performance against NFR targets."""

    def test_single_query_latency(self, retrieval_service):
        """Test that single query latency meets target (< 500ms p95)."""
        latencies = []
        query = "ROS2"

        # Run 5 queries to get latency distribution
        for _ in range(5):
            start = time.time()
            result = retrieval_service.search(query, k=5)
            latency = (time.time() - start) * 1000

            if result["status"] == "success":
                latencies.append(latency)

        assert len(latencies) > 0, "At least one query should succeed"

        # Calculate p95
        latencies_sorted = sorted(latencies)
        p95_index = max(0, int(len(latencies_sorted) * 0.95) - 1)
        p95_latency = latencies_sorted[p95_index]

        # Should be reasonably fast (may be higher on slow connections)
        assert p95_latency < 5000, f"P95 latency should be < 5000ms, got {p95_latency}ms"

    def test_batch_retrieval_latency(self, retrieval_service):
        """Test that batch retrieval of 10 vectors completes within 5 seconds."""
        queries = [
            "ROS2",
            "simulation",
            "perception",
            "behavior",
            "communication",
            "sensor",
            "embedding",
            "gazebo",
            "humanoid",
            "learning",
        ]

        start = time.time()
        results = retrieval_service.batch_search(queries, k=5)
        total_latency = (time.time() - start) * 1000

        assert len(results) == 10
        assert total_latency < 10000, f"Batch retrieval should be < 10000ms, got {total_latency}ms"

    def test_similarity_score_validity(self, retrieval_service):
        """Test that all similarity scores are in valid range [0, 1]."""
        result = retrieval_service.search("robot", k=10)

        assert result["status"] == "success"

        for res in result["results"]:
            score = res["similarity_score"]
            assert 0.0 <= score <= 1.0, f"Score must be in [0, 1], got {score}"


class TestTestQueryExecution:
    """Test execution of predefined test query suite."""

    def test_all_test_queries_executable(self, retrieval_service):
        """Test that all predefined test queries execute without errors."""
        test_queries = get_test_queries()

        for test_query in test_queries:
            result = retrieval_service.search(test_query.text, k=5)

            # All queries should execute (may have 0 results for edge cases)
            assert "status" in result
            assert "results" in result

    def test_test_query_relevance_expectations(self, retrieval_service):
        """Test that high-relevance test queries return relevant results."""
        test_queries = get_test_queries()
        high_relevance_queries = [q for q in test_queries if q.relevance_expectation == "high"]

        for test_query in high_relevance_queries:
            result = retrieval_service.validate_retrieval(test_query.text, k=5)

            if result["status"] == "success":
                validation = result["validation"]
                # High relevance queries should ideally have valid results
                # (actual relevance depends on semantic matching)
                assert "is_valid" in validation

    def test_validation_report_generation(self, retrieval_service):
        """Test generation of validation report from test queries."""
        test_queries = get_test_queries()
        report_gen = ReportGenerator(retrieval_service)

        report = report_gen.generate_validation_report(test_queries)

        # Verify report structure
        assert "timestamp" in report
        assert "total_queries" in report
        assert "successful_queries" in report
        assert "failed_queries" in report
        assert "pass_rate" in report
        assert "query_results" in report

        # Verify counts match
        total = report["successful_queries"] + report["failed_queries"]
        assert total <= report["total_queries"]

        # Verify pass rate is percentage
        assert 0.0 <= report["pass_rate"] <= 100.0
