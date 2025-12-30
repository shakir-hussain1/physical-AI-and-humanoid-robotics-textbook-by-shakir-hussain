"""Validation tests for retrieval pipeline using test query suite."""

import pytest
from backend.src.retrieval.test_queries import (
    get_test_queries,
    validate_test_query_suite,
    get_test_queries_by_category,
    get_test_queries_by_module,
)


class TestQuerySuite:
    """Test query suite definition and coverage."""

    def test_test_query_suite_exists(self):
        """Test that test query suite is defined."""
        queries = get_test_queries()
        assert queries is not None
        assert len(queries) >= 8

    def test_test_query_suite_structure(self):
        """Test that each query has required fields."""
        queries = get_test_queries()
        for query in queries:
            assert hasattr(query, 'text')
            assert hasattr(query, 'expected_primary_domain')
            assert hasattr(query, 'expected_top_keywords')
            assert hasattr(query, 'expected_modules')
            assert hasattr(query, 'relevance_expectation')
            assert hasattr(query, 'category')

    def test_query_text_not_empty(self):
        """Test that all queries have non-empty text."""
        queries = get_test_queries()
        for query in queries:
            assert query.text is not None
            assert len(query.text.strip()) > 0

    def test_query_categories_valid(self):
        """Test that all queries have valid categories."""
        valid_categories = {"keyword", "phrase", "complex", "edge_case"}
        queries = get_test_queries()
        for query in queries:
            assert query.category in valid_categories

    def test_query_relevance_expectation_valid(self):
        """Test that relevance expectations are valid."""
        valid_expectations = {"high", "medium", "low"}
        queries = get_test_queries()
        for query in queries:
            assert query.relevance_expectation in valid_expectations

    def test_test_query_suite_covers_keywords(self):
        """Test that test suite includes keyword queries."""
        keyword_queries = get_test_queries_by_category("keyword")
        assert len(keyword_queries) > 0

    def test_test_query_suite_covers_phrases(self):
        """Test that test suite includes phrase queries."""
        phrase_queries = get_test_queries_by_category("phrase")
        assert len(phrase_queries) > 0

    def test_test_query_suite_covers_complex(self):
        """Test that test suite includes complex queries."""
        complex_queries = get_test_queries_by_category("complex")
        assert len(complex_queries) > 0

    def test_test_query_suite_covers_edge_cases(self):
        """Test that test suite includes edge case queries."""
        edge_queries = get_test_queries_by_category("edge_case")
        assert len(edge_queries) > 0

    def test_test_query_suite_covers_all_modules(self):
        """Test that test suite covers queries for all modules."""
        modules = ["module-1-ros2", "module-2-digital-twin", "module-3-isaac", "module-4-vla"]
        for module in modules:
            module_queries = get_test_queries_by_module(module)
            assert len(module_queries) > 0, f"No queries found for {module}"

    def test_validate_test_query_suite(self):
        """Test that test query suite passes validation."""
        is_valid, issues = validate_test_query_suite()
        assert is_valid is True
        assert len(issues) == 0


class TestQuerySuiteConsistency:
    """Test consistency of query suite data."""

    def test_all_queries_have_keywords(self):
        """Test that all queries have expected keywords defined."""
        queries = get_test_queries()
        for query in queries:
            assert len(query.expected_top_keywords) > 0

    def test_all_queries_have_modules(self):
        """Test that all queries have expected modules defined."""
        queries = get_test_queries()
        for query in queries:
            assert len(query.expected_modules) > 0

    def test_query_module_are_valid(self):
        """Test that module references are valid."""
        valid_modules = {"module-1-ros2", "module-2-digital-twin", "module-3-isaac", "module-4-vla"}
        queries = get_test_queries()
        for query in queries:
            for module in query.expected_modules:
                assert module in valid_modules, f"Invalid module: {module}"

    def test_queries_are_deterministic(self):
        """Test that calling get_test_queries returns consistent results."""
        queries1 = get_test_queries()
        queries2 = get_test_queries()
        assert len(queries1) == len(queries2)
        for q1, q2 in zip(queries1, queries2):
            assert q1.text == q2.text
            assert q1.category == q2.category
