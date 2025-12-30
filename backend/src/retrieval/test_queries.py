"""Deterministic test query suite for retrieval validation.

Defines 8-10 predefined test queries with expected relevance patterns.
"""

from typing import List, NamedTuple


class TestQuery(NamedTuple):
    """Test query definition with expected outcomes."""

    text: str
    expected_primary_domain: str
    expected_top_keywords: List[str]
    expected_modules: List[str]
    relevance_expectation: str  # "high", "medium", or "low"
    category: str  # "keyword", "phrase", "complex", "edge_case"


# Deterministic test query suite (8 core queries covering diverse topics)
TEST_QUERIES: List[TestQuery] = [
    # Single keywords (Module 1: ROS2 Basics)
    TestQuery(
        text="ROS2",
        expected_primary_domain="robotics communication",
        expected_top_keywords=["ROS2", "middleware", "DDS", "publication", "subscription"],
        expected_modules=["module-1-ros2"],
        relevance_expectation="high",
        category="keyword",
    ),

    # Single keywords (Module 2: Digital Twin)
    TestQuery(
        text="simulation",
        expected_primary_domain="digital twin simulation",
        expected_top_keywords=["Gazebo", "simulation", "physics", "environment"],
        expected_modules=["module-2-digital-twin"],
        relevance_expectation="high",
        category="keyword",
    ),

    # Multi-word phrase (Module 1: Communication)
    TestQuery(
        text="robot communication middleware",
        expected_primary_domain="robotics",
        expected_top_keywords=["ROS2", "communication", "middleware", "publish", "subscribe"],
        expected_modules=["module-1-ros2"],
        relevance_expectation="high",
        category="phrase",
    ),

    # Multi-word phrase (Module 2: Virtual Environment)
    TestQuery(
        text="digital twin virtual environment",
        expected_primary_domain="simulation",
        expected_top_keywords=["Gazebo", "digital twin", "environment", "simulation"],
        expected_modules=["module-2-digital-twin"],
        relevance_expectation="high",
        category="phrase",
    ),

    # Complex query (Module 3: Perception)
    TestQuery(
        text="how to implement perception pipeline for humanoid robot",
        expected_primary_domain="perception and sensing",
        expected_top_keywords=["perception", "sensor", "Isaac", "vision", "pipeline"],
        expected_modules=["module-3-isaac"],
        relevance_expectation="medium",
        category="complex",
    ),

    # Complex query (Module 4: AI/VLA)
    TestQuery(
        text="large language model for robot behavior planning",
        expected_primary_domain="AI and behavior generation",
        expected_top_keywords=["language model", "behavior", "planning", "VLA"],
        expected_modules=["module-4-vla"],
        relevance_expectation="medium",
        category="complex",
    ),

    # Edge case: Single short word
    TestQuery(
        text="a",
        expected_primary_domain="general",
        expected_top_keywords=["general content"],
        expected_modules=["module-1-ros2", "module-2-digital-twin"],
        relevance_expectation="low",
        category="edge_case",
    ),

    # Edge case: Numbers and special characters
    TestQuery(
        text="ROS 2.0 @learning",
        expected_primary_domain="robotics",
        expected_top_keywords=["ROS", "learning", "module"],
        expected_modules=["module-1-ros2"],
        relevance_expectation="medium",
        category="edge_case",
    ),
]


def get_test_queries() -> List[TestQuery]:
    """Get the complete test query suite.

    Returns:
        List of TestQuery objects
    """
    return TEST_QUERIES


def get_test_queries_by_category(category: str) -> List[TestQuery]:
    """Get test queries filtered by category.

    Args:
        category: One of "keyword", "phrase", "complex", "edge_case"

    Returns:
        List of TestQuery objects matching the category
    """
    return [q for q in TEST_QUERIES if q.category == category]


def get_test_queries_by_module(module: str) -> List[TestQuery]:
    """Get test queries relevant to a specific module.

    Args:
        module: Module name (e.g., "module-1-ros2")

    Returns:
        List of TestQuery objects relevant to the module
    """
    return [q for q in TEST_QUERIES if module in q.expected_modules]


def validate_test_query_suite() -> tuple[bool, list[str]]:
    """Validate the test query suite for completeness.

    Returns:
        Tuple of (is_valid: bool, issues: List[str])
    """
    issues = []

    if len(TEST_QUERIES) < 8:
        issues.append(f"Test suite has only {len(TEST_QUERIES)} queries, expected >= 8")

    categories = {"keyword", "phrase", "complex", "edge_case"}
    found_categories = {q.category for q in TEST_QUERIES}
    missing_categories = categories - found_categories
    if missing_categories:
        issues.append(f"Missing query categories: {missing_categories}")

    modules = {"module-1-ros2", "module-2-digital-twin", "module-3-isaac", "module-4-vla"}
    found_modules = set()
    for q in TEST_QUERIES:
        found_modules.update(q.expected_modules)
    missing_modules = modules - found_modules
    if missing_modules:
        issues.append(f"Missing module coverage in test queries: {missing_modules}")

    for i, q in enumerate(TEST_QUERIES):
        if not q.text or not q.text.strip():
            issues.append(f"Query {i}: text is empty")
        if q.relevance_expectation not in ["high", "medium", "low"]:
            issues.append(f"Query {i}: invalid relevance_expectation: {q.relevance_expectation}")

    return len(issues) == 0, issues
