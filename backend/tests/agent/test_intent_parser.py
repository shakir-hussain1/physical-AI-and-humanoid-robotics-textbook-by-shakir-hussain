"""Tests for intent parser."""

import pytest
from backend.src.agent.intent_parser import IntentParser


@pytest.fixture
def parser():
    """Create intent parser instance."""
    return IntentParser()


def test_parse_intent_factual(parser):
    """Test parsing factual questions."""
    intent = parser.parse_intent("What is ROS2?")
    assert intent.query_type == "factual"
    assert intent.primary_topic == "ros"


def test_parse_intent_how_to(parser):
    """Test parsing how-to questions."""
    intent = parser.parse_intent("How do you implement perception?")
    assert intent.query_type == "how_to"
    assert intent.primary_topic == "perception"


def test_parse_intent_conceptual(parser):
    """Test parsing conceptual questions."""
    intent = parser.parse_intent("Why is simulation important?")
    assert intent.query_type == "conceptual"


def test_parse_intent_out_of_domain(parser):
    """Test detecting out-of-domain queries."""
    intent = parser.parse_intent("What is the weather today?")
    assert intent.query_type == "out_of_scope"


def test_extract_topic_ros(parser):
    """Test topic extraction for ROS."""
    intent = parser.parse_intent("How do ROS2 topics work?")
    assert intent.primary_topic == "ros"


def test_extract_topic_simulation(parser):
    """Test topic extraction for simulation."""
    intent = parser.parse_intent("What is the digital twin concept?")
    assert intent.primary_topic == "simulation"


def test_extract_topic_none(parser):
    """Test when no topic detected."""
    intent = parser.parse_intent("What is a general question?")
    assert intent.primary_topic is None


def test_intent_caching(parser):
    """Test that intent results are cached."""
    query = "How does ROS communication work?"
    intent1 = parser.parse_intent(query)
    intent2 = parser.parse_intent(query)
    assert intent1 is intent2  # Same object reference


def test_cache_clear(parser):
    """Test cache clearing."""
    query = "What is simulation?"
    parser.parse_intent(query)
    assert query in parser._intent_cache
    parser.clear_cache()
    assert len(parser._intent_cache) == 0


def test_multiple_domain_keywords(parser):
    """Test detection with multiple keywords."""
    intent = parser.parse_intent("How do ROS2 nodes and topics work together?")
    assert intent.query_type == "how_to"
    assert intent.primary_topic == "ros"


def test_case_insensitivity(parser):
    """Test case-insensitive parsing."""
    intent1 = parser.parse_intent("What is ROS2?")
    intent2 = parser.parse_intent("what is ros2?")
    assert intent1.query_type == intent2.query_type
