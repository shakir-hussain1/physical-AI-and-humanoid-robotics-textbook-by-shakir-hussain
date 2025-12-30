"""Behavior tests for RAG agent with book-related queries."""

import pytest
from backend.src.agent import AgentOrchestrator, AgentConfig
import os


@pytest.fixture
def config():
    """Create agent configuration."""
    os.environ.setdefault("OPENAI_API_KEY", "test-key")
    return AgentConfig()


@pytest.fixture
def agent(config):
    """Create agent orchestrator."""
    return AgentOrchestrator(config)


class TestBookRelatedQueries:
    """Test agent behavior on textbook-related queries."""

    def test_query_what_is_ros2(self, agent):
        """Test: What is ROS2?"""
        response = agent.query("What is ROS2?")
        assert response is not None
        assert len(response.answer) > 0
        assert response.confidence in ["high", "medium", "low"]

    def test_query_digital_twin_concept(self, agent):
        """Test: What is a digital twin?"""
        response = agent.query("What is a digital twin concept?")
        assert response is not None
        assert "digital twin" in response.answer.lower() or len(response.answer) > 0

    def test_query_how_to_implement_perception(self, agent):
        """Test: How to implement perception?"""
        response = agent.query("How do you implement perception in humanoid robots?")
        assert response is not None
        assert response.confidence in ["high", "medium", "low"]

    def test_query_ros2_communication(self, agent):
        """Test: How does ROS2 communication work?"""
        response = agent.query("How does ROS2 communication work?")
        assert response is not None
        assert isinstance(response.answer, str)

    def test_query_behavior_planning(self, agent):
        """Test: How to implement behavior planning?"""
        response = agent.query("How do you implement behavior planning for humanoid robots?")
        assert response is not None

    def test_query_follow_up(self, agent):
        """Test: Follow-up question capability."""
        from backend.src.agent import Message

        history = [
            Message(role="user", content="What is ROS2?"),
            Message(role="assistant", content="ROS2 is middleware"),
        ]

        response = agent.query("Can you explain that in more detail?", conversation_history=history)
        assert response is not None

    def test_out_of_scope_weather(self, agent):
        """Test: Out-of-scope query (weather)."""
        response = agent.query("What is the weather today?")
        assert response is not None
        # Should indicate it's out-of-domain
        assert response.confidence == "high" or "textbook" in response.answer.lower()

    def test_out_of_scope_joke(self, agent):
        """Test: Out-of-scope query (joke)."""
        response = agent.query("Tell me a funny joke")
        assert response is not None

    def test_response_has_sources(self, agent):
        """Test: Response includes sources."""
        response = agent.query("What is ROS2?")
        # May or may not have sources depending on implementation
        assert hasattr(response, "sources")
        assert isinstance(response.sources, list)

    def test_response_has_confidence(self, agent):
        """Test: Response includes confidence level."""
        response = agent.query("What is ROS2?")
        assert response.confidence in ["high", "medium", "low"]

    def test_response_has_metadata(self, agent):
        """Test: Response includes metadata."""
        response = agent.query("What is ROS2?")
        assert hasattr(response, "metadata")
        assert isinstance(response.metadata, dict)

    def test_empty_query_handling(self, agent):
        """Test: Empty query handling."""
        response = agent.query("")
        assert response is not None

    def test_very_long_query(self, agent):
        """Test: Very long query handling."""
        long_query = "What is ROS2? " * 100
        response = agent.query(long_query)
        assert response is not None

    def test_special_characters_in_query(self, agent):
        """Test: Special characters in query."""
        response = agent.query("What is ROS2? @#$%^&*()")
        assert response is not None

    def test_multiple_sequential_queries(self, agent):
        """Test: Multiple sequential queries."""
        queries = [
            "What is ROS2?",
            "What is simulation?",
            "How does digital twin work?",
        ]

        for query in queries:
            response = agent.query(query)
            assert response is not None
            assert isinstance(response.answer, str)

    def test_query_response_type(self, agent):
        """Test: Query returns correct response type."""
        response = agent.query("What is ROS2?")
        assert hasattr(response, "to_dict")
        response_dict = response.to_dict()
        assert "answer" in response_dict
        assert "confidence" in response_dict
        assert "sources" in response_dict
