"""Integration tests for agent workflow."""

import pytest
from backend.src.agent import (
    AgentOrchestrator,
    AgentConfig,
    IntentParser,
    ContextConstructor,
    PromptBuilder,
    GroundingValidator,
    ResponseFormatter,
)


@pytest.fixture
def config():
    """Create agent configuration."""
    import os

    os.environ.setdefault("OPENAI_API_KEY", "test-key")
    return AgentConfig()


@pytest.fixture
def orchestrator(config):
    """Create agent orchestrator."""
    return AgentOrchestrator(config)


def test_intent_parser_component(orchestrator):
    """Test intent parser component."""
    intent = orchestrator.intent_parser.parse_intent("What is ROS2?")
    assert intent is not None
    assert intent.query_type in ["factual", "conceptual", "how_to", "clarification", "out_of_scope"]


def test_context_constructor_component(orchestrator):
    """Test context constructor component."""
    chunks = [
        {
            "text": "ROS2 is a middleware framework",
            "similarity_score": 0.9,
            "metadata": {"url": "http://example.com", "page_title": "ROS2 Intro"},
        }
    ]

    context = orchestrator.context_constructor.construct_context(chunks, "What is ROS2?")
    assert "ROS2" in context or "middleware" in context


def test_prompt_builder_system_prompt(orchestrator):
    """Test system prompt building."""
    prompt = orchestrator.prompt_builder.build_system_prompt("student")
    assert "answer" in prompt.lower()
    assert "context" in prompt.lower()


def test_prompt_builder_user_prompt(orchestrator):
    """Test user prompt building."""
    context = "ROS2 is a middleware framework"
    query = "What is ROS2?"

    prompt = orchestrator.prompt_builder.build_user_prompt(query, context)
    assert query in prompt
    assert context in prompt


def test_grounding_validator_component(orchestrator):
    """Test grounding validator component."""
    answer = "ROS2 is a framework"
    context = "ROS2 is a middleware framework for robotics"

    is_grounded, confidence, reason = orchestrator.grounding_validator.validate_grounding(
        answer, context
    )
    assert isinstance(is_grounded, bool)
    assert 0 <= confidence <= 1


def test_response_formatter_component(orchestrator):
    """Test response formatter component."""
    answer = "ROS2 is middleware"
    sources = [
        {
            "url": "http://example.com",
            "page_title": "ROS2",
            "relevance_score": 0.9,
            "chunk_index": 0,
        }
    ]

    response = orchestrator.response_formatter.format_response(
        answer, sources, "high"
    )
    assert response.answer == answer
    assert len(response.sources) > 0
    assert response.confidence == "high"


def test_out_of_domain_detection(orchestrator):
    """Test out-of-domain query handling."""
    response = orchestrator.query("What is the weather today?")
    assert response is not None
    assert response.confidence == "high"  # Clear error case
    assert "cannot answer" in response.answer.lower() or "textbook" in response.answer.lower()


def test_component_initialization(orchestrator):
    """Test that all components are initialized."""
    assert orchestrator.intent_parser is not None
    assert orchestrator.context_constructor is not None
    assert orchestrator.prompt_builder is not None
    assert orchestrator.grounding_validator is not None
    assert orchestrator.response_formatter is not None
    assert orchestrator.monitor is not None
    assert orchestrator.error_handler is not None


def test_query_returns_agent_response(orchestrator):
    """Test that query returns valid AgentResponse."""
    response = orchestrator.query("What is ROS2?")
    assert response is not None
    assert hasattr(response, "answer")
    assert hasattr(response, "confidence")
    assert hasattr(response, "metadata")


def test_conversation_history_handling(orchestrator):
    """Test conversation history support."""
    from backend.src.agent import Message

    history = [
        Message(role="user", content="What is ROS2?"),
        Message(role="assistant", content="ROS2 is middleware"),
    ]

    response = orchestrator.query("Can you explain more?", conversation_history=history)
    assert response is not None


def test_multiple_queries(orchestrator):
    """Test handling multiple queries."""
    queries = [
        "What is ROS2?",
        "How does simulation work?",
        "What is a digital twin?",
    ]

    for query in queries:
        response = orchestrator.query(query)
        assert response is not None
        assert isinstance(response.answer, str)
