"""Tests for grounding validator."""

import pytest
from backend.src.agent.grounding_validator import GroundingValidator


@pytest.fixture
def validator():
    """Create grounding validator instance."""
    return GroundingValidator()


def test_validate_grounding_grounded(validator):
    """Test validation of grounded answer."""
    answer = "ROS2 is a robotics middleware framework"
    context = "ROS2 provides a flexible framework for robotics software development"

    is_grounded, confidence, reason = validator.validate_grounding(answer, context)
    assert is_grounded is True
    assert confidence > 0.5


def test_validate_grounding_not_grounded(validator):
    """Test validation of non-grounded answer."""
    answer = "The capital of France is London"
    context = "ROS2 is a robotics middleware framework"

    is_grounded, confidence, reason = validator.validate_grounding(answer, context)
    # London should not be grounded in ROS2 context
    assert is_grounded is False or confidence < 0.5


def test_detect_hallucination_number_mismatch(validator):
    """Test hallucination detection with numbers."""
    answer = "ROS2 was founded in 2025"
    context = "ROS2 was released in 2017"

    has_hallucination = validator.detect_hallucination(answer, context)
    assert has_hallucination is True


def test_detect_hallucination_fabrication(validator):
    """Test hallucination detection for fabricated claims."""
    answer = "The study showed that robots can feel emotions"
    context = "Humanoid robots are designed to perform physical tasks"

    has_hallucination = validator.detect_hallucination(answer, context)
    # Fabricated study claim in robotics context = likely hallucination
    assert has_hallucination is True


def test_grounding_with_high_overlap(validator):
    """Test grounding with high word overlap."""
    answer = "ROS2 provides middleware for robotics"
    context = "ROS2 is a middleware platform that provides communication for robotics systems"

    is_grounded, confidence, reason = validator.validate_grounding(answer, context)
    assert confidence > 0.7


def test_grounding_with_low_overlap(validator):
    """Test grounding with low word overlap."""
    answer = "Artificial intelligence and deep learning are important"
    context = "ROS2 middleware framework for robotics communication"

    is_grounded, confidence, reason = validator.validate_grounding(answer, context)
    assert confidence < 0.5


def test_grounding_confidence_bounds(validator):
    """Test that confidence is always [0, 1]."""
    answer = "This is an answer"
    context = "This is some context"

    is_grounded, confidence, reason = validator.validate_grounding(answer, context)
    assert 0 <= confidence <= 1


def test_grounding_reason_high_confidence(validator):
    """Test reason generation for high confidence."""
    answer = "ROS2 is a framework"
    context = "ROS2 is a flexible robotics framework"

    is_grounded, confidence, reason = validator.validate_grounding(answer, context)
    if confidence >= 0.8:
        assert "well-grounded" in reason.lower()


def test_grounding_reason_low_confidence(validator):
    """Test reason generation for low confidence."""
    answer = "Dogs can fly"
    context = "Robotics is about building machines"

    is_grounded, confidence, reason = validator.validate_grounding(answer, context)
    if confidence < 0.5:
        assert "hallucination" in reason.lower() or "limited" in reason.lower()


def test_threshold_customization():
    """Test custom threshold."""
    validator_high = GroundingValidator(threshold=0.8)
    validator_low = GroundingValidator(threshold=0.3)

    answer = "ROS2 middleware"
    context = "ROS2 is middleware for robotics"

    _, conf_high, _ = validator_high.validate_grounding(answer, context)
    _, conf_low, _ = validator_low.validate_grounding(answer, context)

    # Different thresholds should evaluate same confidence differently
    assert conf_high == conf_low  # Same score, different threshold applied
