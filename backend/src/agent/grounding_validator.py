"""Validate that responses are grounded in retrieved context."""

from typing import Tuple, List


class GroundingValidator:
    """Detect hallucinations and validate response grounding."""

    def __init__(self, threshold: float = 0.5):
        """Initialize grounding validator.

        Args:
            threshold: Confidence threshold for grounding (default 0.5).
        """
        self.threshold = threshold

    def validate_grounding(
        self, answer: str, context: str
    ) -> Tuple[bool, float, str]:
        """Validate that answer is grounded in context.

        Args:
            answer: Generated answer text.
            context: Retrieved context.

        Returns:
            Tuple of (is_grounded, confidence_score [0-1], reason).
        """
        # Simple heuristic: check for key entities/numbers in answer that appear in context
        answer_words = set(answer.lower().split())
        context_words = set(context.lower().split())

        # Count overlapping words
        overlap = answer_words & context_words
        overlap_ratio = len(overlap) / len(answer_words) if answer_words else 0

        # Check for obvious hallucinations (specific dates/names not in context)
        hallucination_score = self._detect_hallucination(answer, context)

        # Combined score
        confidence = (overlap_ratio * 0.6) + ((1 - hallucination_score) * 0.4)

        is_grounded = confidence >= self.threshold and hallucination_score < 0.5

        reason = self._generate_reason(confidence, overlap_ratio, hallucination_score)

        return is_grounded, confidence, reason

    def detect_hallucination(self, answer: str, context: str) -> bool:
        """Detect obvious hallucinations.

        Args:
            answer: Generated answer.
            context: Retrieved context.

        Returns:
            True if hallucination detected, False otherwise.
        """
        score = self._detect_hallucination(answer, context)
        return score > 0.7

    def _detect_hallucination(self, answer: str, context: str) -> float:
        """Internal hallucination detection score.

        Args:
            answer: Generated answer.
            context: Retrieved context.

        Returns:
            Hallucination score [0-1].
        """
        # Check for numbers in answer that don't appear in context
        import re

        answer_numbers = set(re.findall(r'\d+', answer))
        context_numbers = set(re.findall(r'\d+', context))

        if answer_numbers and not answer_numbers <= context_numbers:
            # Numbers in answer not in context = likely hallucination
            return 0.8

        # Check for specific fabrication patterns
        fabrication_patterns = [
            r'according to.*said',
            r'the study showed',
            r'research proved',
        ]

        for pattern in fabrication_patterns:
            if re.search(pattern, answer.lower()) and pattern not in context.lower():
                return 0.7

        return 0.2

    def _generate_reason(
        self, confidence: float, overlap_ratio: float, hallucination_score: float
    ) -> str:
        """Generate human-readable reason for grounding decision.

        Args:
            confidence: Overall confidence score.
            overlap_ratio: Word overlap ratio.
            hallucination_score: Hallucination likelihood.

        Returns:
            Reason string.
        """
        if confidence >= 0.8:
            return "Answer is well-grounded in context"
        elif confidence >= 0.5:
            return "Answer is partially grounded; some claims not fully supported"
        else:
            return "Answer has limited grounding; high risk of hallucination"
