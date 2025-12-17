"""Input validation utilities."""

import logging
from src.config import settings
from src.utils.exceptions import QueryValidationError

logger = logging.getLogger(__name__)


def validate_query(query: str) -> str:
    """Validate and normalize user query."""
    if not query:
        raise QueryValidationError("Query cannot be empty")

    if len(query) < settings.QUERY_MIN_LENGTH:
        raise QueryValidationError(f"Query too short (min {settings.QUERY_MIN_LENGTH} chars)")

    if len(query) > settings.QUERY_MAX_LENGTH:
        raise QueryValidationError(f"Query too long (max {settings.QUERY_MAX_LENGTH} chars)")

    # Check for valid UTF-8
    try:
        query.encode("utf-8").decode("utf-8")
    except UnicodeDecodeError:
        raise QueryValidationError("Query contains invalid UTF-8 characters")

    # Check for null bytes
    if "\0" in query:
        raise QueryValidationError("Query contains null bytes")

    return query.strip()


def detect_prompt_injection(query: str) -> bool:
    """Detect potential prompt injection attempts."""
    # Simple pattern matching for common injection attempts
    injection_patterns = [
        "ignore your",
        "ignore the",
        "system prompt",
        "instructions",
        "you are now",
        "pretend you",
        "forget about",
    ]

    query_lower = query.lower()
    for pattern in injection_patterns:
        if pattern in query_lower:
            logger.warning(f"Potential prompt injection detected: {pattern}")
            return True

    return False


def validate_passage(passage: str) -> str:
    """Validate user-selected passage."""
    if not passage:
        raise QueryValidationError("Passage cannot be empty")

    if len(passage) < 50:
        raise QueryValidationError("Passage too short (min 50 chars)")

    if len(passage) > 5000:
        raise QueryValidationError("Passage too long (max 5000 chars)")

    return passage.strip()
