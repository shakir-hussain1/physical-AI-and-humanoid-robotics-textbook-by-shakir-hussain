"""Request validation utilities."""

from typing import Any, Dict, List, Optional, Tuple


def validate_query_length(query: str, max_length: int = 10000) -> Tuple[bool, Optional[str]]:
    """Validate query is within acceptable length."""
    if not query or not query.strip():
        return False, "Query cannot be empty"
    if len(query) > max_length:
        return False, f"Query exceeds maximum length of {max_length} characters"
    return True, None


def validate_history_format(history: Optional[List[Dict[str, Any]]]) -> Tuple[bool, Optional[str]]:
    """Validate conversation history has correct format."""
    if history is None:
        return True, None

    if not isinstance(history, list):
        return False, "Conversation history must be a list of message objects"

    for i, msg in enumerate(history):
        if not isinstance(msg, dict):
            return False, f"History item {i} must be a dict, got {type(msg).__name__}"

        if "role" not in msg or "content" not in msg:
            return False, f"History item {i} must have 'role' and 'content' fields"

        if not isinstance(msg["role"], str):
            return False, f"History item {i}: role must be a string"

        if not isinstance(msg["content"], str):
            return False, f"History item {i}: content must be a string"

        if msg["role"] not in ["user", "assistant"]:
            return False, f"History item {i}: invalid role '{msg['role']}' (must be 'user' or 'assistant')"

    return True, None


def validate_k_range(k: int, min_k: int = 1, max_k: int = 20) -> Tuple[bool, Optional[str]]:
    """Validate k parameter is within acceptable range."""
    if not isinstance(k, int):
        return False, f"k must be an integer, got {type(k).__name__}"

    if k < min_k or k > max_k:
        return False, f"k must be between {min_k} and {max_k}, got {k}"

    return True, None


def validate_selected_text(text: str, max_length: int = 5000) -> Tuple[bool, Optional[str]]:
    """Validate selected text is within acceptable length."""
    if not text or not text.strip():
        return False, "Selected text cannot be empty"

    if len(text) > max_length:
        return False, f"Selected text exceeds maximum length of {max_length} characters"

    return True, None


def validate_user_role(role: str) -> Tuple[bool, Optional[str]]:
    """Validate user role is one of the allowed values."""
    allowed_roles = ["student", "teacher", "researcher"]

    if role not in allowed_roles:
        return False, f"User role must be one of: {', '.join(allowed_roles)}, got '{role}'"

    return True, None
