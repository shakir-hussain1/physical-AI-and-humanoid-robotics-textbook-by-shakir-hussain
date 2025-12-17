"""Custom exception classes for RAG Chatbot Backend."""


class QueryValidationError(Exception):
    """Raised when query validation fails."""
    pass


class LowConfidenceError(Exception):
    """Raised when confidence score is below threshold."""
    pass


class RetrieverError(Exception):
    """Raised when retrieval service encounters an error."""
    pass


class GenerationError(Exception):
    """Raised when generation service encounters an error."""
    pass


class TimeoutError(Exception):
    """Raised when operation times out."""
    pass


class HallucinationsDetectedError(Exception):
    """Raised when hallucinations are detected in the response."""
    pass
