"""Request context management for tracing."""
import uuid
from typing import Optional
from contextvars import ContextVar

# Context variable to store trace ID for the current request
_trace_id: ContextVar[Optional[str]] = ContextVar('_trace_id', default=None)

def generate_trace_id() -> str:
    """Generate a new trace ID."""
    return str(uuid.uuid4())

def get_trace_id() -> Optional[str]:
    """Get the current trace ID from context."""
    return _trace_id.get()

def set_trace_id(trace_id: Optional[str] = None) -> str:
    """Set the trace ID in context, generating a new one if not provided."""
    if trace_id is None:
        trace_id = generate_trace_id()

    _trace_id.set(trace_id)
    return trace_id

def clear_trace_id():
    """Clear the current trace ID from context."""
    _trace_id.set(None)