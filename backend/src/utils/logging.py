"""Structured JSON logging configuration."""

import json
import logging
import sys
import time
from datetime import datetime
from typing import Any, Optional


class JSONFormatter(logging.Formatter):
    """JSON formatter for structured logging."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON."""
        log_data = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }

        # Add request context if available
        if hasattr(record, "request_id"):
            log_data["request_id"] = record.request_id
        if hasattr(record, "endpoint"):
            log_data["endpoint"] = record.endpoint
        if hasattr(record, "method"):
            log_data["method"] = record.method
        if hasattr(record, "status_code"):
            log_data["status_code"] = record.status_code
        if hasattr(record, "latency_ms"):
            log_data["latency_ms"] = record.latency_ms

        # Add exception info if present
        if record.exc_info:
            log_data["exception"] = self.formatException(record.exc_info)

        return json.dumps(log_data)


def configure_logging(level: str = "INFO", format_type: str = "json") -> logging.Logger:
    """Configure logging with specified level and format."""
    logger = logging.getLogger("rag_api")
    logger.setLevel(getattr(logging, level.upper()))

    # Remove existing handlers
    logger.handlers = []

    # Create console handler
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(getattr(logging, level.upper()))

    if format_type.lower() == "json":
        formatter = JSONFormatter()
    else:
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger


def get_logger(name: str = "rag_api") -> logging.Logger:
    """Get or create logger instance."""
    return logging.getLogger(name)


class LogContextFilter(logging.Filter):
    """Add request context to log records."""

    def __init__(self, request_id: Optional[str] = None, endpoint: Optional[str] = None):
        """Initialize filter with context."""
        super().__init__()
        self.request_id = request_id
        self.endpoint = endpoint

    def filter(self, record: logging.LogRecord) -> bool:
        """Add context to record."""
        if self.request_id:
            record.request_id = self.request_id
        if self.endpoint:
            record.endpoint = self.endpoint
        return True
