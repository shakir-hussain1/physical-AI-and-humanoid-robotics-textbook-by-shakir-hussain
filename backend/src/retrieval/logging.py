"""Structured logging for retrieval service.

Provides JSON and human-readable logging for all retrieval operations.
"""

import logging
import json
import sys
from datetime import datetime
from typing import Optional, Any, Dict


class JSONFormatter(logging.Formatter):
    """Custom JSON formatter for structured logging."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON.

        Args:
            record: LogRecord to format

        Returns:
            JSON-formatted log string
        """
        log_obj = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }

        # Add extra fields if present
        if hasattr(record, "operation"):
            log_obj["operation"] = record.operation
        if hasattr(record, "query"):
            log_obj["query"] = record.query
        if hasattr(record, "result_count"):
            log_obj["result_count"] = record.result_count
        if hasattr(record, "latency_ms"):
            log_obj["latency_ms"] = record.latency_ms
        if hasattr(record, "status"):
            log_obj["status"] = record.status

        # Include exception info if present
        if record.exc_info:
            log_obj["exception"] = self.formatException(record.exc_info)

        return json.dumps(log_obj)


def setup_retrieval_logging(
    log_level: str = "INFO",
    log_format: str = "json",
    log_file: Optional[str] = None,
) -> logging.Logger:
    """Setup logging for retrieval service.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
        log_format: Log format (json or text)
        log_file: Optional file path for log output

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger("retrieval")
    logger.setLevel(getattr(logging, log_level.upper()))

    # Remove existing handlers
    logger.handlers = []

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(getattr(logging, log_level.upper()))

    if log_format.lower() == "json":
        formatter = JSONFormatter()
    else:
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )

    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # File handler (if specified)
    if log_file:
        try:
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(getattr(logging, log_level.upper()))
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)
        except Exception as e:
            logger.warning(f"Failed to setup file logging: {str(e)}")

    return logger


def log_retrieval_operation(
    logger: logging.Logger,
    operation: str,
    query: str,
    result_count: int,
    latency_ms: float,
    status: str,
    additional_info: Optional[Dict[str, Any]] = None,
) -> None:
    """Log a retrieval operation with structured fields.

    Args:
        logger: Logger instance
        operation: Operation type (e.g., "search", "validate", "batch")
        query: Query text (truncated if long)
        result_count: Number of results returned
        latency_ms: Operation latency in milliseconds
        status: Operation status (success, error, etc.)
        additional_info: Additional fields to log
    """
    # Truncate long queries for logging
    query_display = query[:100] if query else ""

    extra = {
        "operation": operation,
        "query": query_display,
        "result_count": result_count,
        "latency_ms": round(latency_ms, 2),
        "status": status,
    }

    if additional_info:
        extra.update(additional_info)

    log_level = logging.INFO if status == "success" else logging.WARNING
    logger.log(log_level, f"{operation} operation completed", extra=extra)


def log_validation_result(
    logger: logging.Logger,
    query: str,
    validation_report: Dict[str, Any],
) -> None:
    """Log validation results.

    Args:
        logger: Logger instance
        query: Original query
        validation_report: Validation report dictionary
    """
    query_display = query[:100] if query else ""

    is_valid = validation_report.get("is_valid", False)
    issues = validation_report.get("issues", [])

    log_level = logging.INFO if is_valid else logging.WARNING
    message = f"Validation for query '{query_display}': {'PASS' if is_valid else 'FAIL'}"

    extra = {
        "operation": "validate",
        "query": query_display,
        "is_valid": is_valid,
        "issue_count": len(issues),
        "status": "success",
    }

    logger.log(log_level, message, extra=extra)

    if issues:
        for issue in issues:
            logger.warning(f"Validation issue: {issue}")


# Module-level logger instance
_logger: Optional[logging.Logger] = None


def get_retrieval_logger() -> logging.Logger:
    """Get or create the retrieval logger.

    Returns:
        Configured logger instance
    """
    global _logger
    if _logger is None:
        _logger = setup_retrieval_logging(
            log_level="INFO",
            log_format="json",
            log_file="backend/logs/retrieval.log",
        )
    return _logger
