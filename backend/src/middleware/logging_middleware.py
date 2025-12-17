"""Structured logging middleware."""

from fastapi import Request
from starlette.middleware.base import BaseHTTPMiddleware
import logging
import time
import json

logger = logging.getLogger(__name__)


class LoggingMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        start_time = time.time()

        try:
            response = await call_next(request)
        except Exception as exc:
            process_time = time.time() - start_time
            log_data = {
                "method": request.method,
                "path": request.url.path,
                "status_code": 500,
                "latency_ms": int(process_time * 1000),
                "error": str(exc),
            }
            logger.error(json.dumps(log_data))
            raise

        process_time = time.time() - start_time
        log_data = {
            "method": request.method,
            "path": request.url.path,
            "status_code": response.status_code,
            "latency_ms": int(process_time * 1000),
        }
        logger.info(json.dumps(log_data))

        return response
