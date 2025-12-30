"""Health check endpoint."""

import time
from functools import lru_cache

from fastapi import APIRouter, Request

from ..models import HealthResponse
from ...utils.logging import get_logger

router = APIRouter(prefix="/api", tags=["health"])
logger = get_logger()

# Cache health check results for 30 seconds
_last_health_check = {"timestamp": 0, "result": None}
_cache_duration = 30


@router.get("/health", response_model=HealthResponse, status_code=200)
async def health_check(request: Request) -> HealthResponse:
    """
    Check service health and component status.

    GET /api/health
    - Always returns 200 OK (regardless of component status)
    - Reports: agent, retrieval, llm component status
    - Cached: Results cached for 30 seconds
    """
    request_id = getattr(request.state, "request_id", "unknown")
    current_time = time.time()

    # Check cache
    if (
        _last_health_check["result"] is not None
        and (current_time - _last_health_check["timestamp"]) < _cache_duration
    ):
        logger.debug(
            "Health check result from cache",
            extra={"request_id": request_id},
        )
        return _last_health_check["result"]

    # Perform health check
    components = {}

    # Check agent
    try:
        from ..main import agent

        if agent:
            components["agent"] = "ready"
        else:
            components["agent"] = "initializing"
    except Exception as e:
        logger.warning(
            f"Agent health check failed: {str(e)}",
            extra={"request_id": request_id},
        )
        components["agent"] = "error"

    # Check retrieval service
    try:
        from ..main import retrieval_service

        if retrieval_service:
            components["retrieval"] = "ready"
        else:
            components["retrieval"] = "initializing"
    except Exception as e:
        logger.warning(
            f"Retrieval service health check failed: {str(e)}",
            extra={"request_id": request_id},
        )
        components["retrieval"] = "error"

    # Check LLM (simple check - just verify API key exists)
    try:
        from ...config import get_config

        config = get_config()
        if config.openai_api_key:
            components["llm"] = "ready"
        else:
            components["llm"] = "error"
    except Exception as e:
        logger.warning(
            f"LLM health check failed: {str(e)}",
            extra={"request_id": request_id},
        )
        components["llm"] = "error"

    # Determine overall status
    if all(status == "ready" for status in components.values()):
        overall_status = "healthy"
    elif any(status == "error" for status in components.values()):
        overall_status = "degraded"
    else:
        overall_status = "healthy"

    response = HealthResponse(
        status=overall_status,
        components=components,
    )

    # Cache result
    _last_health_check["timestamp"] = current_time
    _last_health_check["result"] = response

    logger.info(
        "Health check completed",
        extra={
            "request_id": request_id,
            "overall_status": overall_status,
            "components": components,
        },
    )

    return response
