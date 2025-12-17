"""Health check endpoint."""

from fastapi import APIRouter
from src.db.postgres import check_db_connection
from src.db.qdrant_client import check_qdrant_connection
from src.models.schemas import HealthResponse
from datetime import datetime
import logging

router = APIRouter()
logger = logging.getLogger(__name__)


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """GET /health - Liveness and readiness check."""
    postgres_ok = check_db_connection()
    qdrant_ok = check_qdrant_connection()

    status = "healthy" if (postgres_ok and qdrant_ok) else "degraded"

    return HealthResponse(
        status=status,
        timestamp=datetime.utcnow().isoformat(),
        services={
            "postgres": postgres_ok,
            "qdrant": qdrant_ok,
            "claude_api": True,  # Assume available; verify on first call
        },
    )
