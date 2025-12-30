"""FastAPI application for RAG chatbot backend."""

import time
import uuid
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from ..config import get_config
from ..utils.logging import configure_logging, get_logger

# Initialize configuration
try:
    config = get_config()
except ValueError as e:
    print(f"Configuration error: {e}")
    raise

# Configure logging
logger = configure_logging(level=config.log_level, format_type=config.log_format)

# Global dependencies (will be initialized in lifespan)
agent = None
retrieval_service = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application startup and shutdown."""
    logger.info("Starting RAG Chatbot API")

    # Startup - Initialize database
    try:
        from .database import init_db
        init_db()
        logger.info("Database initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize database: {str(e)}")
        raise

    # Startup - Initialize RAG agent
    try:
        # Import and initialize RAG agent
        global agent
        from ..agent import AgentOrchestrator, AgentConfig

        agent_config = AgentConfig.load_from_env()
        agent = AgentOrchestrator(agent_config)
        logger.info("RAG agent initialized successfully")

    except Exception as e:
        logger.error(f"Failed to initialize agent: {str(e)}")
        raise

    # Initialize retrieval service (optional - API works without it)
    try:
        global retrieval_service
        from ..retrieval.retrieval_service import RetrievalService
        from ..retrieval.config import RetrievalConfig

        # Load retrieval config from environment
        retrieval_config = RetrievalConfig()
        retrieval_service = RetrievalService(retrieval_config)
        logger.info("Retrieval service initialized successfully")
    except Exception as e:
        logger.warning(f"Failed to initialize retrieval service (continuing without it): {str(e)}")
        retrieval_service = None

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API")


# Create FastAPI application
app = FastAPI(
    title=config.api_title,
    description=config.api_description,
    version=config.api_version,
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=config.cors_origins,
    allow_credentials=config.cors_allow_credentials,
    allow_methods=config.cors_allow_methods,
    allow_headers=config.cors_allow_headers,
    max_age=86400,  # 1 day
)


# Middleware for request ID tracking and logging
@app.middleware("http")
async def add_request_id_middleware(request: Request, call_next):
    """Add request ID to all requests and log timing."""
    request_id = str(uuid.uuid4())
    request.state.request_id = request_id

    # Log request start
    start_time = time.time()
    logger.info(
        f"Request started",
        extra={
            "request_id": request_id,
            "endpoint": request.url.path,
            "method": request.method,
        },
    )

    try:
        response = await call_next(request)
        latency_ms = int((time.time() - start_time) * 1000)

        # Add request ID to response headers
        response.headers["X-Request-ID"] = request_id

        # Log response
        logger.info(
            f"Request completed",
            extra={
                "request_id": request_id,
                "endpoint": request.url.path,
                "method": request.method,
                "status_code": response.status_code,
                "latency_ms": latency_ms,
            },
        )

        return response

    except Exception as e:
        latency_ms = int((time.time() - start_time) * 1000)
        logger.error(
            f"Request failed: {str(e)}",
            extra={
                "request_id": request_id,
                "endpoint": request.url.path,
                "method": request.method,
                "latency_ms": latency_ms,
            },
        )
        raise


# Global exception handler
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Handle unhandled exceptions."""
    request_id = getattr(request.state, "request_id", "unknown")

    logger.error(
        f"Unhandled exception: {str(exc)}",
        extra={"request_id": request_id},
        exc_info=True,
    )

    return JSONResponse(
        status_code=500,
        content={
            "error": "internal_server_error",
            "message": "An unexpected error occurred",
            "request_id": request_id,
        },
    )


# Health check endpoint (temporary, will be replaced with full endpoint)
@app.get("/api/health", tags=["health"])
async def health_check(request: Request):
    """Check service health."""
    request_id = getattr(request.state, "request_id", "unknown")

    return {
        "status": "healthy",
        "timestamp": time.time(),
        "request_id": request_id,
        "components": {
            "agent": "ready" if agent else "initializing",
            "retrieval": "ready" if retrieval_service else "initializing",
            "api": "ready",
        },
    }


# Register endpoint routers
from .endpoints import query, retrieve, health, auth, personalization, translation

app.include_router(auth.router)
app.include_router(personalization.router)
app.include_router(translation.router)
app.include_router(query.router)
app.include_router(retrieve.router)
app.include_router(health.router)


# Root endpoint
@app.get("/", tags=["root"])
async def root(request: Request):
    """API root endpoint."""
    return {
        "name": config.api_title,
        "version": config.api_version,
        "status": "ready",
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "backend.src.api.main:app",
        host=config.api_host,
        port=config.api_port,
        reload=True,
    )
