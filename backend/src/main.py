"""FastAPI application entrypoint for RAG Chatbot Backend."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging
import sys

from src.config import settings
from src.middleware.logging_middleware import LoggingMiddleware
from src.middleware.error_handler import setup_exception_handlers
from src.middleware.api_key_auth import api_key_auth_middleware
from src.api import health, query, context, content

# Configure logging
logging.basicConfig(
    level=settings.LOG_LEVEL,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    stream=sys.stdout
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot Backend",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # For MVP; restrict in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add API key authentication middleware
app.middleware("http")(api_key_auth_middleware)

# Add logging middleware
app.add_middleware(LoggingMiddleware)

# Setup exception handlers
setup_exception_handlers(app)

# Include routers
app.include_router(health.router, tags=["system"])
app.include_router(query.router, prefix="/chat", tags=["chat"])
app.include_router(context.router, prefix="/chat", tags=["chat"])
app.include_router(content.router, prefix="/content", tags=["content"])

@app.on_event("startup")
async def startup_event():
    logger.info("RAG Chatbot Backend starting up...")
    logger.info(f"Environment: {settings.FASTAPI_ENV}")
    logger.info(f"Log level: {settings.LOG_LEVEL}")

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("RAG Chatbot Backend shutting down...")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host=settings.FASTAPI_HOST,
        port=settings.FASTAPI_PORT,
        reload=settings.FASTAPI_ENV == "development"
    )
