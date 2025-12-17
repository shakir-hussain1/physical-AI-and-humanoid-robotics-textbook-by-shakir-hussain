"""FastAPI application - Main entry point for RAG Chatbot Backend."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging
import sys
import os

from src.api import chat

# Configure logging
logging.basicConfig(
    level=os.getenv('LOG_LEVEL', 'INFO'),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    stream=sys.stdout
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    openapi_url="/api/openapi.json"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, prefix="/chat", tags=["chat"])

@app.get('/')
async def root():
    """Root endpoint - API information."""
    return {
        'service': 'Physical AI & Humanoid Robotics RAG Chatbot',
        'version': '1.0.0',
        'docs': '/api/docs',
        'health': '/chat/health'
    }

@app.get('/health')
async def health_check():
    """Global health check endpoint."""
    return {
        'status': 'healthy',
        'service': 'RAG Chatbot Backend'
    }

@app.on_event('startup')
async def startup_event():
    """Startup event - Initialize RAG service."""
    try:
        from src.services.rag_service import get_rag_service
        rag_service = get_rag_service()
        logger.info("RAG service initialized successfully")
    except Exception as e:
        logger.error(f"Error initializing RAG service: {e}")
        raise

if __name__ == '__main__':
    import uvicorn
    uvicorn.run(
        app,
        host=os.getenv('FASTAPI_HOST', '0.0.0.0'),
        port=int(os.getenv('FASTAPI_PORT', 8000)),
        log_level=os.getenv('LOG_LEVEL', 'info').lower()
    )
