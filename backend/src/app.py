"""FastAPI application - Main entry point for RAG Chatbot Backend."""

import os
import sys
from pathlib import Path

# Add backend directory to Python path for imports
backend_dir = str(Path(__file__).parent.parent)
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

# Load environment variables FIRST before any other imports
from dotenv import load_dotenv
env_path = Path(__file__).parent.parent / '.env'
load_dotenv(env_path)

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import logging

from src.api import chat, auth, chapter_personalization, translation

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
    description="Retrieval-Augmented Generation chatbot with personalized learning",
    version="2.0.0",
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
app.include_router(auth.router, prefix="/auth", tags=["authentication"])
app.include_router(chat.router, prefix="/chat", tags=["chat"])
app.include_router(chapter_personalization.router, prefix="/personalization", tags=["chapter_personalization"])
app.include_router(translation.router, prefix="/translation", tags=["translation"])

@app.get('/')
async def root():
    """Root endpoint - API information."""
    return {
        'service': 'Physical AI & Humanoid Robotics RAG Chatbot',
        'version': '2.0.0',
        'docs': '/api/docs',
        'features': ['RAG Chatbot', 'User Authentication', 'Personalized Learning']
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
    """Startup event - Initialize services."""
    try:
        from src.services.rag_service import get_rag_service
        from src.services.auth_service import get_auth_service
        from src.services.personalization_service import get_personalization_service

        rag_service = get_rag_service()
        auth_service = get_auth_service()
        personalization_service = get_personalization_service()

        logger.info("All services initialized successfully")
    except Exception as e:
        logger.error(f"Error during startup: {e}")
        raise

if __name__ == '__main__':
    import uvicorn
    uvicorn.run(
        app,
        host=os.getenv('FASTAPI_HOST', '0.0.0.0'),
        port=int(os.getenv('FASTAPI_PORT', 8000)),
        log_level=os.getenv('LOG_LEVEL', 'info').lower()
    )
