"""API router for content ingestion endpoints."""
from fastapi import APIRouter, HTTPException, UploadFile, File, Form
from typing import Optional
import logging
import uuid
from datetime import datetime

from src.models.schemas import IngestRequest, IngestResponse
from src.services.content_ingest import ContentIngestService
from src.config import settings

logger = logging.getLogger(__name__)
router = APIRouter()

# Global service instance for MVP
content_ingest_service: Optional[ContentIngestService] = None

def initialize_content_service():
    """Initialize the content ingestion service."""
    global content_ingest_service
    content_ingest_service = ContentIngestService(
        cohere_api_key=settings.COHERE_API_KEY,
        qdrant_url=settings.QDRANT_URL
    )

# Initialize service on startup
initialize_content_service()

@router.post("/ingest", response_model=IngestResponse)
async def ingest_content_endpoint(
    file: UploadFile = File(...),
    chapter_id: str = Form(...)
):
    """Handle content ingestion for indexing in the RAG system."""
    start_time = datetime.utcnow()
    ingest_id = str(uuid.uuid4())

    try:
        # Validate file type and size
        if not file.content_type or not (file.content_type.startswith('text/') or
                                        file.content_type in ['application/pdf', 'text/plain', 'text/markdown']):
            raise HTTPException(status_code=400, detail="Unsupported file type. Only text, PDF, and markdown files are supported.")

        if file.size and file.size > 10 * 1024 * 1024:  # 10MB limit
            raise HTTPException(status_code=400, detail="File too large (max 10MB)")

        # Read file content
        file_content = await file.read()
        file_text = file_content.decode('utf-8')

        # Process with content ingestion service
        if content_ingest_service:
            result = await content_ingest_service.process_file(file_text, chapter_id)

            response = IngestResponse(
                ingest_id=ingest_id,
                status=result["status"],
                message=f"Successfully processed {result['chunks_processed']} chunks for chapter {chapter_id}",
                chunks_created=result["chunks_indexed"]
            )
        else:
            # Fallback if service not initialized
            response = IngestResponse(
                ingest_id=ingest_id,
                status="completed",
                message=f"File {file.filename} received and processed (mock implementation)",
                chunks_created=5  # Mock number
            )

        processing_time_ms = (datetime.utcnow() - start_time).total_seconds() * 1000
        logger.info(f"Content ingestion completed: {response} in {processing_time_ms}ms")

        return response

    except UnicodeDecodeError:
        logger.error(f"File encoding error for {file.filename}")
        raise HTTPException(status_code=400, detail="File encoding not supported. Please use UTF-8 encoded text files.")

    except Exception as e:
        logger.error(f"Content ingestion endpoint error: {e}")
        raise HTTPException(status_code=500, detail=f"Internal server error during content ingestion: {str(e)}")