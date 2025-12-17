"""Chat API endpoints - Main RAG chatbot interface."""

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import logging
from src.services.rag_service import get_rag_service

logger = logging.getLogger(__name__)
router = APIRouter()

class ChatQuery(BaseModel):
    """Chat query request model."""
    query: str
    user_context: Optional[str] = None
    conversation_history: Optional[List[Dict[str, str]]] = None

class ChatResponse(BaseModel):
    """Chat response model."""
    answer: str
    sources: List[Dict[str, Any]]
    confidence: float
    confidence_level: str
    status: str

@router.post('/query', response_model=ChatResponse)
async def chat_query(request: ChatQuery):
    """
    Process a chat query using RAG pipeline.

    - **query**: The user's question
    - **user_context**: Optional text selected by user
    - **conversation_history**: Optional previous messages
    """
    try:
        if not request.query or len(request.query.strip()) < 1:
            raise HTTPException(status_code=400, detail='Query cannot be empty')

        if len(request.query) > 1000:
            raise HTTPException(status_code=400, detail='Query too long (max 1000 characters)')

        rag_service = get_rag_service()
        result = rag_service.generate_answer(
            query=request.query,
            user_context=request.user_context,
            conversation_history=request.conversation_history
        )

        return ChatResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f'Error in chat query: {e}')
        raise HTTPException(status_code=500, detail=str(e))

@router.post('/context')
async def save_context(context_data: Dict[str, Any]):
    """
    Save conversation context for enhanced RAG.

    Stores user's selected text or conversation context for better understanding.
    """
    try:
        if not context_data or 'content' not in context_data:
            raise HTTPException(status_code=400, detail='Context content required')

        # In production, this would save to database
        logger.info(f"Context saved: {context_data}")
        return {
            'status': 'success',
            'message': 'Context saved successfully'
        }

    except Exception as e:
        logger.error(f'Error saving context: {e}')
        raise HTTPException(status_code=500, detail=str(e))

@router.get('/health')
async def health_check():
    """Health check endpoint."""
    return {
        'status': 'healthy',
        'service': 'RAG Chatbot Backend',
        'version': '1.0.0'
    }
