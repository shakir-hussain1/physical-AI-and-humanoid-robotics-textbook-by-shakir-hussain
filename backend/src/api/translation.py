"""
Translation API endpoints - Translate chapter content to different languages

Features:
- HTML-aware translation (preserves structure)
- Language support (extensible)
- Translation caching
- Confidence scoring
"""

from fastapi import APIRouter, HTTPException, Header, Depends
from pydantic import BaseModel, validator
from typing import Optional, Dict, Any, List
import logging
import re
from datetime import datetime

from src.services.auth_service import get_auth_service
from src.services.translation_service import TranslationService

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize translation service
translation_service = TranslationService()

# Models
class TranslationRequest(BaseModel):
    """Translation request model"""
    content: str  # HTML content to translate
    chapter_id: str  # Chapter identifier
    target_language: str = "urdu"  # Target language code

    @validator('content')
    def content_not_empty(cls, v):
        if not v or len(v.strip()) < 10:
            raise ValueError('Content must be at least 10 characters')
        return v

    @validator('chapter_id')
    def chapter_id_valid(cls, v):
        if not v or len(v) < 3:
            raise ValueError('Chapter ID must be at least 3 characters')
        return v

    @validator('target_language')
    def language_valid(cls, v):
        valid_languages = ['urdu', 'spanish', 'french', 'arabic', 'hindi']
        if v not in valid_languages:
            raise ValueError(f'Language must be one of {valid_languages}')
        return v


class TranslationResponse(BaseModel):
    """Translation response model"""
    status: str = "success"
    translated_content: str
    chapter_id: str
    target_language: str
    confidence_score: float
    from_cache: bool = False
    translated_at: str


class LanguageInfo(BaseModel):
    """Language information model"""
    code: str
    name: str
    native_name: str


def get_current_user(authorization: str = Header(None)) -> Dict[str, Any]:
    """Extract user from JWT token in Authorization header"""
    if not authorization or not authorization.startswith('Bearer '):
        logger.warning('Missing or invalid authorization header')
        raise HTTPException(status_code=401, detail='Missing or invalid token')

    try:
        token = authorization.split(' ')[1]
        auth_service = get_auth_service()
        payload = auth_service.verify_token(token)

        if not payload:
            raise HTTPException(status_code=401, detail='Invalid or expired token')

        return {
            'user_id': int(payload.get('sub')),
            'email': payload.get('email'),
            'username': payload.get('username')
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error extracting user from token: {e}")
        raise HTTPException(status_code=401, detail='Invalid or expired token')


@router.post('/translate', response_model=TranslationResponse)
async def translate_content(
    request: TranslationRequest,
    authorization: str = Header(None)
) -> Dict[str, Any]:
    """
    Translate chapter content to target language

    Endpoint: POST /translation/translate

    Request body:
    {
        "content": "<h1>Chapter Title</h1><p>Content...</p>",
        "chapter_id": "modules/module-1-ros2/chapter-01",
        "target_language": "urdu"
    }

    Returns:
    {
        "status": "success",
        "translated_content": "<h1>عنوان</h1><p>مواد...</p>",
        "chapter_id": "modules/module-1-ros2/chapter-01",
        "target_language": "urdu",
        "confidence_score": 0.95,
        "from_cache": false,
        "translated_at": "2025-12-20T10:30:38.529051"
    }
    """
    try:
        # Authenticate user
        user = get_current_user(authorization)
        user_id = user['user_id']

        logger.info(
            f"Translation requested for user {user_id} "
            f"chapter {request.chapter_id} to {request.target_language}"
        )

        # Translate content
        result = await translation_service.translate(
            content=request.content,
            target_language=request.target_language,
            chapter_id=request.chapter_id,
            user_id=user_id
        )

        logger.info(f"Successfully translated content for chapter {request.chapter_id}")

        return {
            'status': 'success',
            'translated_content': result['translated_content'],
            'chapter_id': request.chapter_id,
            'target_language': request.target_language,
            'confidence_score': result.get('confidence_score', 0.9),
            'from_cache': result.get('from_cache', False),
            'translated_at': datetime.utcnow().isoformat()
        }

    except HTTPException:
        raise
    except ValueError as e:
        logger.error(f"Validation error: {e}")
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        import traceback
        logger.error(f"Error translating content: {e}")
        logger.error(f"Traceback: {traceback.format_exc()}")
        raise HTTPException(
            status_code=500,
            detail=f'Failed to translate content: {str(e)}'
        )


@router.get('/languages')
async def get_supported_languages(
    authorization: str = Header(None)
) -> Dict[str, Any]:
    """
    Get list of supported languages for translation

    Returns:
    {
        "status": "success",
        "languages": [
            {
                "code": "urdu",
                "name": "Urdu",
                "native_name": "اردو"
            },
            ...
        ]
    }
    """
    try:
        # Authenticate user (optional, but good practice)
        get_current_user(authorization)

        languages = translation_service.get_supported_languages()

        return {
            'status': 'success',
            'languages': languages
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error fetching languages: {e}")
        raise HTTPException(status_code=500, detail='Failed to fetch languages')


@router.get('/stats/{chapter_id}')
async def get_translation_stats(
    chapter_id: str,
    authorization: str = Header(None)
) -> Dict[str, Any]:
    """
    Get translation statistics for a chapter

    Useful for analytics and debugging

    Returns:
    {
        "status": "success",
        "chapter_id": "modules/module-1-ros2/chapter-01",
        "languages_translated": ["urdu", "spanish"],
        "total_translations": 5,
        "cache_hit_rate": 0.6,
        "last_translated_at": "2025-12-20T10:30:38.529051"
    }
    """
    try:
        # Authenticate user
        user = get_current_user(authorization)
        user_id = user['user_id']

        logger.info(f"Translation stats requested for chapter {chapter_id}")

        stats = await translation_service.get_stats(chapter_id)

        return {
            'status': 'success',
            'chapter_id': chapter_id,
            **stats
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error fetching translation stats: {e}")
        raise HTTPException(status_code=500, detail='Failed to fetch translation stats')


@router.post('/cache/clear/{chapter_id}')
async def clear_translation_cache(
    chapter_id: str,
    authorization: str = Header(None)
) -> Dict[str, Any]:
    """
    Clear cached translations for a chapter

    Useful for refreshing translations

    Returns:
    {
        "status": "success",
        "message": "Translation cache cleared for chapter {chapter_id}"
    }
    """
    try:
        # Authenticate user
        user = get_current_user(authorization)

        logger.info(f"Clearing translation cache for chapter {chapter_id}")

        await translation_service.clear_cache(chapter_id)

        return {
            'status': 'success',
            'message': f'Translation cache cleared for chapter {chapter_id}'
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error clearing cache: {e}")
        raise HTTPException(status_code=500, detail='Failed to clear cache')


@router.get('/health')
async def translation_health_check() -> Dict[str, Any]:
    """
    Health check for translation service

    Returns:
    {
        "status": "healthy",
        "service": "translation",
        "supported_languages": ["urdu", "spanish", "french"],
        "cache_enabled": true
    }
    """
    return {
        'status': 'healthy',
        'service': 'translation',
        'supported_languages': translation_service.get_supported_languages_codes(),
        'cache_enabled': True,
        'timestamp': datetime.utcnow().isoformat()
    }
