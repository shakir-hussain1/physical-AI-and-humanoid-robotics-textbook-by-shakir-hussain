"""Chapter Personalization endpoints - Save/retrieve/reset chapter preferences."""

from fastapi import APIRouter, HTTPException, Header
from pydantic import BaseModel, validator
from typing import Optional, Dict, Any, List
import logging

from src.services.auth_service import get_auth_service

logger = logging.getLogger(__name__)
router = APIRouter()

# Models
class PersonalizationSettings(BaseModel):
    """Chapter personalization settings."""
    difficulty_level: str = "intermediate"  # beginner, intermediate, advanced, expert
    content_style: str = "balanced"  # text, visual, code, balanced
    example_density: str = "moderate"  # minimal, moderate, rich
    learning_pace: str = "standard"  # concise, standard, detailed
    custom_preferences: Dict[str, Any] = {}

    @validator('difficulty_level')
    def validate_difficulty(cls, v):
        allowed = ['beginner', 'intermediate', 'advanced', 'expert']
        if v not in allowed:
            raise ValueError(f'difficulty_level must be one of {allowed}')
        return v

    @validator('content_style')
    def validate_style(cls, v):
        allowed = ['text', 'visual', 'code', 'balanced']
        if v not in allowed:
            raise ValueError(f'content_style must be one of {allowed}')
        return v

    @validator('example_density')
    def validate_density(cls, v):
        allowed = ['minimal', 'moderate', 'rich']
        if v not in allowed:
            raise ValueError(f'example_density must be one of {allowed}')
        return v

    @validator('learning_pace')
    def validate_pace(cls, v):
        allowed = ['concise', 'standard', 'detailed']
        if v not in allowed:
            raise ValueError(f'learning_pace must be one of {allowed}')
        return v


class PersonalizationResponse(BaseModel):
    """Chapter personalization response."""
    chapter_id: str
    difficulty_level: str
    content_style: str
    example_density: str
    learning_pace: str
    custom_preferences: Dict[str, Any]
    created_at: str
    updated_at: str


# In-memory storage for chapter personalizations
# In production, this would be in a database
chapter_personalizations_db = {}  # {user_id: {chapter_id: settings}}


def get_current_user(authorization: str = Header(None)) -> Dict[str, Any]:
    """Extract user from JWT token in Authorization header."""
    if not authorization or not authorization.startswith('Bearer '):
        logger.warning('Missing or invalid authorization header')
        raise HTTPException(status_code=401, detail='Missing or invalid token')

    try:
        token = authorization.split(' ')[1]
        logger.debug(f'Extracted token from header')

        auth_service = get_auth_service()
        logger.debug(f'Got auth service')

        payload = auth_service.verify_token(token)
        logger.debug(f'Verified token, payload type: {type(payload)}')

        if not payload:
            logger.warning('Token verification returned empty payload')
            raise HTTPException(status_code=401, detail='Invalid or expired token')

        sub = payload.get('sub')
        logger.debug(f'Extracted sub from payload: {sub} (type: {type(sub)})')

        # Try to convert to int, handle both string and int cases
        try:
            user_id = int(sub) if isinstance(sub, str) else sub
        except (ValueError, TypeError) as e:
            logger.error(f'Failed to convert sub to int: {sub} - {e}')
            raise ValueError(f'Invalid user_id format: {sub}')

        result = {
            'user_id': user_id,
            'email': payload.get('email'),
            'username': payload.get('username')
        }
        logger.info(f'Successfully extracted user: {user_id}')
        return result

    except HTTPException:
        raise
    except (IndexError, ValueError) as e:
        logger.error(f'Token format error: {e}')
        raise HTTPException(status_code=401, detail=f'Invalid token format: {str(e)}')
    except Exception as e:
        import traceback
        logger.error(f'Error extracting user from token: {e}')
        logger.error(f'Traceback: {traceback.format_exc()}')
        raise HTTPException(status_code=401, detail=f'Invalid or expired token: {str(e)}')


@router.post('/chapter/{chapter_id:path}')
async def save_chapter_personalization(
    chapter_id: str,
    settings: PersonalizationSettings,
    authorization: str = Header(None)
) -> Dict[str, Any]:
    """
    Save or update personalization settings for a chapter.

    - **chapter_id**: Unique identifier for the chapter (e.g., "modules/module-1-ros2/chapter-01")
    - **settings**: Personalization settings (difficulty_level, content_style, example_density, learning_pace)

    Returns saved personalization object.
    """
    try:
        logger.debug(f"Saving personalization for chapter: {chapter_id}")
        logger.debug(f"Settings received: {settings}")

        # Get current user from token
        logger.debug('Step 1: Getting current user')
        user = get_current_user(authorization)
        user_id = user['user_id']
        logger.info(f"Authenticated user: {user_id}")

        # Initialize user's personalization dict if not exists
        logger.debug('Step 2: Initializing user personalization dict')
        if user_id not in chapter_personalizations_db:
            chapter_personalizations_db[user_id] = {}

        # Get current time
        logger.debug('Step 3: Getting current timestamp')
        from datetime import datetime
        now = datetime.utcnow().isoformat()

        # Check if personalization already exists to preserve created_at
        logger.debug('Step 4: Checking for existing personalization')
        existing = chapter_personalizations_db[user_id].get(chapter_id)
        created_at = existing.get('created_at', now) if existing else now

        # Save or update personalization
        logger.debug('Step 5: Creating personalization record')
        personalization_data = {
            'chapter_id': chapter_id,
            'difficulty_level': settings.difficulty_level,
            'content_style': settings.content_style,
            'example_density': settings.example_density,
            'learning_pace': settings.learning_pace,
            'custom_preferences': settings.custom_preferences,
            'created_at': created_at,
            'updated_at': now
        }

        logger.debug('Step 6: Storing in database')
        chapter_personalizations_db[user_id][chapter_id] = personalization_data

        logger.info(f"Successfully saved personalization for user {user_id} chapter {chapter_id}")

        return {
            'status': 'success',
            'message': 'Personalization saved successfully',
            'data': personalization_data
        }

    except HTTPException as http_err:
        logger.warning(f"HTTP error in save_personalization: {http_err.status_code} - {http_err.detail}")
        raise
    except Exception as e:
        import traceback
        error_msg = str(e)
        traceback_str = traceback.format_exc()
        logger.error(f"Unexpected error saving personalization: {error_msg}")
        logger.error(f"Traceback:\n{traceback_str}")
        print(f"[ERROR] Save personalization failed: {error_msg}")
        print(f"[ERROR] Traceback:\n{traceback_str}")
        raise HTTPException(
            status_code=500,
            detail=f'Failed to save personalization: {error_msg}'
        )


@router.get('/chapter/{chapter_id:path}')
async def get_chapter_personalization(
    chapter_id: str,
    authorization: str = Header(None)
) -> Dict[str, Any]:
    """
    Retrieve personalization settings for a chapter.

    Returns personalization settings if found, or null if not set.
    """
    try:
        user = get_current_user(authorization)
        user_id = user['user_id']

        # Check if user has any personalizations
        if user_id not in chapter_personalizations_db:
            logger.info(f"No personalizations found for user {user_id}")
            return {'status': 'success', 'data': None}

        # Check if this chapter has personalization
        if chapter_id not in chapter_personalizations_db[user_id]:
            logger.info(f"No personalization found for user {user_id} chapter {chapter_id}")
            return {'status': 'success', 'data': None}

        data = chapter_personalizations_db[user_id][chapter_id]
        logger.info(f"Retrieved personalization for user {user_id} chapter {chapter_id}")

        return {
            'status': 'success',
            'data': data
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error retrieving personalization: {e}")
        raise HTTPException(status_code=500, detail='Failed to retrieve personalization')


@router.get('/chapters')
async def get_all_chapter_personalizations(
    authorization: str = Header(None)
) -> Dict[str, Any]:
    """
    Get all chapter personalizations for the current user.

    Returns array of personalization objects.
    """
    try:
        user = get_current_user(authorization)
        user_id = user['user_id']

        if user_id not in chapter_personalizations_db:
            logger.info(f"No personalizations found for user {user_id}")
            return {'status': 'success', 'data': []}

        personalizations = list(chapter_personalizations_db[user_id].values())
        logger.info(f"Retrieved {len(personalizations)} personalizations for user {user_id}")

        return {
            'status': 'success',
            'data': personalizations
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error retrieving personalizations: {e}")
        raise HTTPException(status_code=500, detail='Failed to retrieve personalizations')


@router.delete('/chapter/{chapter_id:path}')
async def reset_chapter_personalization(
    chapter_id: str,
    authorization: str = Header(None)
) -> Dict[str, Any]:
    """
    Reset chapter personalization to default (remove custom settings).

    Deletes personalization data for this chapter.
    """
    try:
        user = get_current_user(authorization)
        user_id = user['user_id']

        # Check if personalization exists
        if user_id not in chapter_personalizations_db or chapter_id not in chapter_personalizations_db[user_id]:
            logger.info(f"No personalization to delete for user {user_id} chapter {chapter_id}")
            return {
                'status': 'success',
                'message': 'No personalization to reset'
            }

        # Delete personalization
        del chapter_personalizations_db[user_id][chapter_id]
        logger.info(f"Reset personalization for user {user_id} chapter {chapter_id}")

        return {
            'status': 'success',
            'message': 'Personalization reset to default'
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error resetting personalization: {e}")
        raise HTTPException(status_code=500, detail='Failed to reset personalization')
