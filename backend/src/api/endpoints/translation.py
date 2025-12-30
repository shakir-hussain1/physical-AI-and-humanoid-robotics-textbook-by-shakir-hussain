"""Translation endpoints for Urdu language support."""

from fastapi import APIRouter, Depends, HTTPException, status, Request, Query
from sqlalchemy.orm import Session

from ..translation_service import TranslationService, TerminologyService
from ..translation_schemas import (
    TranslateContentRequest,
    BatchTranslateRequest,
    TranslationResponse,
    TranslationStatusResponse,
    LoadTerminologyRequest,
    TerminologyResponse,
    ToggleLanguageRequest,
    RTLResponse,
)
from ..auth_models import User
from .auth import get_current_user, get_db

# Try to get logger
try:
    from ...utils.logging import get_logger
    logger = get_logger(__name__)
except ImportError:
    import logging
    logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/translation", tags=["translation"])


@router.post("/translate", response_model=TranslationResponse)
async def translate_content(
    request: Request,
    translate_data: TranslateContentRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Translate content to Urdu with terminology replacement."""
    try:
        service = TranslationService(db)

        result = service.translate_content(
            user_id=current_user.id,
            content_id=translate_data.content_id,
            content_text=translate_data.content_text,
            content_type=translate_data.content_type,
            target_language=translate_data.target_language,
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to translate content",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Content translated for user: {current_user.id}, content: {translate_data.content_id}",
            extra={"request_id": request_id},
        )

        return TranslationResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error translating content: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.get("/content/{content_id}", response_model=TranslationResponse)
async def get_translation(
    request: Request,
    content_id: str,
    language: str = Query("ur", description="Target language code"),
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Get cached translation for content."""
    try:
        service = TranslationService(db)

        result = service.get_translation(
            content_id=content_id,
            target_language=language,
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Translation not found",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Translation retrieved for content: {content_id}",
            extra={"request_id": request_id},
        )

        return TranslationResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error getting translation: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.get("/status/{content_id}", response_model=TranslationStatusResponse)
async def get_translation_status(
    request: Request,
    content_id: str,
    language: str = Query("ur", description="Target language code"),
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Get translation status."""
    try:
        service = TranslationService(db)

        result = service.get_translation_status(
            content_id=content_id,
            target_language=language,
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Status not found",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Translation status retrieved for content: {content_id}",
            extra={"request_id": request_id},
        )

        return TranslationStatusResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error getting status: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.post("/batch", response_model=list[TranslationResponse])
async def batch_translate(
    request: Request,
    batch_data: BatchTranslateRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Batch translate multiple chapters efficiently."""
    try:
        service = TranslationService(db)

        results = service.batch_translate_chapters(
            user_id=current_user.id,
            chapters=batch_data.chapters,
            target_language=batch_data.target_language,
        )

        if not results:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to translate chapters",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Batch translated {len(results)} chapters for user: {current_user.id}",
            extra={"request_id": request_id},
        )

        return [TranslationResponse(**r) for r in results]

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error in batch translation: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.post("/terminology/load", status_code=status.HTTP_201_CREATED)
async def load_terminology(
    request: Request,
    terminology_data: LoadTerminologyRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Load robotics terminology for translation consistency."""
    try:
        service = TerminologyService(db)

        count = service.load_terminology(terminology_data.terms)

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Loaded {count} terminology entries",
            extra={"request_id": request_id},
        )

        return {
            "status": "success",
            "message": f"Loaded {count} terminology entries",
            "count": count,
        }

    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error loading terminology: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to load terminology",
        )


@router.get("/terminology", response_model=list[TerminologyResponse])
async def get_terminology(
    request: Request,
    category: str = Query(None, description="Filter by category"),
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Get translation terminology."""
    try:
        service = TerminologyService(db)

        terms = service.get_terminology(category=category)

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Retrieved {len(terms)} terminology entries",
            extra={"request_id": request_id},
        )

        return [TerminologyResponse(**t) for t in terms]

    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error getting terminology: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve terminology",
        )


@router.delete("/cache/{content_id}")
async def clear_translation_cache(
    request: Request,
    content_id: str = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Clear translation cache."""
    try:
        service = TranslationService(db)

        count = service.clear_translation_cache(content_id=content_id)

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Cleared {count} cache entries",
            extra={"request_id": request_id},
        )

        return {
            "status": "success",
            "message": f"Cleared {count} cache entries",
            "count": count,
        }

    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error clearing cache: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to clear cache",
        )


@router.get("/rtl-config", response_model=RTLResponse)
async def get_rtl_config(
    request: Request,
    language: str = Query("ur", description="Language code"),
    current_user: User = Depends(get_current_user),
):
    """Get RTL configuration for language."""
    try:
        is_rtl = language == "ur"  # Urdu is RTL

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"RTL config retrieved for language: {language}",
            extra={"request_id": request_id},
        )

        return RTLResponse(
            is_rtl=is_rtl,
            text_align="right" if is_rtl else "left",
            direction="rtl" if is_rtl else "ltr",
            font_family="'Noto Sans Urdu', 'Arabic Typesetting', sans-serif"
            if is_rtl
            else "system-ui, -apple-system, sans-serif",
        )

    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error getting RTL config: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )
