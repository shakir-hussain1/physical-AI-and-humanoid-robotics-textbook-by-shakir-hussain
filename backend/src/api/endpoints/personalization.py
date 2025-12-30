"""Personalization endpoints for content preferences."""

from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session

from ..personalization_service import PersonalizationService
from ..personalization_schemas import (
    SetPreferenceRequest,
    SetChapterPreferenceRequest,
    SetModulePreferenceRequest,
    PreferenceResponse,
    AllPreferencesResponse,
    ContentFilterRequest,
    EffectivePreferenceResponse,
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

router = APIRouter(prefix="/api/personalization", tags=["personalization"])


@router.post("/global-preference", response_model=PreferenceResponse)
async def set_global_preference(
    request: Request,
    preference_data: SetPreferenceRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Set global content preference (applies to all chapters)."""
    try:
        service = PersonalizationService(db)
        result = service.set_global_preference(
            user_id=current_user.id,
            content_level=preference_data.content_level,
            show_math=preference_data.show_math,
            show_code=preference_data.show_code,
            show_diagrams=preference_data.show_diagrams,
            show_advanced_topics=preference_data.show_advanced_topics,
            focus_areas=preference_data.focus_areas,
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to set preference",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(f"Global preference set for user: {current_user.id}", extra={"request_id": request_id})

        return PreferenceResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error setting global preference: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.post("/module-preference", response_model=PreferenceResponse)
async def set_module_preference(
    request: Request,
    preference_data: SetModulePreferenceRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Set module-level content preference."""
    try:
        service = PersonalizationService(db)
        result = service.set_module_preference(
            user_id=current_user.id,
            module_id=preference_data.module_id,
            content_level=preference_data.content_level,
            show_math=preference_data.show_math,
            show_code=preference_data.show_code,
            show_diagrams=preference_data.show_diagrams,
            show_advanced_topics=preference_data.show_advanced_topics,
            focus_areas=preference_data.focus_areas,
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to set preference",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Module preference set for user: {current_user.id}, module: {preference_data.module_id}",
            extra={"request_id": request_id},
        )

        return PreferenceResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error setting module preference: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.post("/chapter-preference", response_model=PreferenceResponse)
async def set_chapter_preference(
    request: Request,
    preference_data: SetChapterPreferenceRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Set chapter-level content preference."""
    try:
        service = PersonalizationService(db)
        result = service.set_chapter_preference(
            user_id=current_user.id,
            chapter_id=preference_data.chapter_id,
            module_id=preference_data.module_id,
            content_level=preference_data.content_level,
            show_math=preference_data.show_math,
            show_code=preference_data.show_code,
            show_diagrams=preference_data.show_diagrams,
            show_advanced_topics=preference_data.show_advanced_topics,
            focus_areas=preference_data.focus_areas,
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to set preference",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Chapter preference set for user: {current_user.id}, chapter: {preference_data.chapter_id}",
            extra={"request_id": request_id},
        )

        return PreferenceResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error setting chapter preference: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.get("/effective-preference", response_model=EffectivePreferenceResponse)
async def get_effective_preference(
    request: Request,
    chapter_id: str,
    module_id: str = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Get effective preference using hierarchy: chapter > module > global."""
    try:
        service = PersonalizationService(db)
        result = service.get_effective_preference(
            user_id=current_user.id,
            chapter_id=chapter_id,
            module_id=module_id,
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Preferences not found",
            )

        # Determine source
        applied_from = "global"
        if chapter_id and any(
            p.get("chapter_id") == chapter_id
            for p in [result] if "chapter_id" in p
        ):
            applied_from = "chapter"
        elif module_id and result.get("module_id") == module_id:
            applied_from = "module"

        result["applied_from"] = applied_from

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Effective preference retrieved for user: {current_user.id}",
            extra={"request_id": request_id},
        )

        return EffectivePreferenceResponse(**result)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error getting effective preference: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.get("/all", response_model=AllPreferencesResponse)
async def get_all_preferences(
    request: Request,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Get all preferences for current user."""
    try:
        service = PersonalizationService(db)
        all_prefs = service.get_all_preferences(current_user.id)

        if all_prefs is None:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found",
            )

        # Convert to response format
        global_pref = None
        if all_prefs.get("global"):
            global_pref = PreferenceResponse(**all_prefs["global"])

        module_prefs = {
            module_id: PreferenceResponse(**pref_dict)
            for module_id, pref_dict in all_prefs.get("modules", {}).items()
        }

        chapter_prefs = {
            chapter_id: PreferenceResponse(**pref_dict)
            for chapter_id, pref_dict in all_prefs.get("chapters", {}).items()
        }

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"All preferences retrieved for user: {current_user.id}",
            extra={"request_id": request_id},
        )

        return AllPreferencesResponse(
            global_preference=global_pref,
            module_preferences=module_prefs,
            chapter_preferences=chapter_prefs,
        )

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error getting all preferences: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )


@router.delete("/preference/{preference_level}")
async def delete_preference(
    request: Request,
    preference_level: str,
    chapter_id: str = None,
    module_id: str = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Delete a specific preference."""
    try:
        if preference_level not in ["global", "module", "chapter"]:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid preference level",
            )

        service = PersonalizationService(db)

        if preference_level == "chapter" and not chapter_id:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="chapter_id required for chapter preference",
            )

        if preference_level == "module" and not module_id:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="module_id required for module preference",
            )

        deleted = service.delete_preference(
            user_id=current_user.id,
            chapter_id=chapter_id if preference_level == "chapter" else None,
            module_id=module_id if preference_level == "module" else None,
        )

        if not deleted:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Preference not found",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(
            f"Preference deleted for user: {current_user.id}, level: {preference_level}",
            extra={"request_id": request_id},
        )

        return {"status": "success", "message": "Preference deleted"}

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error deleting preference: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error",
        )
