"""Service for managing personalization preferences."""

from typing import Optional, Dict, Any, List
from datetime import datetime
from sqlalchemy.orm import Session
import json

from .auth_models import UserProfile, ContentPreference
from .auth_service import AuthService

# Try to get logger
try:
    from ..utils.logging import get_logger
    logger = get_logger(__name__)
except ImportError:
    import logging
    logger = logging.getLogger(__name__)


class PersonalizationService:
    """Service for managing user content personalization preferences."""

    def __init__(self, db: Session):
        """Initialize personalization service with database session."""
        self.db = db

    def set_global_preference(
        self,
        user_id: str,
        content_level: str = "beginner",
        show_math: bool = True,
        show_code: bool = True,
        show_diagrams: bool = True,
        show_advanced_topics: bool = False,
        focus_areas: Optional[List[str]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Set global content preference (applies to all chapters)."""
        try:
            profile = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            if not profile:
                return None

            # Check if global preference exists
            global_pref = (
                self.db.query(ContentPreference)
                .filter(
                    ContentPreference.user_profile_id == profile.id,
                    ContentPreference.module_id.is_(None),
                    ContentPreference.chapter_id.is_(None),
                )
                .first()
            )

            if global_pref:
                # Update existing
                global_pref.content_level = content_level
                global_pref.show_math = show_math
                global_pref.show_code = show_code
                global_pref.show_diagrams = show_diagrams
                global_pref.show_advanced_topics = show_advanced_topics
                global_pref.focus_areas = json.dumps(focus_areas) if focus_areas else None
                global_pref.updated_at = datetime.utcnow()
            else:
                # Create new
                global_pref = ContentPreference(
                    user_profile_id=profile.id,
                    content_level=content_level,
                    show_math=show_math,
                    show_code=show_code,
                    show_diagrams=show_diagrams,
                    show_advanced_topics=show_advanced_topics,
                    focus_areas=json.dumps(focus_areas) if focus_areas else None,
                )
                self.db.add(global_pref)

            self.db.commit()
            logger.info(f"Global preference set for user: {user_id}")
            return self._preference_to_dict(global_pref)

        except Exception as e:
            self.db.rollback()
            logger.error(f"Error setting global preference: {str(e)}")
            return None

    def set_module_preference(
        self,
        user_id: str,
        module_id: str,
        content_level: str = "beginner",
        show_math: bool = True,
        show_code: bool = True,
        show_diagrams: bool = True,
        show_advanced_topics: bool = False,
        focus_areas: Optional[List[str]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Set module-level content preference."""
        try:
            profile = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            if not profile:
                return None

            # Check if module preference exists
            module_pref = (
                self.db.query(ContentPreference)
                .filter(
                    ContentPreference.user_profile_id == profile.id,
                    ContentPreference.module_id == module_id,
                    ContentPreference.chapter_id.is_(None),
                )
                .first()
            )

            if module_pref:
                # Update existing
                module_pref.content_level = content_level
                module_pref.show_math = show_math
                module_pref.show_code = show_code
                module_pref.show_diagrams = show_diagrams
                module_pref.show_advanced_topics = show_advanced_topics
                module_pref.focus_areas = json.dumps(focus_areas) if focus_areas else None
                module_pref.updated_at = datetime.utcnow()
            else:
                # Create new
                module_pref = ContentPreference(
                    user_profile_id=profile.id,
                    module_id=module_id,
                    content_level=content_level,
                    show_math=show_math,
                    show_code=show_code,
                    show_diagrams=show_diagrams,
                    show_advanced_topics=show_advanced_topics,
                    focus_areas=json.dumps(focus_areas) if focus_areas else None,
                )
                self.db.add(module_pref)

            self.db.commit()
            logger.info(f"Module preference set for user: {user_id}, module: {module_id}")
            return self._preference_to_dict(module_pref)

        except Exception as e:
            self.db.rollback()
            logger.error(f"Error setting module preference: {str(e)}")
            return None

    def set_chapter_preference(
        self,
        user_id: str,
        chapter_id: str,
        module_id: Optional[str] = None,
        content_level: str = "beginner",
        show_math: bool = True,
        show_code: bool = True,
        show_diagrams: bool = True,
        show_advanced_topics: bool = False,
        focus_areas: Optional[List[str]] = None,
    ) -> Optional[Dict[str, Any]]:
        """Set chapter-level content preference."""
        try:
            profile = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            if not profile:
                return None

            # Check if chapter preference exists
            chapter_pref = (
                self.db.query(ContentPreference)
                .filter(
                    ContentPreference.user_profile_id == profile.id,
                    ContentPreference.chapter_id == chapter_id,
                )
                .first()
            )

            if chapter_pref:
                # Update existing
                chapter_pref.module_id = module_id
                chapter_pref.content_level = content_level
                chapter_pref.show_math = show_math
                chapter_pref.show_code = show_code
                chapter_pref.show_diagrams = show_diagrams
                chapter_pref.show_advanced_topics = show_advanced_topics
                chapter_pref.focus_areas = json.dumps(focus_areas) if focus_areas else None
                chapter_pref.updated_at = datetime.utcnow()
            else:
                # Create new
                chapter_pref = ContentPreference(
                    user_profile_id=profile.id,
                    module_id=module_id,
                    chapter_id=chapter_id,
                    content_level=content_level,
                    show_math=show_math,
                    show_code=show_code,
                    show_diagrams=show_diagrams,
                    show_advanced_topics=show_advanced_topics,
                    focus_areas=json.dumps(focus_areas) if focus_areas else None,
                )
                self.db.add(chapter_pref)

            self.db.commit()
            logger.info(f"Chapter preference set for user: {user_id}, chapter: {chapter_id}")
            return self._preference_to_dict(chapter_pref)

        except Exception as e:
            self.db.rollback()
            logger.error(f"Error setting chapter preference: {str(e)}")
            return None

    def get_effective_preference(
        self,
        user_id: str,
        chapter_id: Optional[str] = None,
        module_id: Optional[str] = None,
    ) -> Optional[Dict[str, Any]]:
        """Get effective preference using hierarchy: chapter > module > global."""
        try:
            profile = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            if not profile:
                return None

            # Priority 1: Chapter-specific preference
            if chapter_id:
                chapter_pref = (
                    self.db.query(ContentPreference)
                    .filter(
                        ContentPreference.user_profile_id == profile.id,
                        ContentPreference.chapter_id == chapter_id,
                    )
                    .first()
                )
                if chapter_pref:
                    return self._preference_to_dict(chapter_pref)

            # Priority 2: Module-specific preference
            if module_id:
                module_pref = (
                    self.db.query(ContentPreference)
                    .filter(
                        ContentPreference.user_profile_id == profile.id,
                        ContentPreference.module_id == module_id,
                        ContentPreference.chapter_id.is_(None),
                    )
                    .first()
                )
                if module_pref:
                    return self._preference_to_dict(module_pref)

            # Priority 3: Global preference
            global_pref = (
                self.db.query(ContentPreference)
                .filter(
                    ContentPreference.user_profile_id == profile.id,
                    ContentPreference.module_id.is_(None),
                    ContentPreference.chapter_id.is_(None),
                )
                .first()
            )
            if global_pref:
                return self._preference_to_dict(global_pref)

            # Default if no preferences set
            return {
                "content_level": "beginner",
                "show_math": True,
                "show_code": True,
                "show_diagrams": True,
                "show_advanced_topics": False,
                "focus_areas": [],
            }

        except Exception as e:
            logger.error(f"Error getting effective preference: {str(e)}")
            return None

    def get_all_preferences(self, user_id: str) -> Optional[Dict[str, Any]]:
        """Get all preferences for a user organized by level."""
        try:
            profile = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            if not profile:
                return None

            preferences = self.db.query(ContentPreference).filter(
                ContentPreference.user_profile_id == profile.id
            ).all()

            global_pref = None
            module_prefs = {}
            chapter_prefs = {}

            for pref in preferences:
                pref_dict = self._preference_to_dict(pref)
                if pref.module_id is None and pref.chapter_id is None:
                    global_pref = pref_dict
                elif pref.chapter_id is None:
                    module_prefs[pref.module_id] = pref_dict
                else:
                    chapter_prefs[pref.chapter_id] = pref_dict

            return {
                "global": global_pref,
                "modules": module_prefs,
                "chapters": chapter_prefs,
            }

        except Exception as e:
            logger.error(f"Error getting all preferences: {str(e)}")
            return None

    def delete_preference(self, user_id: str, chapter_id: Optional[str] = None, module_id: Optional[str] = None) -> bool:
        """Delete a specific preference."""
        try:
            profile = self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
            if not profile:
                return False

            query = self.db.query(ContentPreference).filter(ContentPreference.user_profile_id == profile.id)

            if chapter_id:
                query = query.filter(ContentPreference.chapter_id == chapter_id)
            elif module_id:
                query = query.filter(
                    ContentPreference.module_id == module_id,
                    ContentPreference.chapter_id.is_(None),
                )
            else:
                query = query.filter(
                    ContentPreference.module_id.is_(None),
                    ContentPreference.chapter_id.is_(None),
                )

            pref = query.first()
            if pref:
                self.db.delete(pref)
                self.db.commit()
                logger.info(f"Preference deleted for user: {user_id}")
                return True

            return False

        except Exception as e:
            self.db.rollback()
            logger.error(f"Error deleting preference: {str(e)}")
            return False

    @staticmethod
    def _preference_to_dict(preference: ContentPreference) -> Dict[str, Any]:
        """Convert ContentPreference model to dictionary."""
        return {
            "id": preference.id,
            "module_id": preference.module_id,
            "chapter_id": preference.chapter_id,
            "content_level": preference.content_level,
            "show_math": preference.show_math,
            "show_code": preference.show_code,
            "show_diagrams": preference.show_diagrams,
            "show_advanced_topics": preference.show_advanced_topics,
            "focus_areas": json.loads(preference.focus_areas) if preference.focus_areas else [],
            "created_at": preference.created_at.isoformat(),
            "updated_at": preference.updated_at.isoformat(),
        }
