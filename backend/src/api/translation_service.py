"""Service for managing content translation to Urdu with terminology replacement."""

import json
import re
from typing import Optional, Dict, Any, List, Tuple
from datetime import datetime
from sqlalchemy.orm import Session
from sqlalchemy import and_

from .auth_models import Translation, TranslationTerminology, ContentPreference

# Try to import Google Translate - optional
try:
    from google.cloud import translate_v2
    GOOGLE_TRANSLATE_AVAILABLE = True
except ImportError:
    GOOGLE_TRANSLATE_AVAILABLE = False

# Try to get logger
try:
    from ..utils.logging import get_logger
    logger = get_logger(__name__)
except ImportError:
    import logging
    logger = logging.getLogger(__name__)


class TerminologyService:
    """Service for managing translation terminology."""

    def __init__(self, db: Session):
        """Initialize terminology service."""
        self.db = db

    def load_terminology(self, terms: List[Dict[str, str]]) -> int:
        """Load terminology from list of dicts."""
        try:
            count = 0
            for term in terms:
                # Check if term exists
                existing = self.db.query(TranslationTerminology).filter(
                    TranslationTerminology.english_term == term["en"]
                ).first()

                if not existing:
                    terminology = TranslationTerminology(
                        english_term=term["en"],
                        urdu_translation=term["ur"],
                        category=term.get("category", "general"),
                        is_approved=term.get("approved", True),
                        priority=term.get("priority", 0),
                    )
                    self.db.add(terminology)
                    count += 1

            self.db.commit()
            logger.info(f"Loaded {count} new terminology entries")
            return count

        except Exception as e:
            self.db.rollback()
            logger.error(f"Error loading terminology: {str(e)}")
            return 0

    def get_terminology(self, category: Optional[str] = None) -> List[Dict[str, Any]]:
        """Get terminology by category."""
        try:
            query = self.db.query(TranslationTerminology).filter(
                TranslationTerminology.is_approved == True
            )

            if category:
                query = query.filter(TranslationTerminology.category == category)

            terms = query.order_by(TranslationTerminology.priority.desc()).all()

            return [
                {
                    "id": t.id,
                    "english": t.english_term,
                    "urdu": t.urdu_translation,
                    "category": t.category,
                    "priority": t.priority,
                }
                for t in terms
            ]

        except Exception as e:
            logger.error(f"Error getting terminology: {str(e)}")
            return []

    def replace_terminology(self, text: str, terminology: List[Dict[str, Any]]) -> Tuple[str, List[str]]:
        """Replace English terms with Urdu terminology in text."""
        used_terms = []

        try:
            for term in terminology:
                english = term["english"]
                urdu = term["urdu"]
                term_id = term.get("id", "")

                # Case-insensitive replacement using word boundaries
                pattern = r"\b" + re.escape(english) + r"\b"
                if re.search(pattern, text, re.IGNORECASE):
                    text = re.sub(pattern, urdu, text, flags=re.IGNORECASE)
                    used_terms.append(term_id)

            return text, used_terms

        except Exception as e:
            logger.error(f"Error replacing terminology: {str(e)}")
            return text, used_terms


class SimpleTranslator:
    """Simple translator without external API dependencies."""

    # Urdu translation dictionary for key robotics and AI terms
    URDU_DICTIONARY = {
        # Robotics terms
        "robot": "روبوٹ",
        "robotics": "روبوٹکس",
        "humanoid": "انسان نما",
        "robot operating system": "روبوٹ آپریٹنگ سسٹم",
        "ros2": "آر او ایس 2",
        "digital twin": "ڈیجیٹل جڑواں",
        "simulation": "نقل / تخیلات",
        "perception": "ادراک",
        "sensor": "سینسر",
        "vision": "بینائی",
        "control": "کنٹرول",
        "algorithm": "طریقہ کار",
        "machine learning": "مشین لرننگ",
        "artificial intelligence": "مصنوعی ذہانت",
        "locomotion": "رفتار",
        "manipulation": "ہیرا پھیری",
        "navigation": "رہنمائی",
        "planning": "منصوبہ بندی",
        "learning": "سیکھنا",
        "training": "ٹریننگ",

        # AI/ML terms
        "neural network": "اعصابی نیٹ ورک",
        "deep learning": "گہری سیکھ",
        "reinforcement learning": "تقویتی سیکھ",
        "model": "ماڈل",
        "data": "ڈیٹا",
        "network": "نیٹ ورک",
        "algorithm": "الگورتھم",

        # General terms
        "system": "سسٹم",
        "development": "ترقی",
        "framework": "ڈھانچہ",
        "technology": "ٹیکنالوجی",
        "process": "عمل",
        "method": "طریقہ",
        "improve": "بہتری",
        "performance": "کارکردگی",
    }

    @staticmethod
    def translate_text(text: str, target_language: str = "ur") -> str:
        """Simple translation using pre-built terminology."""
        if target_language != "ur":
            return text

        translated = text

        # Sort by length (longest first) to avoid partial replacements
        sorted_terms = sorted(
            SimpleTranslator.URDU_DICTIONARY.items(),
            key=lambda x: len(x[0]),
            reverse=True
        )

        for english, urdu in sorted_terms:
            # Case-insensitive replacement with word boundaries
            pattern = r"\b" + re.escape(english) + r"\b"
            translated = re.sub(pattern, urdu, translated, flags=re.IGNORECASE)

        return translated


class TranslationService:
    """Service for managing content translation."""

    def __init__(self, db: Session):
        """Initialize translation service."""
        self.db = db
        self.terminology_service = TerminologyService(db)

        # Initialize translator
        if GOOGLE_TRANSLATE_AVAILABLE:
            try:
                self.translator = translate_v2.Client()
                self.translator_available = True
            except Exception as e:
                logger.warning(f"Google Translate not available: {str(e)}")
                self.translator_available = False
        else:
            self.translator_available = False

    def translate_content(
        self,
        user_id: str,
        content_id: str,
        content_text: str,
        content_type: str = "section",
        target_language: str = "ur",
    ) -> Optional[Dict[str, Any]]:
        """Translate content with terminology replacement and caching."""
        try:
            # Get user's profile
            from .auth_models import UserProfile
            user_profile = self.db.query(UserProfile).filter(
                UserProfile.user_id == user_id
            ).first()

            if not user_profile:
                return None

            # Get or create a preference for this user
            preference = self.db.query(ContentPreference).filter(
                ContentPreference.user_profile_id == user_profile.id,
                ContentPreference.module_id.is_(None),
                ContentPreference.chapter_id.is_(None),
            ).first()

            if not preference:
                # Create a default preference
                preference = ContentPreference(
                    user_profile_id=user_profile.id,
                    content_level="beginner",
                )
                self.db.add(preference)
                self.db.flush()  # Get the preference ID without committing yet

            cached = self.db.query(Translation).filter(
                and_(
                    Translation.content_preference_id == preference.id,
                    Translation.content_id == content_id,
                    Translation.language == target_language,
                    Translation.content_type == content_type,
                )
            ).first()

            if cached:
                logger.info(f"Translation cache hit for {content_id}")
                return self._translation_to_dict(cached)

            # Perform translation
            translated_text = content_text

            # Step 1: Get terminology
            terminology = self.terminology_service.get_terminology()

            # Step 2: Replace terminology first (preserves consistency)
            translated_text, used_terms = self.terminology_service.replace_terminology(
                translated_text, terminology
            )

            # Step 3: Use SimpleTranslator for Urdu translation
            if target_language == "ur":
                logger.info(f"SimpleTranslator: Translating '{translated_text[:50]}...' to Urdu")
                translated_text = SimpleTranslator.translate_text(translated_text, target_language="ur")
                logger.info(f"SimpleTranslator result: '{translated_text[:50]}...'")
            # Step 4: Use Google Translate for remaining text if available (backup)
            elif self.translator_available:
                try:
                    result = self.translator.translate_text(
                        translated_text,
                        target_language=target_language,
                        format_="text",
                    )
                    translated_text = result.get("translatedText", translated_text)
                except Exception as e:
                    logger.warning(f"Google Translate failed: {str(e)}, using terminology-only")

            # Calculate word count
            word_count = len(content_text.split())

            # Create translation record
            translation = Translation(
                content_preference_id=preference.id,
                content_id=content_id,
                language=target_language,
                content_type=content_type,
                original_content=content_text,
                translated_content=translated_text,
                translation_status="auto_generated",
                accuracy_score=85.0,  # Default score for auto-generated
                word_count=word_count,
                is_cached=True,
                terminology_used=json.dumps(used_terms),
                custom_terms_applied=len(used_terms),
            )

            self.db.add(translation)
            self.db.commit()

            logger.info(f"Translation created for {content_id}, {word_count} words")

            return self._translation_to_dict(translation)

        except Exception as e:
            self.db.rollback()
            logger.error(f"Error translating content: {str(e)}")
            return None

    def get_translation(
        self,
        content_id: str,
        target_language: str = "ur",
        preference_id: Optional[str] = None,
    ) -> Optional[Dict[str, Any]]:
        """Get cached translation."""
        try:
            query = self.db.query(Translation).filter(
                and_(
                    Translation.content_id == content_id,
                    Translation.language == target_language,
                )
            )

            if preference_id:
                query = query.filter(Translation.content_preference_id == preference_id)

            translation = query.first()
            if translation:
                return self._translation_to_dict(translation)

            return None

        except Exception as e:
            logger.error(f"Error getting translation: {str(e)}")
            return None

    def get_translation_status(
        self,
        content_id: str,
        target_language: str = "ur",
    ) -> Optional[Dict[str, Any]]:
        """Get translation status."""
        try:
            translation = self.db.query(Translation).filter(
                and_(
                    Translation.content_id == content_id,
                    Translation.language == target_language,
                )
            ).first()

            if not translation:
                return {
                    "content_id": content_id,
                    "status": "not_translated",
                    "available": False,
                }

            return {
                "content_id": content_id,
                "status": translation.translation_status,
                "available": True,
                "accuracy_score": translation.accuracy_score,
                "word_count": translation.word_count,
                "terminology_used": len(json.loads(translation.terminology_used or "[]")),
                "created_at": translation.created_at.isoformat(),
            }

        except Exception as e:
            logger.error(f"Error getting translation status: {str(e)}")
            return None

    def batch_translate_chapters(
        self,
        user_id: str,
        chapters: List[Dict[str, str]],
        target_language: str = "ur",
    ) -> List[Dict[str, Any]]:
        """Translate multiple chapters efficiently (batch)."""
        results = []

        try:
            for chapter in chapters:
                result = self.translate_content(
                    user_id=user_id,
                    content_id=chapter.get("id"),
                    content_text=chapter.get("content"),
                    content_type="chapter",
                    target_language=target_language,
                )
                if result:
                    results.append(result)

            logger.info(f"Batch translated {len(results)} chapters")
            return results

        except Exception as e:
            logger.error(f"Error in batch translation: {str(e)}")
            return results

    def clear_translation_cache(self, content_id: Optional[str] = None) -> int:
        """Clear translation cache."""
        try:
            if content_id:
                count = (
                    self.db.query(Translation)
                    .filter(Translation.content_id == content_id)
                    .delete()
                )
            else:
                count = self.db.query(Translation).delete()

            self.db.commit()
            logger.info(f"Cleared {count} translation cache entries")
            return count

        except Exception as e:
            self.db.rollback()
            logger.error(f"Error clearing cache: {str(e)}")
            return 0

    @staticmethod
    def _translation_to_dict(translation: Translation) -> Dict[str, Any]:
        """Convert Translation model to dictionary."""
        return {
            "id": translation.id,
            "content_id": translation.content_id,
            "language": translation.language,
            "content_type": translation.content_type,
            "original_content": translation.original_content,
            "translated_content": translation.translated_content,
            "status": translation.translation_status,
            "accuracy_score": translation.accuracy_score,
            "word_count": translation.word_count,
            "is_cached": translation.is_cached,
            "terminology_used": json.loads(translation.terminology_used or "[]"),
            "custom_terms_applied": translation.custom_terms_applied,
            "created_at": translation.created_at.isoformat(),
            "updated_at": translation.updated_at.isoformat(),
        }
