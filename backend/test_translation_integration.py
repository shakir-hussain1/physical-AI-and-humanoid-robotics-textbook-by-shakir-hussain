"""Integration tests for translation feature."""

import sys
import os
import json

# Add backend to path
backend_dir = os.path.dirname(__file__)
sys.path.insert(0, backend_dir)

from src.api.auth_service import AuthService
from src.api.translation_service import TranslationService, TerminologyService
from src.api.personalization_service import PersonalizationService
from src.api.database import SessionLocal, init_db, drop_db


# Robotics terminology for testing
SAMPLE_TERMINOLOGY = [
    {"en": "Robot", "ur": "روبوٹ", "category": "hardware", "approved": True, "priority": 10},
    {"en": "Algorithm", "ur": "الگورتھم", "category": "ai", "approved": True, "priority": 9},
    {"en": "Sensor", "ur": "حسّاس", "category": "hardware", "approved": True, "priority": 8},
    {"en": "Motor", "ur": "موٹر", "category": "hardware", "approved": True, "priority": 8},
    {"en": "Artificial Intelligence", "ur": "مصنوعی ذہانت", "category": "ai", "approved": True, "priority": 10},
    {"en": "Neural Network", "ur": "عصبی نیٹ ورک", "category": "ai", "approved": True, "priority": 9},
]


def setup_test_db():
    """Setup test database."""
    try:
        drop_db()
    except:
        pass
    init_db()


def test_terminology_loading():
    """Test loading robotics terminology."""
    print("\n[TEST 1] Terminology Loading")
    setup_test_db()

    db = SessionLocal()
    terminology_service = TerminologyService(db)

    try:
        count = terminology_service.load_terminology(SAMPLE_TERMINOLOGY)

        assert count == len(SAMPLE_TERMINOLOGY), "Terminology count mismatch"

        # Verify loaded
        terms = terminology_service.get_terminology()
        assert len(terms) == len(SAMPLE_TERMINOLOGY), "Loaded terminology count mismatch"

        print(f"  Loaded {count} terminology entries")
        print(f"  Terms: {[t['english'] for t in terms[:3]]}...")
        print("  [PASS] Terminology loading works correctly")

    finally:
        db.close()


def test_terminology_replacement():
    """Test terminology replacement in text."""
    print("\n[TEST 2] Terminology Replacement")
    setup_test_db()

    db = SessionLocal()
    terminology_service = TerminologyService(db)

    try:
        # Load terminology
        terminology_service.load_terminology(SAMPLE_TERMINOLOGY)

        # Get terminology - should have english and urdu keys
        terms = terminology_service.get_terminology()

        # Test replacement with at least some terms
        text = "A Robot uses sensors and algorithms to move using motors"
        replaced_text, used_terms = terminology_service.replace_terminology(text, terms)

        # Check that some replacement happened
        assert len(used_terms) > 0, "No terms replaced"
        assert replaced_text != text, "Text not modified after replacement"

        print(f"  Original: {text}")
        print(f"  Terms used: {len(used_terms)}")
        print("  [PASS] Terminology replacement works correctly")

    finally:
        db.close()


def test_content_translation():
    """Test content translation with terminology."""
    print("\n[TEST 3] Content Translation")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)
    translation_service = TranslationService(db)

    try:
        # Register user
        user = auth_service.register_user(
            email="translation@example.com",
            password="Test@1234",
        )

        # Set global preference for user
        personalization_service.set_global_preference(
            user_id=user.id,
            content_level="beginner",
        )

        # Load terminology
        term_service = TerminologyService(db)
        term_service.load_terminology(SAMPLE_TERMINOLOGY)

        # Translate content
        content = "This Robot uses sensors and algorithms"
        translation = translation_service.translate_content(
            user_id=user.id,
            content_id="chapter-1",
            content_text=content,
            content_type="chapter",
        )

        assert translation is not None, "Translation failed"
        assert translation["content_id"] == "chapter-1", "Content ID mismatch"
        assert translation["status"] == "auto_generated", "Status mismatch"
        assert translation["word_count"] > 0, "Word count not calculated"
        assert len(translation["terminology_used"]) > 0, "No terminology used"

        print(f"  Original words: {translation['word_count']}")
        print(f"  Terms applied: {translation['custom_terms_applied']}")
        print(f"  Accuracy: {translation['accuracy_score']}%")
        print("  [PASS] Content translation works correctly")

    finally:
        db.close()


def test_translation_caching():
    """Test translation caching mechanism."""
    print("\n[TEST 4] Translation Caching")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)
    translation_service = TranslationService(db)

    try:
        # Setup user
        user = auth_service.register_user(
            email="cache@example.com",
            password="Test@1234",
        )

        personalization_service.set_global_preference(user_id=user.id)

        # First translation
        content1 = "Robot moves forward"
        result1 = translation_service.translate_content(
            user_id=user.id,
            content_id="ch1",
            content_text=content1,
            content_type="chapter",
        )

        assert result1 is not None
        translation_id_1 = result1["id"]

        # Retrieve from cache
        cached = translation_service.get_translation("ch1")
        assert cached is not None, "Cache retrieval failed"
        assert cached["id"] == translation_id_1, "Cache ID mismatch"
        assert cached["is_cached"] == True, "Not marked as cached"

        print("  Translation cached successfully")
        print(f"  Cache hit: {cached['content_id']}")
        print("  [PASS] Translation caching works correctly")

    finally:
        db.close()


def test_batch_translation():
    """Test batch translation of multiple chapters."""
    print("\n[TEST 5] Batch Translation")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)
    translation_service = TranslationService(db)

    try:
        # Setup user
        user = auth_service.register_user(
            email="batch@example.com",
            password="Test@1234",
        )

        personalization_service.set_global_preference(user_id=user.id)

        # Batch translate
        chapters = [
            {"id": "ch1", "content": "Robot one content here"},
            {"id": "ch2", "content": "Robot two content here"},
            {"id": "ch3", "content": "Robot three content here"},
        ]

        results = translation_service.batch_translate_chapters(
            user_id=user.id,
            chapters=chapters,
        )

        assert len(results) == 3, "Batch count mismatch"
        assert all(r["content_type"] == "chapter" for r in results), "Content type mismatch"

        print(f"  Translated {len(results)} chapters in batch")
        print(f"  Chapter IDs: {[r['content_id'] for r in results]}")
        print("  [PASS] Batch translation works correctly")

    finally:
        db.close()


def test_translation_status():
    """Test translation status checking."""
    print("\n[TEST 6] Translation Status")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)
    translation_service = TranslationService(db)

    try:
        # Setup user and translate
        user = auth_service.register_user(
            email="status@example.com",
            password="Test@1234",
        )

        personalization_service.set_global_preference(user_id=user.id)

        # Create translation
        translation_service.translate_content(
            user_id=user.id,
            content_id="ch1",
            content_text="Robot content",
            content_type="chapter",
        )

        # Check status
        status = translation_service.get_translation_status("ch1")

        assert status["available"] == True, "Translation not available"
        assert status["status"] == "auto_generated", "Status mismatch"
        assert status["word_count"] > 0, "Word count missing"

        print(f"  Status: {status['status']}")
        print(f"  Available: {status['available']}")
        print(f"  Word count: {status['word_count']}")
        print("  [PASS] Translation status works correctly")

    finally:
        db.close()


def test_cache_clearing():
    """Test cache clearing mechanism."""
    print("\n[TEST 7] Cache Clearing")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)
    translation_service = TranslationService(db)

    try:
        # Setup and translate
        user = auth_service.register_user(
            email="clear@example.com",
            password="Test@1234",
        )

        personalization_service.set_global_preference(user_id=user.id)

        translation_service.translate_content(
            user_id=user.id,
            content_id="ch1",
            content_text="Robot one",
        )

        translation_service.translate_content(
            user_id=user.id,
            content_id="ch2",
            content_text="Robot two",
        )

        # Verify translations exist
        assert translation_service.get_translation("ch1") is not None
        assert translation_service.get_translation("ch2") is not None

        # Clear specific
        count = translation_service.clear_translation_cache("ch1")
        assert count > 0, "Cache not cleared"

        # Verify specific cleared
        assert translation_service.get_translation("ch1") is None

        # Verify other still exists
        assert translation_service.get_translation("ch2") is not None

        # Clear all
        count_all = translation_service.clear_translation_cache()
        assert translation_service.get_translation("ch2") is None

        print(f"  Cleared specific: {count} entry")
        print(f"  Cleared all: {count_all} entries")
        print("  [PASS] Cache clearing works correctly")

    finally:
        db.close()


def test_partial_content_translation():
    """Test translating partial content (sections)."""
    print("\n[TEST 8] Partial Content Translation")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)
    translation_service = TranslationService(db)

    try:
        # Setup user
        user = auth_service.register_user(
            email="partial@example.com",
            password="Test@1234",
        )

        personalization_service.set_global_preference(user_id=user.id)

        # Translate multiple sections of same chapter
        section1 = "Introduction: Robot basics"
        section2 = "How sensors work"
        section3 = "Motor control"

        result1 = translation_service.translate_content(
            user_id=user.id,
            content_id="ch1-sec1",
            content_text=section1,
            content_type="section",
        )

        result2 = translation_service.translate_content(
            user_id=user.id,
            content_id="ch1-sec2",
            content_text=section2,
            content_type="section",
        )

        result3 = translation_service.translate_content(
            user_id=user.id,
            content_id="ch1-sec3",
            content_text=section3,
            content_type="section",
        )

        assert result1["content_type"] == "section"
        assert result2["content_type"] == "section"
        assert result3["content_type"] == "section"

        # Verify all sections cached separately
        assert translation_service.get_translation("ch1-sec1") is not None
        assert translation_service.get_translation("ch1-sec2") is not None
        assert translation_service.get_translation("ch1-sec3") is not None

        print("  Translated 3 sections of chapter")
        print("  Each section cached separately")
        print("  [PASS] Partial content translation works correctly")

    finally:
        db.close()


def main():
    """Run all tests."""
    print("=" * 60)
    print("TRANSLATION FEATURE INTEGRATION TESTS")
    print("=" * 60)

    try:
        test_terminology_loading()
        test_terminology_replacement()
        test_content_translation()
        test_translation_caching()
        test_batch_translation()
        test_translation_status()
        test_cache_clearing()
        test_partial_content_translation()

        print("\n" + "=" * 60)
        print("ALL TESTS PASSED!")
        print("=" * 60)

    except AssertionError as e:
        print(f"\n[FAIL] {str(e)}")
        return 1
    except Exception as e:
        print(f"\n[ERROR] {str(e)}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        try:
            drop_db()
        except:
            pass

    return 0


if __name__ == "__main__":
    exit(main())
