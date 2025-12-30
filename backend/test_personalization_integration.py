"""Integration tests for personalization feature."""

import sys
import os
import json

# Add backend to path
backend_dir = os.path.dirname(__file__)
sys.path.insert(0, backend_dir)

from src.api.auth_service import AuthService
from src.api.personalization_service import PersonalizationService
from src.api.database import SessionLocal, init_db, drop_db


def setup_test_db():
    """Setup test database."""
    try:
        drop_db()
    except:
        pass
    init_db()


def test_global_preference():
    """Test setting and retrieving global preference."""
    print("\n[TEST 1] Global Preference")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)

    try:
        # Register test user
        user = auth_service.register_user(
            email="global@example.com",
            password="Test@1234",
        )

        # Set global preference
        pref = personalization_service.set_global_preference(
            user_id=user.id,
            content_level="intermediate",
            show_math=True,
            show_code=True,
            show_diagrams=False,
            show_advanced_topics=True,
            focus_areas=["robotics", "AI"],
        )

        assert pref is not None, "Failed to set global preference"
        assert pref["content_level"] == "intermediate", "Content level mismatch"
        assert pref["show_diagrams"] == False, "show_diagrams mismatch"
        assert pref["show_advanced_topics"] == True, "show_advanced_topics mismatch"
        assert "robotics" in pref["focus_areas"], "Focus area not saved"

        print(f"  Global preference set: {pref['content_level']}")
        print(f"  Focus areas: {pref['focus_areas']}")
        print("  [PASS] Global preference works correctly")

    finally:
        db.close()


def test_module_preference():
    """Test setting and retrieving module preference."""
    print("\n[TEST 2] Module Preference")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)

    try:
        # Register test user
        user = auth_service.register_user(
            email="module@example.com",
            password="Test@1234",
        )

        # Set module preference
        pref = personalization_service.set_module_preference(
            user_id=user.id,
            module_id="module-1",
            content_level="advanced",
            show_math=True,
            show_code=False,
            show_diagrams=True,
        )

        assert pref is not None, "Failed to set module preference"
        assert pref["module_id"] == "module-1", "Module ID mismatch"
        assert pref["content_level"] == "advanced", "Content level mismatch"
        assert pref["show_code"] == False, "show_code mismatch"

        print(f"  Module preference set for module-1: {pref['content_level']}")
        print("  [PASS] Module preference works correctly")

    finally:
        db.close()


def test_chapter_preference():
    """Test setting and retrieving chapter preference."""
    print("\n[TEST 3] Chapter Preference")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)

    try:
        # Register test user
        user = auth_service.register_user(
            email="chapter@example.com",
            password="Test@1234",
        )

        # Set chapter preference
        pref = personalization_service.set_chapter_preference(
            user_id=user.id,
            chapter_id="chapter-1",
            module_id="module-1",
            content_level="beginner",
            show_math=False,
            show_code=True,
        )

        assert pref is not None, "Failed to set chapter preference"
        assert pref["chapter_id"] == "chapter-1", "Chapter ID mismatch"
        assert pref["module_id"] == "module-1", "Module ID mismatch"
        assert pref["show_math"] == False, "show_math mismatch"

        print(f"  Chapter preference set for chapter-1: {pref['content_level']}")
        print("  [PASS] Chapter preference works correctly")

    finally:
        db.close()


def test_preference_hierarchy():
    """Test preference hierarchy (chapter > module > global)."""
    print("\n[TEST 4] Preference Hierarchy")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)

    try:
        # Register test user
        user = auth_service.register_user(
            email="hierarchy@example.com",
            password="Test@1234",
        )

        # Set global preference
        personalization_service.set_global_preference(
            user_id=user.id,
            content_level="beginner",
            show_advanced_topics=False,
        )

        # Set module preference
        personalization_service.set_module_preference(
            user_id=user.id,
            module_id="module-1",
            content_level="intermediate",
            show_advanced_topics=False,
        )

        # Set chapter preference
        personalization_service.set_chapter_preference(
            user_id=user.id,
            chapter_id="chapter-1",
            content_level="advanced",
            show_advanced_topics=True,
        )

        # Test hierarchy: chapter takes precedence
        effective = personalization_service.get_effective_preference(
            user_id=user.id,
            chapter_id="chapter-1",
            module_id="module-1",
        )

        assert effective["content_level"] == "advanced", "Chapter preference not applied"
        assert effective["show_advanced_topics"] == True, "Chapter show_advanced_topics not applied"

        print("  Chapter preference applied when chapter_id provided")

        # Test module preference when chapter not specified
        effective = personalization_service.get_effective_preference(
            user_id=user.id,
            chapter_id=None,
            module_id="module-1",
        )

        assert effective["content_level"] == "intermediate", "Module preference not applied"

        print("  Module preference applied when chapter_id not provided")

        # Test global preference when neither specified
        effective = personalization_service.get_effective_preference(
            user_id=user.id,
            chapter_id=None,
            module_id=None,
        )

        assert effective["content_level"] == "beginner", "Global preference not applied"

        print("  Global preference applied when neither chapter_id nor module_id provided")
        print("  [PASS] Preference hierarchy works correctly")

    finally:
        db.close()


def test_get_all_preferences():
    """Test getting all preferences organized by level."""
    print("\n[TEST 5] Get All Preferences")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)

    try:
        # Register test user
        user = auth_service.register_user(
            email="all@example.com",
            password="Test@1234",
        )

        # Set various preferences
        personalization_service.set_global_preference(
            user_id=user.id,
            content_level="beginner",
        )

        personalization_service.set_module_preference(
            user_id=user.id,
            module_id="module-1",
            content_level="intermediate",
        )

        personalization_service.set_module_preference(
            user_id=user.id,
            module_id="module-2",
            content_level="advanced",
        )

        personalization_service.set_chapter_preference(
            user_id=user.id,
            chapter_id="chapter-1-1",
            content_level="intermediate",
        )

        # Get all preferences
        all_prefs = personalization_service.get_all_preferences(user.id)

        assert all_prefs is not None, "Failed to get all preferences"
        assert all_prefs["global"] is not None, "Global preference missing"
        assert len(all_prefs["modules"]) == 2, "Module preferences count mismatch"
        assert len(all_prefs["chapters"]) == 1, "Chapter preferences count mismatch"

        print(f"  Global: 1 preference")
        print(f"  Modules: {len(all_prefs['modules'])} preferences")
        print(f"  Chapters: {len(all_prefs['chapters'])} preferences")
        print("  [PASS] Get all preferences works correctly")

    finally:
        db.close()


def test_update_preference():
    """Test updating existing preference."""
    print("\n[TEST 6] Update Preference")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)

    try:
        # Register test user
        user = auth_service.register_user(
            email="update@example.com",
            password="Test@1234",
        )

        # Set initial preference
        pref1 = personalization_service.set_global_preference(
            user_id=user.id,
            content_level="beginner",
            show_math=True,
        )

        assert pref1["content_level"] == "beginner"

        # Update preference
        pref2 = personalization_service.set_global_preference(
            user_id=user.id,
            content_level="advanced",
            show_math=False,
        )

        assert pref2["content_level"] == "advanced", "Content level not updated"
        assert pref2["show_math"] == False, "show_math not updated"

        # Verify only one preference exists (not duplicated)
        all_prefs = personalization_service.get_all_preferences(user.id)
        assert all_prefs["global"] is not None
        assert all_prefs["global"]["content_level"] == "advanced"

        print("  Preference updated successfully")
        print(f"  New content level: {pref2['content_level']}")
        print("  [PASS] Update preference works correctly")

    finally:
        db.close()


def test_delete_preference():
    """Test deleting preference."""
    print("\n[TEST 7] Delete Preference")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)
    personalization_service = PersonalizationService(db)

    try:
        # Register test user
        user = auth_service.register_user(
            email="delete@example.com",
            password="Test@1234",
        )

        # Set preferences
        personalization_service.set_global_preference(
            user_id=user.id,
            content_level="beginner",
        )

        personalization_service.set_chapter_preference(
            user_id=user.id,
            chapter_id="chapter-1",
            content_level="advanced",
        )

        # Delete chapter preference
        deleted = personalization_service.delete_preference(
            user_id=user.id,
            chapter_id="chapter-1",
        )

        assert deleted == True, "Failed to delete preference"

        # Verify deletion
        all_prefs = personalization_service.get_all_preferences(user.id)
        assert len(all_prefs["chapters"]) == 0, "Chapter preference not deleted"
        assert all_prefs["global"] is not None, "Global preference should still exist"

        print("  Chapter preference deleted successfully")
        print("  [PASS] Delete preference works correctly")

    finally:
        db.close()


def main():
    """Run all tests."""
    print("=" * 60)
    print("PERSONALIZATION FEATURE INTEGRATION TESTS")
    print("=" * 60)

    try:
        test_global_preference()
        test_module_preference()
        test_chapter_preference()
        test_preference_hierarchy()
        test_get_all_preferences()
        test_update_preference()
        test_delete_preference()

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
