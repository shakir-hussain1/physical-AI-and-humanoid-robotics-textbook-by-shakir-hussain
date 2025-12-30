"""Integration tests for authentication endpoints."""

import sys
import os
import asyncio
import json

# Add backend to path
backend_dir = os.path.dirname(__file__)
sys.path.insert(0, backend_dir)

# Direct imports to bypass __init__.py issues
from src.api.auth_service import AuthService, PasswordService, TokenService
from src.api.database import SessionLocal, init_db, drop_db
from src.api.auth_models import User, UserProfile

def setup_test_db():
    """Setup test database."""
    try:
        drop_db()
    except:
        pass
    init_db()

def test_password_hashing():
    """Test password hashing functionality."""
    print("\n[TEST 1] Password Hashing")
    password = "Test@1234"
    hashed = PasswordService.hash_password(password)

    print(f"  Original password: {password}")
    print(f"  Hashed password: {hashed[:50]}...")
    print(f"  Hash length: {len(hashed)}")

    # Verify password
    assert PasswordService.verify_password(password, hashed), "Password verification failed"
    assert not PasswordService.verify_password("WrongPassword@123", hashed), "Wrong password should fail"

    print("  [PASS] Password hashing works correctly")

def test_token_generation():
    """Test JWT token generation."""
    print("\n[TEST 2] JWT Token Generation")
    user_id = "test-user-123"
    email = "test@example.com"

    # Create tokens
    access_token = TokenService.create_access_token(user_id, email)
    refresh_token = TokenService.create_refresh_token(user_id, email)

    print(f"  Access token: {access_token[:50]}...")
    print(f"  Refresh token: {refresh_token[:50]}...")

    # Verify tokens
    access_payload = TokenService.verify_token(access_token)
    refresh_payload = TokenService.verify_token(refresh_token)

    assert access_payload is not None, "Access token verification failed"
    assert refresh_payload is not None, "Refresh token verification failed"
    assert access_payload.get("sub") == user_id, "Token sub claim mismatch"
    assert access_payload.get("type") == "access", "Token type mismatch for access token"
    assert refresh_payload.get("type") == "refresh", "Token type mismatch for refresh token"

    print("  [PASS] JWT token generation works correctly")

def test_token_refresh():
    """Test token refresh functionality."""
    print("\n[TEST 3] Token Refresh")
    user_id = "test-user-123"
    email = "test@example.com"

    # Create refresh token
    refresh_token = TokenService.create_refresh_token(user_id, email)
    print(f"  Original refresh token: {refresh_token[:50]}...")

    # Refresh access token
    new_access_token = TokenService.refresh_access_token(refresh_token)
    assert new_access_token is not None, "Token refresh failed"

    print(f"  New access token: {new_access_token[:50]}...")

    # Verify new access token
    payload = TokenService.verify_token(new_access_token)
    assert payload.get("sub") == user_id, "New token sub claim mismatch"
    assert payload.get("email") == email, "New token email claim mismatch"

    print("  [PASS] Token refresh works correctly")

def test_user_registration():
    """Test user registration."""
    print("\n[TEST 4] User Registration")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)

    try:
        user = auth_service.register_user(
            email="testuser@example.com",
            password="Test@1234",
            first_name="Test",
            last_name="User",
            software_background="beginner",
            hardware_background="intermediate",
            programming_languages=["Python", "JavaScript"],
            robotics_experience="some",
            ai_ml_experience="none",
        )

        assert user is not None, "User registration returned None"
        assert user.email == "testuser@example.com", "Email mismatch"
        assert user.first_name == "Test", "First name mismatch"
        assert user.last_name == "User", "Last name mismatch"
        assert user.is_active == True, "User should be active"
        assert user.profile is not None, "User profile not created"

        print(f"  User ID: {user.id}")
        print(f"  Email: {user.email}")
        print(f"  Profile: {user.profile.software_background} software, {user.profile.hardware_background} hardware")
        print("  [PASS] User registration works correctly")

        # Test duplicate registration
        try:
            auth_service.register_user(
                email="testuser@example.com",
                password="Test@5678",
            )
            assert False, "Should not allow duplicate registration"
        except ValueError as e:
            print(f"  [PASS] Duplicate registration correctly rejected: {str(e)}")

    finally:
        db.close()

def test_user_authentication():
    """Test user authentication."""
    print("\n[TEST 5] User Authentication")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)

    try:
        # Register user
        user = auth_service.register_user(
            email="logintest@example.com",
            password="Test@1234",
            first_name="Login",
            last_name="Test",
        )

        print(f"  Registered user: {user.email}")

        # Test successful login
        result = auth_service.authenticate_user(
            email="logintest@example.com",
            password="Test@1234",
        )

        assert result is not None, "Authentication failed"
        assert result["user_id"] == user.id, "User ID mismatch"
        assert result["email"] == "logintest@example.com", "Email mismatch"
        assert "access_token" in result, "Access token missing"
        assert "refresh_token" in result, "Refresh token missing"

        print(f"  Access token: {result['access_token'][:50]}...")
        print(f"  Refresh token: {result['refresh_token'][:50]}...")
        print("  [PASS] Successful authentication")

        # Test wrong password
        result = auth_service.authenticate_user(
            email="logintest@example.com",
            password="WrongPassword@123",
        )
        assert result is None, "Wrong password should fail"
        print("  [PASS] Wrong password correctly rejected")

        # Test non-existent user
        result = auth_service.authenticate_user(
            email="nonexistent@example.com",
            password="Test@1234",
        )
        assert result is None, "Non-existent user should fail"
        print("  [PASS] Non-existent user correctly rejected")

    finally:
        db.close()

def test_profile_retrieval():
    """Test user profile retrieval."""
    print("\n[TEST 6] Profile Retrieval")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)

    try:
        # Register user with background info
        user = auth_service.register_user(
            email="profiletest@example.com",
            password="Test@1234",
            first_name="Profile",
            last_name="Test",
            software_background="advanced",
            hardware_background="expert",
            programming_languages=["Python", "C++", "ROS"],
            robotics_experience="intermediate",
            ai_ml_experience="advanced",
        )

        # Retrieve profile
        profile = auth_service.get_user_profile(user.id)

        assert profile is not None, "Profile retrieval failed"
        assert profile["software_background"] == "advanced", "Software background mismatch"
        assert profile["hardware_background"] == "expert", "Hardware background mismatch"
        assert "Python" in profile["programming_languages"], "Python missing from languages"
        assert "C++" in profile["programming_languages"], "C++ missing from languages"
        assert profile["robotics_experience"] == "intermediate", "Robotics experience mismatch"
        assert profile["ai_ml_experience"] == "advanced", "AI/ML experience mismatch"

        print(f"  Profile retrieved for: {profile['email']}")
        print(f"  Software: {profile['software_background']}")
        print(f"  Hardware: {profile['hardware_background']}")
        print(f"  Languages: {profile['programming_languages']}")
        print("  [PASS] Profile retrieval works correctly")

    finally:
        db.close()

def test_profile_update():
    """Test user profile update."""
    print("\n[TEST 7] Profile Update")
    setup_test_db()

    db = SessionLocal()
    auth_service = AuthService(db)

    try:
        # Register user
        user = auth_service.register_user(
            email="updatetest@example.com",
            password="Test@1234",
            first_name="Update",
            last_name="Test",
        )

        # Update profile
        updated = auth_service.update_user_profile(
            user.id,
            software_background="expert",
            preferred_language="ur",
            theme="dark",
            show_advanced_content=True,
        )

        assert updated is not None, "Profile update failed"
        assert updated["software_background"] == "expert", "Software background not updated"
        assert updated["preferred_language"] == "ur", "Preferred language not updated"
        assert updated["theme"] == "dark", "Theme not updated"
        assert updated["show_advanced_content"] == True, "Show advanced content not updated"

        print(f"  Profile updated for: {updated['email']}")
        print(f"  New software background: {updated['software_background']}")
        print(f"  New language preference: {updated['preferred_language']}")
        print(f"  New theme: {updated['theme']}")
        print("  [PASS] Profile update works correctly")

    finally:
        db.close()

def main():
    """Run all tests."""
    print("=" * 60)
    print("AUTHENTICATION SYSTEM INTEGRATION TESTS")
    print("=" * 60)

    try:
        test_password_hashing()
        test_token_generation()
        test_token_refresh()
        test_user_registration()
        test_user_authentication()
        test_profile_retrieval()
        test_profile_update()

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
