"""
Secure system test for RAG Chatbot Backend.
Loads credentials from .env file - NEVER hardcodes secrets.
"""
import os
import json
import sys
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables from .env
env_path = Path(__file__).parent.parent / '.env'
load_dotenv(env_path)

def print_section(title):
    """Print a formatted section header."""
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}\n")

def test_imports():
    """Test that all modules can be imported."""
    print_section("[1/7] Testing Module Imports")

    try:
        from src.app import app
        print("[OK] FastAPI app imported")

        from src.services.auth_service import get_auth_service
        print("[OK] Auth service imported")

        from src.services.rag_service import get_rag_service
        print("[OK] RAG service imported")

        from src.services.personalization_service import get_personalization_service
        print("[OK] Personalization service imported")

        return True, app
    except Exception as e:
        print(f"[ERROR] Import failed: {e}")
        import traceback
        traceback.print_exc()
        return False, None

def test_health_check(client):
    """Test health check endpoint."""
    print_section("[2/7] Testing Health Check")

    try:
        response = client.get('/health')
        if response.status_code == 200:
            data = response.json()
            print(f"[OK] Health check passed")
            print(f"     Status: {data['status']}")
            print(f"     Service: {data['service']}")
            return True
        else:
            print(f"[ERROR] Health check failed with status {response.status_code}")
            return False
    except Exception as e:
        print(f"[ERROR] Health check error: {e}")
        return False

def test_user_signup(client):
    """Test user signup endpoint."""
    print_section("[3/7] Testing User Signup")

    try:
        signup_data = {
            "email": "testuser@example.com",
            "username": "testuser123",
            "password": "TestPassword123",
            "full_name": "Test User",
            "confirm_password": "TestPassword123"
        }

        response = client.post('/auth/signup', json=signup_data)

        if response.status_code == 200:
            data = response.json()
            print(f"[OK] Signup successful")
            print(f"     User ID: {data.get('user_id')}")
            print(f"     Email: {data.get('email')}")
            return True, data.get('user_id')
        else:
            print(f"[ERROR] Signup failed with status {response.status_code}")
            print(f"     Response: {response.text}")
            return False, None
    except Exception as e:
        print(f"[ERROR] Signup error: {e}")
        return False, None

def test_user_signin(client):
    """Test user signin endpoint."""
    print_section("[4/7] Testing User Signin")

    try:
        signin_data = {
            "email": "testuser@example.com",
            "password": "TestPassword123"
        }

        response = client.post('/auth/signin', json=signin_data)

        if response.status_code == 200:
            data = response.json()
            token = data.get('access_token')
            print(f"[OK] Signin successful")
            print(f"     Token type: {data.get('token_type')}")
            print(f"     User: {data.get('user', {}).get('email')}")
            return True, token
        else:
            print(f"[ERROR] Signin failed with status {response.status_code}")
            print(f"     Response: {response.text}")
            return False, None
    except Exception as e:
        print(f"[ERROR] Signin error: {e}")
        return False, None

def test_save_background(client, token):
    """Test saving user background profile."""
    print_section("[5/7] Testing Save Background Profile")

    try:
        profile_data = {
            "software_background": "intermediate",
            "hardware_background": "beginner",
            "programming_languages": ["Python", "JavaScript", "C++"],
            "interest_areas": ["ROS", "Kinematics", "Humanoid"],
            "profession": "Software Engineer",
            "organization": "Tech Company",
            "years_of_experience": 5,
            "preferred_learning_style": "interactive",
            "learning_pace": "moderate"
        }

        headers = {"Authorization": f"Bearer {token}"}
        response = client.post('/auth/profile/background',
                             json=profile_data,
                             headers=headers)

        if response.status_code == 200:
            data = response.json()
            print(f"[OK] Background profile saved")
            print(f"     Software level: {data.get('software_background')}")
            print(f"     Interests: {', '.join(data.get('interest_areas', []))}")
            return True
        else:
            print(f"[ERROR] Failed to save background with status {response.status_code}")
            print(f"     Response: {response.text}")
            return False
    except Exception as e:
        print(f"[ERROR] Background save error: {e}")
        return False

def test_chat_query(client, token):
    """Test chat query endpoint."""
    print_section("[6/7] Testing Chat Query")

    try:
        # Verify OpenAI API key is configured
        openai_key = os.getenv('OPENAI_API_KEY')
        if not openai_key:
            print("[WARNING] OPENAI_API_KEY not configured - skipping LLM test")
            print("          Set OPENAI_API_KEY in .env to test chat endpoint")
            return True

        query_data = {
            "query": "What is Robot Operating System (ROS)?",
            "conversation_history": []
        }

        headers = {"Authorization": f"Bearer {token}"}
        response = client.post('/chat/query',
                             json=query_data,
                             headers=headers)

        if response.status_code == 200:
            data = response.json()
            print(f"[OK] Chat query successful")
            print(f"     Status: {data.get('status')}")
            print(f"     Confidence: {data.get('confidence', 'N/A'):.2f}")
            if data.get('sources'):
                print(f"     Sources: {len(data.get('sources', []))} retrieved")
            answer_preview = data.get('answer', '')[:100]
            print(f"     Answer preview: {answer_preview}...")
            return True
        else:
            print(f"[ERROR] Chat query failed with status {response.status_code}")
            print(f"     Response: {response.text[:200]}")
            return False
    except Exception as e:
        print(f"[ERROR] Chat query error: {e}")
        return False

def test_personalization(client, token):
    """Test personalization endpoint."""
    print_section("[7/7] Testing Personalization Data")

    try:
        headers = {"Authorization": f"Bearer {token}"}
        response = client.get('/auth/personalization', headers=headers)

        if response.status_code == 200:
            data = response.json()
            print(f"[OK] Personalization data retrieved")
            print(f"     Complexity level: {data.get('content_complexity', 'N/A')}")
            if data.get('recommended_chapters'):
                print(f"     Recommended chapters: {len(data.get('recommended_chapters', []))}")
            return True
        else:
            print(f"[ERROR] Failed to get personalization with status {response.status_code}")
            print(f"     Response: {response.text}")
            return False
    except Exception as e:
        print(f"[ERROR] Personalization error: {e}")
        return False

def main():
    """Run all tests."""
    print("\n" + "="*60)
    print("  RAG CHATBOT BACKEND - SYSTEM TEST")
    print("="*60)
    print("\nTesting secure configuration from .env file\n")

    # Test imports
    success, app = test_imports()
    if not success or app is None:
        print("\n[FATAL] Failed to import app - cannot continue")
        return False

    # Create test client
    from fastapi.testclient import TestClient
    client = TestClient(app)

    all_passed = True

    # Test health check
    if not test_health_check(client):
        all_passed = False

    # Test signup
    signup_success, user_id = test_user_signup(client)
    if not signup_success:
        all_passed = False

    # Test signin
    signin_success, token = test_user_signin(client)
    if not signin_success:
        all_passed = False
    else:
        # Test background profile (requires token)
        if not test_save_background(client, token):
            all_passed = False

        # Test chat query (requires token)
        if not test_chat_query(client, token):
            all_passed = False

        # Test personalization (requires token)
        if not test_personalization(client, token):
            all_passed = False

    # Summary
    print_section("TEST SUMMARY")
    if all_passed:
        print("[SUCCESS] All tests passed!")
        print("\nYour RAG Chatbot Backend is ready for deployment.")
        print("\nNext steps:")
        print("1. Run: uvicorn src.app:app --reload")
        print("2. Access API docs: http://localhost:8000/api/docs")
        print("3. Start the frontend: npm start")
        print("4. Open: http://localhost:3000")
    else:
        print("[ERROR] Some tests failed - check output above")
        return False

    return True

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
