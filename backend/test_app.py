"""Test script to verify the RAG Chatbot Backend application works correctly."""
import asyncio
import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

def test_imports():
    """Test that all modules can be imported without errors."""
    print("Testing module imports...")

    try:
        from src.main import app
        print("✓ Main app imported successfully")

        from src.config import settings
        print("✓ Config imported successfully")

        from src.api import health, query, context, content
        print("✓ API modules imported successfully")

        from src.services import retrieval, generation, citations, content_ingest, audit_logger
        print("✓ Service modules imported successfully")

        from src.middleware import error_handler, logging_middleware, api_key_auth
        print("✓ Middleware modules imported successfully")

        from src.models import schemas, entities, audit
        print("✓ Model modules imported successfully")

        from src.utils import constants, exceptions, validation, tracing
        print("✓ Utility modules imported successfully")

        from src.db import postgres, qdrant_client
        print("✓ Database modules imported successfully")

        print("✓ All imports successful!")
        return True

    except Exception as e:
        print(f"[ERROR] Import error: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_api_endpoints():
    """Test that API endpoints are properly registered."""
    print("\nTesting API endpoints...")

    try:
        from src.main import app

        # Check that routes are registered
        routes = [route.path for route in app.routes]

        expected_routes = [
            "/health",
            "/chat/query",
            "/chat/context-restricted",
            "/content/ingest"
        ]

        for route in expected_routes:
            if route in routes:
                print(f"✓ Route {route} is registered")
            else:
                print(f"[ERROR] Route {route} is missing")
                return False

        print("✓ All expected endpoints are registered!")
        return True

    except Exception as e:
        print(f"[ERROR] Endpoint test error: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_services():
    """Test that services can be instantiated."""
    print("\nTesting service instantiation...")

    try:
        from src.config import settings
        from src.services.retrieval import RetrieverService
        from src.services.generation import GenerationService
        from src.services.citations import CitationService
        from src.services.content_ingest import ContentIngestService
        from src.services.audit_logger import AuditLoggerService

        # Test service creation (using mock API keys for testing)
        retriever = RetrieverService(qdrant_url="http://localhost:6333", cohere_key="test-key")
        print("✓ RetrieverService instantiated")

        generation = GenerationService(anthropic_api_key="test-key")
        print("✓ GenerationService instantiated")

        citation = CitationService()
        print("✓ CitationService instantiated")

        content_ingest = ContentIngestService(cohere_api_key="test-key", qdrant_url="http://localhost:6333")
        print("✓ ContentIngestService instantiated")

        audit_logger = AuditLoggerService()
        print("✓ AuditLoggerService instantiated")

        print("✓ All services instantiated successfully!")
        return True

    except Exception as e:
        print(f"[ERROR] Service instantiation error: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all tests."""
    print("Starting RAG Chatbot Backend validation tests...\n")

    all_passed = True

    # Test imports
    if not test_imports():
        all_passed = False

    # Test API endpoints
    if not test_api_endpoints():
        all_passed = False

    # Test services
    if not test_services():
        all_passed = False

    print(f"\n{'='*50}")
    if all_passed:
        print("🎉 All tests passed! The RAG Chatbot Backend is ready for the hackathon.")
        print("\nNext steps:")
        print("1. Run 'uvicorn src.main:app --reload' to start the server")
        print("2. The API will be available at http://localhost:8000")
        print("3. Health check: GET /health")
        print("4. Query endpoint: POST /chat/query")
        print("5. Context-restricted: POST /chat/context-restricted")
        print("6. Content ingestion: POST /content/ingest")
    else:
        print("[ERROR] Some tests failed. Please check the errors above.")
        sys.exit(1)

if __name__ == "__main__":
    main()