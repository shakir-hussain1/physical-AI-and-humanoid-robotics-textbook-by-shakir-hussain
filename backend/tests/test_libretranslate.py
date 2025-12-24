#!/usr/bin/env python3
"""
Test LibreTranslate integration for chapter translation
"""

import requests
import json

# Sample chapter content
SAMPLE_HTML = """
<h1>Welcome to Robotics</h1>
<p>This comprehensive chapter introduces you to the fundamentals of robotics and artificial intelligence systems. You will learn about robot design, control systems, sensors, motors, and programming techniques used in modern robotics applications.</p>
<h2>Key Concepts</h2>
<ul>
<li>Perception systems: How robots sense their environment</li>
<li>Decision making algorithms: How robots make intelligent choices</li>
<li>Physical interaction: How robots interact with the world</li>
</ul>
<p>By the end of this chapter, you will understand the basic principles of how robots perceive their environment, make intelligent decisions, and interact with the physical world around them.</p>
"""

def test_translation():
    """Test the translation endpoint"""
    print("=" * 70)
    print("TESTING LIBRETRANSLATE INTEGRATION")
    print("=" * 70)
    print()

    # Backend URL
    backend_url = "http://localhost:8000"

    # Mock token for testing
    token = "mock-test-token-12345"

    # Test translation
    print("Sending translation request...")
    print(f"Content length: {len(SAMPLE_HTML)} chars")
    print(f"Target language: Urdu (ur)")
    print()

    payload = {
        "content": SAMPLE_HTML,
        "chapter_id": "module-1-chapter-01",
        "target_language": "urdu"
    }

    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    try:
        response = requests.post(
            f"{backend_url}/translation/translate",
            json=payload,
            headers=headers,
            timeout=30
        )

        print(f"Response status: {response.status_code}")
        print()

        if response.status_code == 200:
            data = response.json()
            print("SUCCESS! Translation completed:")
            print()
            print(f"Response keys: {list(data.keys())}")
            print(f"Source language: {data.get('source_language')}")
            print(f"Target language: {data.get('target_language')}")
            print(f"Confidence level: {data.get('confidence_level')}")
            print(f"From cache: {data.get('from_cache')}")
            print()

            translated = data.get('translated_content', '')
            print(f"Translated content ({len(translated)} chars):")
            print("-" * 70)
            print(translated[:500])  # Show first 500 chars
            print("...")
            print("-" * 70)
            print()

            # Check if Urdu words are present
            urdu_indicators = ['روبوٹکس', 'اردو', 'کریں', 'یہ']
            found_urdu = any(word in translated for word in urdu_indicators)

            if found_urdu:
                print("✓ Urdu content detected!")
            else:
                print("Searching for Urdu script in translation...")
                if any(ord(c) > 127 for c in translated):
                    print("✓ Non-Latin characters found (likely Urdu)")
                else:
                    print("⚠ No Urdu script detected - may need debugging")

        elif response.status_code == 401:
            print("Authentication failed (expected - using mock token)")
            print("In production, provide valid JWT token")
        else:
            print(f"Error: {response.status_code}")
            print(f"Response: {response.text}")

    except requests.exceptions.Timeout:
        print("⚠ Request timeout - LibreTranslate API may be slow or unavailable")
        print("The free public instance may have rate limits")
    except requests.exceptions.ConnectionError:
        print("✗ Connection error - cannot reach backend or LibreTranslate API")
        print("Check:")
        print("  1. Backend is running on port 8000")
        print("  2. Internet connection is active")
        print("  3. LibreTranslate API is accessible")
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == "__main__":
    test_translation()
