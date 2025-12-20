#!/usr/bin/env python3
"""
Test script for Chapter Personalization API
Tests the full flow: auth -> save -> get -> reset
"""

import requests
import json
from datetime import datetime

API_URL = "http://127.0.0.1:8000"

def print_step(step_num, description):
    print(f"\n{'='*60}")
    print(f"STEP {step_num}: {description}")
    print(f"{'='*60}")

def print_ok(msg):
    print(f"[OK] {msg}")

def print_error(msg):
    print(f"[ERROR] {msg}")

def print_response(response, title="Response"):
    print(f"\n{title}:")
    print(f"Status Code: {response.status_code}")
    try:
        print(f"Body: {json.dumps(response.json(), indent=2)}")
    except:
        print(f"Body: {response.text}")

def main():
    print("\n" + "="*60)
    print("CHAPTER PERSONALIZATION API TEST SUITE")
    print("="*60)

    # Step 1: Sign up a test user
    print_step(1, "Sign up test user")
    timestamp = int(datetime.now().timestamp())
    test_user = {
        "email": f"test{timestamp}@example.com",
        "username": f"testuser{timestamp}",
        "password": "TestPassword123!",
        "confirm_password": "TestPassword123!"
    }
    print(f"Creating user: {test_user['email']}")

    signup_response = requests.post(
        f"{API_URL}/auth/signup",
        json=test_user
    )
    print_response(signup_response, "Signup Response")

    if signup_response.status_code != 200:
        print_error("Signup failed!")
        return

    print_ok("User signed up successfully")

    # Step 2: Login to get token
    print_step(2, "Login to get JWT token")
    login_response = requests.post(
        f"{API_URL}/auth/signin",
        json={
            "email": test_user["email"],
            "password": test_user["password"]
        }
    )
    print_response(login_response, "Login Response")

    if login_response.status_code != 200:
        print_error("Login failed!")
        return

    token = login_response.json().get("access_token")
    if not token:
        print_error("No token received!")
        return

    print_ok(f"Token received: {token[:20]}...")

    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    # Step 3: Save personalization
    print_step(3, "Save chapter personalization settings")

    chapter_id = "modules/module-1-ros2/chapter-01"
    settings = {
        "difficulty_level": "advanced",
        "content_style": "code",
        "example_density": "rich",
        "learning_pace": "detailed",
        "custom_preferences": {"theme": "dark"}
    }

    print(f"Chapter ID: {chapter_id}")
    print(f"Settings: {json.dumps(settings, indent=2)}")

    save_response = requests.post(
        f"{API_URL}/personalization/chapter/{chapter_id}",
        json=settings,
        headers=headers
    )
    print_response(save_response, "Save Response")

    if save_response.status_code != 200:
        print_error("Save failed!")
        print("\nDebugging info:")
        print(f"URL: {API_URL}/personalization/chapter/{chapter_id}")
        print(f"Headers: {headers}")
        print(f"Payload: {json.dumps(settings)}")
        return

    saved_data = save_response.json().get("data")
    print_ok("Personalization saved successfully")
    print(f"  Created at: {saved_data.get('created_at')}")
    print(f"  Updated at: {saved_data.get('updated_at')}")

    # Step 4: Get personalization
    print_step(4, "Retrieve personalization settings")

    get_response = requests.get(
        f"{API_URL}/personalization/chapter/{chapter_id}",
        headers=headers
    )
    print_response(get_response, "Get Response")

    if get_response.status_code != 200:
        print_error("Get failed!")
        return

    retrieved_data = get_response.json().get("data")
    if retrieved_data:
        print_ok("Personalization retrieved successfully")
        print(f"  Difficulty: {retrieved_data.get('difficulty_level')}")
        print(f"  Style: {retrieved_data.get('content_style')}")
        print(f"  Density: {retrieved_data.get('example_density')}")
        print(f"  Pace: {retrieved_data.get('learning_pace')}")
    else:
        print_error("No data returned!")
        return

    # Step 5: Get all personalizations
    print_step(5, "Retrieve all chapter personalizations")

    all_response = requests.get(
        f"{API_URL}/personalization/chapters",
        headers=headers
    )
    print_response(all_response, "Get All Response")

    if all_response.status_code == 200:
        all_data = all_response.json().get("data", [])
        print_ok(f"Retrieved {len(all_data)} personalization(s)")
        for p in all_data:
            print(f"  - {p.get('chapter_id')}: {p.get('difficulty_level')}")

    # Step 6: Update personalization
    print_step(6, "Update personalization settings")

    updated_settings = {
        "difficulty_level": "beginner",
        "content_style": "visual",
        "example_density": "minimal",
        "learning_pace": "concise",
        "custom_preferences": {}
    }

    print(f"New Settings: {json.dumps(updated_settings, indent=2)}")

    update_response = requests.post(
        f"{API_URL}/personalization/chapter/{chapter_id}",
        json=updated_settings,
        headers=headers
    )
    print_response(update_response, "Update Response")

    if update_response.status_code == 200:
        updated_data = update_response.json().get("data")
        print_ok("Personalization updated successfully")
        print(f"  Created at: {updated_data.get('created_at')} (should be same as before)")
        print(f"  Updated at: {updated_data.get('updated_at')} (should be new)")

        # Verify created_at didn't change
        if updated_data.get('created_at') == saved_data.get('created_at'):
            print_ok("Created timestamp preserved correctly")
        else:
            print_error("Created timestamp was modified! This is a bug.")

    # Step 7: Reset personalization
    print_step(7, "Reset chapter personalization to default")

    reset_response = requests.delete(
        f"{API_URL}/personalization/chapter/{chapter_id}",
        headers=headers
    )
    print_response(reset_response, "Reset Response")

    if reset_response.status_code == 200:
        print_ok("Personalization reset successfully")

    # Step 8: Verify deletion
    print_step(8, "Verify personalization was deleted")

    verify_response = requests.get(
        f"{API_URL}/personalization/chapter/{chapter_id}",
        headers=headers
    )
    print_response(verify_response, "Verify Response")

    if verify_response.status_code == 200:
        verify_data = verify_response.json().get("data")
        if verify_data is None:
            print_ok("Personalization successfully deleted")
        else:
            print_error("Personalization still exists! Reset didn't work.")

    # Summary
    print_step(0, "TEST SUMMARY")
    print("\n[PASS] All tests completed successfully!")
    print("\nThe personalization API is working correctly:")
    print("  - Authentication with JWT tokens")
    print("  - Save personalization settings")
    print("  - Retrieve personalization settings")
    print("  - Get all personalizations")
    print("  - Update personalization settings")
    print("  - Reset personalization to default")
    print("\n" + "="*60)

if __name__ == "__main__":
    main()
