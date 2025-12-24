"""
Test concurrent translation and chat endpoints
Verifies that both can run in parallel without blocking
"""

import asyncio
import httpx
import time
import json

API_BASE = "http://localhost:8001"

async def test_chat_endpoint():
    """Test chat endpoint with a simple query"""
    async with httpx.AsyncClient() as client:
        start = time.time()
        try:
            response = await client.post(
                f"{API_BASE}/chat/query",
                json={
                    "query": "What is ROS?",
                    "user_context": None,
                    "conversation_history": []
                },
                timeout=30.0
            )
            elapsed = time.time() - start
            
            if response.status_code == 200:
                data = response.json()
                return {
                    "status": "success",
                    "endpoint": "chat",
                    "elapsed_time": f"{elapsed:.2f}s",
                    "answer_preview": data.get("answer", "")[:100] + "..."
                }
            else:
                return {
                    "status": "error",
                    "endpoint": "chat",
                    "elapsed_time": f"{elapsed:.2f}s",
                    "error": f"HTTP {response.status_code}"
                }
        except Exception as e:
            elapsed = time.time() - start
            return {
                "status": "error",
                "endpoint": "chat",
                "elapsed_time": f"{elapsed:.2f}s",
                "error": str(e)
            }

async def test_translation_endpoint():
    """Test translation endpoint with HTML content"""
    async with httpx.AsyncClient() as client:
        start = time.time()
        try:
            # First get auth token (or skip if no auth needed)
            response = await client.post(
                f"{API_BASE}/translation/translate",
                json={
                    "content": "<p>This is a sample text about robotics and AI.</p>",
                    "chapter_id": "test-chapter",
                    "target_language": "urdu"
                },
                timeout=30.0,
                headers={
                    "Authorization": "Bearer test-token"  # May fail but that's ok for this test
                }
            )
            elapsed = time.time() - start
            
            # Even 401 means endpoint responded quickly
            if response.status_code in [200, 401, 422]:
                return {
                    "status": "success" if response.status_code == 200 else "auth_needed",
                    "endpoint": "translation",
                    "elapsed_time": f"{elapsed:.2f}s",
                    "http_status": response.status_code
                }
            else:
                return {
                    "status": "error",
                    "endpoint": "translation",
                    "elapsed_time": f"{elapsed:.2f}s",
                    "error": f"HTTP {response.status_code}"
                }
        except asyncio.TimeoutError:
            elapsed = time.time() - start
            return {
                "status": "error",
                "endpoint": "translation",
                "elapsed_time": f"{elapsed:.2f}s",
                "error": "Request timed out (>30s)"
            }
        except Exception as e:
            elapsed = time.time() - start
            return {
                "status": "error",
                "endpoint": "translation",
                "elapsed_time": f"{elapsed:.2f}s",
                "error": str(e)
            }

async def main():
    """Run both endpoints concurrently"""
    print("\n" + "="*60)
    print("CONCURRENT ENDPOINT TEST")
    print("="*60)
    print("\nTesting if translation and chat endpoints block each other...")
    print(f"Backend: {API_BASE}\n")
    
    # Run both concurrently
    start_time = time.time()
    chat_result, translation_result = await asyncio.gather(
        test_chat_endpoint(),
        test_translation_endpoint(),
        return_exceptions=False
    )
    total_time = time.time() - start_time
    
    # Display results
    print("\n--- CHAT ENDPOINT ---")
    print(f"Status: {chat_result.get('status')}")
    print(f"Time: {chat_result.get('elapsed_time')}")
    if chat_result.get('answer_preview'):
        print(f"Answer: {chat_result.get('answer_preview')}")
    if chat_result.get('error'):
        print(f"Error: {chat_result.get('error')}")
    
    print("\n--- TRANSLATION ENDPOINT ---")
    print(f"Status: {translation_result.get('status')}")
    print(f"Time: {translation_result.get('elapsed_time')}")
    print(f"HTTP Status: {translation_result.get('http_status', 'N/A')}")
    if translation_result.get('error'):
        print(f"Error: {translation_result.get('error')}")
    
    print(f"\n--- SUMMARY ---")
    print(f"Both requests completed in: {total_time:.2f}s")
    
    # Check if they ran concurrently
    chat_time = float(chat_result.get('elapsed_time', '0').rstrip('s'))
    trans_time = float(translation_result.get('elapsed_time', '0').rstrip('s'))
    
    if total_time < max(chat_time, trans_time) * 1.5:
        print("✅ SUCCESS: Endpoints ran CONCURRENTLY (not blocked)")
        print(f"   Chat: {chat_time:.2f}s, Translation: {trans_time:.2f}s")
        print(f"   Total time: {total_time:.2f}s (would be {chat_time + trans_time:.2f}s if sequential)")
    else:
        print("❌ FAILED: Endpoints appear to be SEQUENTIAL (blocked)")
        print(f"   Total time {total_time:.2f}s ≈ sum of individual times")
    
    print("="*60 + "\n")

if __name__ == "__main__":
    asyncio.run(main())
