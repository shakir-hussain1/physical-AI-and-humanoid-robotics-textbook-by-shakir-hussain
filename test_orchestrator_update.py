#!/usr/bin/env python
"""Test the updated agent orchestrator with full RAG workflow."""

import json
import sys

try:
    from backend.src.agent.orchestrator import AgentOrchestrator
    from backend.src.agent.types import Message

    print("[TEST] Testing updated orchestrator with full RAG workflow...")
    print("[TEST] Initializing agent...")

    agent = AgentOrchestrator()

    print("[TEST] Agent initialized successfully")
    print("[TEST] Testing query: 'What is ROS2?'")

    try:
        # Test the updated query method
        response = agent.query(
            query_text="What is ROS2?",
            conversation_history=[],
            user_role="student"
        )

        print("\n[RESPONSE]")
        print(f"Answer: {response.answer[:100]}...")
        print(f"Confidence: {response.confidence}")
        print(f"Sources: {len(response.sources)} found")
        print(f"Metadata keys: {list(response.metadata.keys())}")

        # Check if this is the stub or real response
        if "stub" in response.answer.lower() and "progress" in response.answer.lower():
            print("\n[WARNING] Still returning stub response")
            print("[INFO] Backend may need restart to load updated code")
        else:
            print("\n[SUCCESS] Real RAG response generated!")
            print(f"Full answer:\n{response.answer}\n")

    except Exception as e:
        print(f"[ERROR] Query failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

except ImportError as e:
    print(f"[ERROR] Import failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
except Exception as e:
    print(f"[ERROR] Unexpected error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n[TEST] Complete!")
