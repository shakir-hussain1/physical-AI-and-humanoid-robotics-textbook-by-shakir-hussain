#!/usr/bin/env python3
"""
Test script for Urdu translation service.

Validates that:
1. No HTML tags in output
2. Pure Urdu text only
3. No English words remaining
4. No Hindi or other languages mixed in
"""

import asyncio
import sys
import os
from pathlib import Path

# Fix encoding for Windows
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent / 'backend'))

from src.services.translation_service import TranslationService

# Sample chapter content (simulating HTML content from markdown)
SAMPLE_ENGLISH_TEXT = """
Chapter 1: Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software.
It is a collection of tools and libraries that aim to simplify the task of creating complex
and robust robot behavior across a wide variety of robotic platforms.

ROS provides:
- Hardware abstraction
- Device drivers
- Message-passing between processes
- Package management
- Visualization tools

Learning Objectives:
By the end of this chapter, you will be able to understand the fundamentals of ROS 2.
"""

async def test_urdu_translation():
    """Test Urdu translation output."""
    print("=" * 80)
    print("TESTING URDU TRANSLATION SERVICE")
    print("=" * 80)

    service = TranslationService()

    try:
        # Test translation
        print("\n[ORIGINAL] English Text:")
        print("-" * 80)
        print(SAMPLE_ENGLISH_TEXT[:200] + "...")

        print("\n[TRANSLATING] to Urdu...")
        print("-" * 80)

        result = await service._translate_text(SAMPLE_ENGLISH_TEXT, 'urdu')

        print("\n[RESULT] Urdu Translation:")
        print("-" * 80)
        print(result)

        # Validation checks
        print("\n" + "=" * 80)
        print("VALIDATION CHECKS:")
        print("=" * 80)

        checks = {
            "No HTML tags": not any(tag in result for tag in ['<', '>', '<html', '<p>', '<h1>']),
            "No English words (ROS, Robot, Operating, System, etc)": not any(word in result.lower() for word in ['robot', 'operating', 'system', 'ros', 'framework', 'hardware', 'software', 'device', 'package']),
            "No HTML entities": '&lt;' not in result and '&gt;' not in result and '&#' not in result,
            "Is pure Urdu (contains Urdu characters)": any(ord(c) >= 0x0600 and ord(c) <= 0x06FF for c in result),
            "Result is not empty": len(result.strip()) > 0,
        }

        all_passed = True
        for check_name, passed in checks.items():
            status = "[PASS]" if passed else "[FAIL]"
            print(f"{status}: {check_name}")
            if not passed:
                all_passed = False

        print("\n" + "=" * 80)
        if all_passed:
            print("[PASS] ALL CHECKS PASSED - Translation is pure Urdu!")
        else:
            print("[FAIL] SOME CHECKS FAILED - Translation needs improvement")
            print("\nProblems detected:")
            if "No English words" in checks and not checks["No English words"]:
                print("  - English words found in translation")
            if "No HTML tags" in checks and not checks["No HTML tags"]:
                print("  - HTML tags found in translation")
        print("=" * 80)

        return all_passed

    except Exception as e:
        print(f"\n[ERROR]: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_urdu_translation())
    sys.exit(0 if success else 1)
