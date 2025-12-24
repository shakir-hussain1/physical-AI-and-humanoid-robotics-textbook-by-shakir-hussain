#!/usr/bin/env python3
"""
Test translation service with real chapter content.
Validates that translations work for multiple chapters in Urdu.
"""

import asyncio
import sys
import os
from pathlib import Path

# Fix encoding for Windows
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

sys.path.insert(0, str(Path(__file__).parent))

from src.services.translation_service import TranslationService

# Sample chapter content from different modules
CHAPTERS = {
    'Chapter-1-ROS': """Chapter 1: Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software.
It is a collection of tools and libraries that aim to simplify the task of creating complex
and robust robot behavior across a wide variety of robotic platforms.

ROS provides:
- Hardware abstraction
- Device drivers
- Message-passing between processes
- Package management
- Visualization tools
""",

    'Chapter-3-URDF': """Chapter 3: Robot Description Format

URDF (Unified Robot Description Format) is an XML format for specifying the structure
of robots. It contains descriptions of all the links and joints of your robot.

Key components:
- Links: rigid bodies with mass and inertia
- Joints: connections between links with motion constraints
- Plugins: extensions for gazebo simulation
- Collision and visual geometry definitions
""",

    'Chapter-5-Perception': """Chapter 5: Robot Perception

Robot perception involves using sensors to understand the environment.
Common sensors include cameras, LiDAR, stereo vision, and depth sensors.

Perception pipeline:
- Sensor data acquisition
- Preprocessing and filtering
- Feature extraction
- Object detection and recognition
- Scene understanding
"""
}

async def test_chapter_translation():
    """Test translation of multiple chapters."""
    print("=" * 80)
    print("CHAPTER TRANSLATION TEST - URDU")
    print("=" * 80)

    service = TranslationService()
    results = []

    for chapter_name, chapter_content in CHAPTERS.items():
        print(f"\n[CHAPTER] Testing: {chapter_name}")
        print("-" * 80)

        try:
            translated = await service._translate_text(chapter_content, 'urdu')

            # Validation
            is_clean = '<' not in translated and '>' not in translated
            has_urdu = any(ord(c) >= 0x0600 and ord(c) <= 0x06FF for c in translated)
            is_not_empty = len(translated.strip()) > 0

            status = "[PASS]" if (is_clean and has_urdu and is_not_empty) else "[FAIL]"

            print(f"{status} Translation successful")
            print(f"  - Clean output: {is_clean}")
            print(f"  - Contains Urdu: {has_urdu}")
            print(f"  - Not empty: {is_not_empty}")
            print(f"  - Length: {len(translated)} chars")

            print(f"\n[SAMPLE] First 200 chars of translation:")
            print(translated[:200])

            results.append({
                'chapter': chapter_name,
                'success': is_clean and has_urdu and is_not_empty,
                'length': len(translated)
            })

        except Exception as e:
            print(f"[ERROR] Translation failed: {e}")
            results.append({
                'chapter': chapter_name,
                'success': False,
                'error': str(e)
            })

    # Summary
    print("\n" + "=" * 80)
    print("TRANSLATION TEST SUMMARY")
    print("=" * 80)

    passed = sum(1 for r in results if r['success'])
    total = len(results)

    for result in results:
        status = "[PASS]" if result['success'] else "[FAIL]"
        print(f"{status}: {result['chapter']}")

    print(f"\nTotal: {passed}/{total} chapters translated successfully")

    if passed == total:
        print("[SUCCESS] All chapters translated perfectly!")
        return True
    else:
        print(f"[WARNING] {total - passed} chapter(s) failed")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_chapter_translation())
    sys.exit(0 if success else 1)
