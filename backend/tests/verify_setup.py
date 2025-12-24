#!/usr/bin/env python3
"""
Verification script to test that the RAG Chatbot is properly set up locally.
Run this to diagnose connection issues.
"""

import subprocess
import sys
import json
import time
from pathlib import Path

def print_header(text):
    print(f"\n{'='*60}")
    print(f"  {text}")
    print(f"{'='*60}\n")

def run_command(cmd, description):
    """Run a command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        print(f"[OK] {description}")
        return True, result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        print(f"[TIMEOUT] {description} - took too long")
        return False, ""
    except Exception as e:
        print(f"[ERROR] {description}: {e}")
        return False, ""

def check_port_in_use(port):
    """Check if a port is in use"""
    try:
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result = sock.connect_ex(('127.0.0.1', port))
        sock.close()
        return result == 0
    except:
        return False

def main():
    print_header("RAG CHATBOT SETUP VERIFICATION")
    print("Checking your local environment...\n")

    all_passed = True

    # Check 1: Python version
    print_header("[1/8] Python Environment")
    py_version = sys.version_info
    if py_version.major >= 3 and py_version.minor >= 8:
        print(f"[OK] Python {py_version.major}.{py_version.minor}.{py_version.micro}")
    else:
        print(f"[ERROR] Python 3.8+ required (found {py_version.major}.{py_version.minor})")
        all_passed = False

    # Check 2: Node.js version
    print_header("[2/8] Node.js Environment")
    success, output = run_command("node --version", "Node.js version")
    if success:
        print(output.strip())
    else:
        print("[WARNING] Node.js not found (needed for Docusaurus)")
        all_passed = False

    # Check 3: Backend directory
    print_header("[3/8] Backend Directory Structure")
    backend_path = Path("backend")
    if backend_path.exists():
        print("[OK] Backend directory exists")
        venv_path = backend_path / "venv"
        if venv_path.exists():
            print("[OK] Virtual environment exists (venv/)")
        else:
            print("[WARNING] Virtual environment not found - run: cd backend && python -m venv venv")

        src_path = backend_path / "src"
        if src_path.exists():
            print("[OK] src/ directory found")
        else:
            print("[ERROR] src/ directory not found")
            all_passed = False
    else:
        print("[ERROR] Backend directory not found")
        all_passed = False

    # Check 4: Check if backend is running
    print_header("[4/8] Backend Service Status")
    if check_port_in_use(8000):
        print("[OK] Port 8000 is in use (backend likely running)")

        # Try to connect to health endpoint
        try:
            import urllib.request
            import json as json_module
            response = urllib.request.urlopen("http://localhost:8000/health", timeout=3)
            data = json_module.loads(response.read().decode())
            print(f"[OK] Backend health check passed")
            print(f"     Status: {data.get('status')}")
            print(f"     Service: {data.get('service')}")
        except Exception as e:
            print(f"[ERROR] Could not reach backend health endpoint: {e}")
            all_passed = False
    else:
        print("[WARNING] Port 8000 is not in use")
        print("         Backend does not appear to be running")
        print("         Run: cd backend && uvicorn src.app:app --reload")

    # Check 5: Check if frontend is running
    print_header("[5/8] Frontend Service Status")
    if check_port_in_use(3000):
        print("[OK] Port 3000 is in use (frontend likely running)")
        try:
            import urllib.request
            response = urllib.request.urlopen("http://localhost:3000", timeout=3)
            print("[OK] Frontend is responding")
        except Exception as e:
            print(f"[WARNING] Could not reach frontend: {e}")
    else:
        print("[INFO] Port 3000 is not in use (frontend not running)")
        print("       Run: npm start")

    # Check 6: ChatWidget source file
    print_header("[6/8] Frontend ChatWidget Component")
    chatwidget_path = Path("src/components/ChatWidget.js")
    if chatwidget_path.exists():
        print("[OK] ChatWidget.js found")
        try:
            with open(chatwidget_path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
                if "getApiUrl" in content:
                    print("[OK] ChatWidget has API URL detection")
                if "checkBackendHealth" in content:
                    print("[OK] ChatWidget has health check")
                if "user_context" in content:
                    print("[OK] ChatWidget uses correct field names")
        except Exception as e:
            print(f"[WARNING] Could not read ChatWidget: {e}")
    else:
        print("[ERROR] ChatWidget.js not found")
        all_passed = False

    # Check 7: Test API endpoint
    print_header("[7/8] API Endpoint Test")
    if check_port_in_use(8000):
        try:
            import urllib.request
            import json as json_module

            req_data = json_module.dumps({
                "query": "What is ROS?",
                "user_context": None,
                "conversation_history": []
            }).encode('utf-8')

            req = urllib.request.Request(
                "http://localhost:8000/chat/query",
                data=req_data,
                headers={"Content-Type": "application/json"},
                method="POST"
            )

            response = urllib.request.urlopen(req, timeout=5)
            data = json_module.loads(response.read().decode())

            print("[OK] Chat endpoint responded")
            print(f"     Status: {data.get('status')}")
            print(f"     Answer length: {len(data.get('answer', ''))} chars")
            print(f"     Confidence: {data.get('confidence', 'N/A')}")
            print(f"     Sources: {len(data.get('sources', []))}")
        except Exception as e:
            print(f"[ERROR] Chat endpoint failed: {e}")
            all_passed = False
    else:
        print("[SKIP] Backend not running - cannot test endpoint")

    # Check 8: Setup files
    print_header("[8/8] Documentation & Setup Files")
    files_to_check = [
        "CHATBOT_QUICKFIX.md",
        "FINAL_SETUP.md",
        "RUN_LOCAL.md",
        "backend/test_system.py"
    ]

    for file in files_to_check:
        path = Path(file)
        if path.exists():
            print(f"[OK] {file}")
        else:
            print(f"[WARNING] {file} not found")

    # Summary
    print_header("SUMMARY & NEXT STEPS")

    if all_passed:
        print("[SUCCESS] All checks passed!")
        print("\nYour RAG Chatbot should be working.")
        print("\nNext steps:")
        print("1. Open http://localhost:3000 in your browser")
        print("2. Click the chat bubble in the bottom right")
        print("3. Ask a question like: 'What is ROS?'")
        print("4. You should see an answer with sources")
    else:
        print("[WARNING] Some checks failed or warnings found.")
        print("\nTroubleshooting:")
        print("1. Make sure backend is running:")
        print("   cd backend && uvicorn src.app:app --reload")
        print("2. Make sure frontend is running (in different terminal):")
        print("   npm start")
        print("3. Check that both services are accessible:")
        print("   - Backend: http://localhost:8000/health")
        print("   - Frontend: http://localhost:3000")
        print("\nFor detailed setup, see:")
        print("   - CHATBOT_QUICKFIX.md (fastest fix)")
        print("   - RUN_LOCAL.md (detailed guide)")
        print("   - FINAL_SETUP.md (complete reference)")

    print("\n" + "="*60)

if __name__ == "__main__":
    main()
