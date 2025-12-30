---
id: "002"
title: "Python virtual environment setup and configuration"
date: "2025-12-28"
stage: "green"
surface: "agent"
model: "claude-haiku-4-5-20251001"
feature: "rag-chatbot"
branch: "feat/chatbot-ui-and-fastapi-integration"
user: "User"
command: "local-setup-session"
labels: ["venv", "python-environment", "backend-setup", "dependencies", "deployment"]
links:
  spec: "specs/rag-chatbot/spec.md"
  plan: "specs/rag-chatbot/plan.md"
  tasks: "specs/rag-chatbot/tasks.md"
  adr: null
files:
  - venv/
  - backend/requirements.txt
  - START_BACKEND.bat
tests:
  - Create virtual environment (venv/)
  - Install all dependencies from requirements.txt
  - Activate venv and verify Python version
  - Start backend server with uvicorn on port 8000
  - Verify API endpoints respond correctly
  - Test health endpoint returns component status
---

## Context

After fixing the API key configuration issue, the backend still had port conflicts and environment variable loading issues. User requested: "ye tum bana krdo or batao main kese step kron" (Create this and tell me how to run it step by step).

**Goal:** Create a reproducible, simple startup script that handles:
1. Virtual environment activation
2. Environment variable configuration
3. Backend server startup with proper logging
4. Clear instructions for end users

## Prompt

User requested a batch script (Windows-specific) that:
- Activates virtual environment automatically
- Sets environment variables for Qdrant, Cohere, OpenAI
- Starts FastAPI backend on port 8000
- Provides clear output messages

## Response

### Phase 1: Virtual Environment Creation

**Command:**
```bash
python -m venv venv
```

**Process:**
- Creates isolated Python environment in `venv/` directory
- Includes local copy of Python interpreter (3.9+)
- Copies pip, setuptools, wheel
- Prevents conflicts with system Python packages

**Size:** ~150MB

### Phase 2: Dependency Installation

**Dependencies Installed:**

| Package | Version | Purpose |
|---------|---------|---------|
| fastapi | 0.104+ | Web framework |
| uvicorn | 0.24+ | ASGI server |
| pydantic | 2.0+ | Data validation |
| python-dotenv | 1.0+ | Environment configuration |
| openai | 1.3+ | GPT API client |
| cohere | 5.0+ | Embedding service |
| qdrant-client | 2.7+ | Vector database client |
| httpx | 0.25+ | HTTP client (async) |

**Total Installation Time:** ~2-3 minutes

### Phase 3: Startup Script Creation

**File:** `START_BACKEND.bat`

Creates Windows batch script that:
- Navigates to project root
- Activates virtual environment
- Sets all 6 environment variables
- Starts uvicorn server on port 8000
- Displays clear status messages

**Key Features:**
- Uses `cd /d` for cross-drive navigation (Windows compatibility)
- `call` to activate venv in same process
- Exports all 6 environment variables explicitly
- `--reload` flag for development auto-restart
- Clear status messages at each step
- Pause at end to keep window open if error occurs

### Phase 4: Usage Instructions

**Step 1: Double-click `START_BACKEND.bat` from project root**

**Step 2: Test the backend**

```bash
curl http://localhost:8000/api/health
```

**Step 3: Verify with sample query**

```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS2?","user_role":"student"}'
```

### Phase 5: Port Configuration (Troubleshooting)

If port 8000 is already in use:

**Option A: Kill existing process**
```bash
netstat -ano | findstr :8000
taskkill /PID [PID] /F
```

**Option B: Use different port**
```batch
python -m uvicorn src.api.main:app --host 0.0.0.0 --port 9000
```

### Phase 6: Verification Tests

- Virtual environment active and Python version correct
- All dependencies installed via pip list
- Backend server running on port 8000
- Health endpoint responds with component status
- Sample queries return real answers with sources

## Outcome

✅ **Environment fully configured and reproducible**
- Virtual environment created with all dependencies
- Startup script automates activation and configuration
- No manual environment variable setting needed for end users
- Backend can start with single double-click on Windows
- All external services configured and ready

**Time to Production:** 5-10 minutes

## Success Criteria - Met ✅

- [x] Virtual environment created with Python 3.9+
- [x] All backend dependencies installed and verified
- [x] Startup script handles venv activation automatically
- [x] All 6 environment variables properly configured
- [x] Backend server starts on port 8000 with --reload
- [x] Health endpoint responds correctly
- [x] Sample queries return real answers (not stubs)
- [x] User can start backend with single script click
- [x] Clear status messages guide user through startup
- [x] Troubleshooting documentation provided

---

**Status:** ✅ COMPLETE - Production-ready local environment setup
