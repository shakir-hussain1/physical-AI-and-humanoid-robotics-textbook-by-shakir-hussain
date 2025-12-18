# Running RAG Chatbot Locally - Step by Step Guide

This guide walks through running the complete RAG Chatbot system on your local machine.

## Prerequisites Check

Before starting, verify you have:

```bash
# Check Python version
python --version  # Should be 3.8+

# Check Node.js version
node --version    # Should be 16+
npm --version     # Should be 8+

# Check Git
git --version
```

If any are missing, install them first.

## Step 1: Prepare Your API Credentials

### Get OpenAI API Key

1. Go to https://platform.openai.com/api-keys
2. Sign in with your OpenAI account
3. Click "Create new secret key"
4. Copy the key (starts with `sk-proj-`)
5. Store securely - **NEVER commit this to Git**

### Get Qdrant Credentials

1. Go to https://qdrant.tech/ and sign up
2. Create a new project/cluster
3. Copy:
   - Qdrant URL (e.g., `https://xxx.qdrant.io`)
   - API Key
4. Store securely

## Step 2: Clone Repository

```bash
# Clone the repository
git clone https://github.com/yourusername/Physical-AI-and-Humanoid-Robotics.git

# Navigate to project
cd Physical-AI-and-Humanoid-Robotics

# Check current branch
git branch
# Should show: * 1-rag-chatbot-backend
```

## Step 3: Create Environment File

```bash
# Create .env file in project root
cat > .env << 'EOF'
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-your-actual-key-here
OPENAI_EMBED_MODEL=text-embedding-3-small
OPENAI_CHAT_MODEL=gpt-4o-mini

# Qdrant Vector Database
QDRANT_URL=https://your-qdrant-instance.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=textbook_embeddings

# Backend Configuration
FASTAPI_HOST=0.0.0.0
FASTAPI_PORT=8000
LOG_LEVEL=INFO

# JWT Authentication
JWT_SECRET_KEY=your-super-secret-jwt-key-change-in-production
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_HOURS=24
REFRESH_TOKEN_EXPIRE_DAYS=30
EOF
```

### Edit .env with Your Credentials

```bash
# Open .env in your editor
code .env              # VS Code
# or
nano .env             # nano editor
# or use notepad/any other editor
```

Replace placeholder values with your actual credentials:
- `OPENAI_API_KEY`: Your actual OpenAI key
- `QDRANT_URL`: Your Qdrant instance URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `JWT_SECRET_KEY`: Can be any random string for local testing

## Step 4: Setup Backend

### Create Python Virtual Environment

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv

# Activate it - Choose based on your OS:

# Windows Command Prompt:
venv\Scripts\activate

# Windows PowerShell:
venv\Scripts\Activate.ps1

# macOS/Linux:
source venv/bin/activate
```

After activation, your prompt should show `(venv)` prefix.

### Install Backend Dependencies

```bash
# Make sure you're in backend directory with venv activated
pip install --upgrade pip
pip install -r requirements.txt
```

Expected packages:
- `fastapi==0.109.0` - Web framework
- `uvicorn==0.27.0` - ASGI server
- `openai==1.3.9` - OpenAI client
- `PyJWT==2.8.1` - JWT tokens
- `qdrant-client==2.7.3` - Vector database client
- `python-dotenv` - Environment variables

### Verify Backend Setup

```bash
# Test that imports work
python -c "from src.app import app; print('Backend OK')"

# Should output: Backend OK
```

### Run System Tests

```bash
# Make sure .env is configured first!
python test_system.py
```

Expected output:
```
============================================================
  RAG CHATBOT BACKEND - SYSTEM TEST
============================================================

============================================================
  [1/7] Testing Module Imports
============================================================

[OK] FastAPI app imported
[OK] Auth service imported
[OK] RAG service imported
[OK] Personalization service imported

============================================================
  [2/7] Testing Health Check
============================================================

[OK] Health check passed
     Status: healthy
     Service: RAG Chatbot Backend

============================================================
  [3/7] Testing User Signup
============================================================

[OK] Signup successful
     User ID: 1
     Email: testuser@example.com

... (continues through all 7 tests)
```

### Start Backend Server

```bash
# Make sure venv is activated and you're in backend directory
uvicorn src.app:app --reload --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete
```

Backend is now running at: **http://localhost:8000**

**Keep this terminal open!**

## Step 5: Setup Frontend

### Open New Terminal Window

```bash
# Navigate back to project root
cd Physical-AI-and-Humanoid-Robotics
# (Do NOT navigate to backend folder)
```

### Install Frontend Dependencies

```bash
# Install npm packages
npm install
```

### Start Frontend Development Server

```bash
npm start
```

Expected output:
```
Compiled successfully!

You can now view rag-chatbot in the browser.

  Local:            http://localhost:3000
  On Your Network:  http://192.168.x.x:3000
```

Frontend will automatically open in your browser at: **http://localhost:3000**

**Keep this terminal open!**

## Step 6: Test the Complete System

### 1. Health Check (Backend)

```bash
curl http://localhost:8000/health
```

Response:
```json
{
  "status": "healthy",
  "service": "RAG Chatbot Backend"
}
```

### 2. Create User Account (Frontend)

1. Open http://localhost:3000 in your browser
2. Click "Sign Up"
3. Enter:
   - Email: `testuser@example.com`
   - Username: `testuser123`
   - Password: `SecurePass123` (must have uppercase and number)
   - Confirm Password: `SecurePass123`
4. Click "Sign Up"

### 3. Complete Background Questionnaire

1. Fill out "Who are you?" section:
   - Software Background: Select a level
   - Hardware Background: Select a level
   - Programming Languages: Select at least one
   - Interest Areas: Select at least one
2. Click "Continue"

### 4. Use Chat Widget

1. Click the chat icon (or go to main page)
2. Ask a question: "What is ROS?"
3. See:
   - AI-generated answer
   - Source documents
   - Confidence score

### 5. Test via API (curl)

```bash
# Get tokens
curl -X POST http://localhost:8000/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email":"testuser@example.com","password":"SecurePass123"}'
```

Copy the `access_token` from response, then:

```bash
# Ask a question (replace TOKEN with actual token)
curl -X POST http://localhost:8000/chat/query \
  -H "Authorization: Bearer TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"query":"What is kinematics in robotics?"}'
```

## API Documentation

While backend is running, visit: **http://localhost:8000/api/docs**

This shows interactive Swagger UI with all endpoints:
- Try authentication endpoints
- Test chat queries
- View request/response schemas
- See error handling

## Debugging Tips

### Backend Not Starting

```bash
# Check if port 8000 is already in use
netstat -an | grep 8000  # macOS/Linux
netstat -ano | findstr :8000  # Windows

# Use different port
uvicorn src.app:app --reload --port 8001
```

### Frontend Can't Connect to Backend

1. Verify backend is running:
   ```bash
   curl http://localhost:8000/health
   ```

2. Check browser console for errors (F12)

3. Verify `.env` file has correct API URL:
   ```
   REACT_APP_API_URL=http://localhost:8000
   ```

4. Clear browser cache and reload

### OpenAI API Errors

```
Error: 401 Unauthorized - "Incorrect API key provided"
```

Solutions:
1. Verify API key is correct in `.env`
2. Check API key hasn't been revoked
3. Verify account has credits

### Module Import Errors

```bash
# Verify virtual environment is activated (should show (venv))
which python  # macOS/Linux
where python  # Windows

# Reinstall dependencies
pip install -r requirements.txt --force-reinstall
```

### Database/Vector Search Errors

1. Verify Qdrant URL is accessible:
   ```bash
   curl https://your-qdrant-instance.cloud.qdrant.io/health
   ```

2. Check Qdrant API key is correct

3. Verify network connectivity

## File Structure When Running

```
Project Root
├── .env                          # Your secrets (NOT in git)
├── backend/
│   ├── venv/                     # Virtual environment (created)
│   ├── src/
│   │   ├── app.py               # Running on http://localhost:8000
│   │   ├── services/
│   │   ├── api/
│   │   └── models/
│   ├── requirements.txt
│   └── test_system.py
├── src/                          # React frontend
│   ├── App.jsx
│   └── components/
├── node_modules/                 # npm packages (created)
├── package.json
└── public/

# Two terminals running:
# Terminal 1: cd backend && source venv/bin/activate && uvicorn src.app:app --reload
# Terminal 2: npm start
```

## Next Steps

1. **Explore API**: Visit http://localhost:8000/api/docs
2. **Test More Queries**: Try different questions about Physical AI and Robotics
3. **Modify Content**: Edit sample textbook content in `backend/src/services/rag_service.py`
4. **Deploy**: When ready, see `FINAL_SETUP.md` for Railway deployment

## Performance Notes

- First request: 2-5 seconds (API calls to OpenAI)
- Subsequent requests: 1-2 seconds (cached embeddings)
- Vector search: < 100ms
- LLM generation: 1-3 seconds

## Database

By default, the system uses:
- **In-memory** storage for users (resets on restart)
- **Qdrant cloud** for vector embeddings (persisted)
- Optional PostgreSQL for production (configure in `.env`)

For local testing, in-memory is sufficient.

## Stopping Services

```bash
# Terminal 1 (Backend): Press Ctrl+C
# Terminal 2 (Frontend): Press Ctrl+C
```

## Restarting Services

```bash
# Backend (in backend directory with venv activated):
uvicorn src.app:app --reload --port 8000

# Frontend (in project root):
npm start
```

## Checking Logs

- **Backend logs**: Displayed in terminal running backend
- **Frontend logs**: Browser console (F12 → Console tab)
- **API requests**: http://localhost:8000/api/docs (Swagger shows responses)

---

**You now have a complete working RAG Chatbot running locally!**

For production deployment, see `FINAL_SETUP.md` and deployment guides.

