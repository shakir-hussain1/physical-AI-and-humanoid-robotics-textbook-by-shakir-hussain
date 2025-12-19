# RAG Chatbot - Quick Fix Guide (Local Testing)

## The Problem

You're seeing: "I am currently unable to connect to the backend service."

This means the frontend can't reach the backend API.

## The Solution: 3 Steps

### Step 1: Start Backend (Terminal 1)

```bash
cd backend
python -m venv venv

# Activate virtual environment:
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Start backend on port 8000
uvicorn src.app:app --reload

# You should see:
# INFO:     Uvicorn running on http://0.0.0.0:8000
# INFO:     Application startup complete
```

**KEEP THIS TERMINAL OPEN!**

### Step 2: Verify Backend is Working

Open a new terminal and test:

```bash
# Test health check
curl http://localhost:8000/health

# Should return: {"status":"healthy","service":"chatbot-backend"}
```

If this works, backend is fine.

### Step 3: Start Frontend (Terminal 2)

```bash
# Go to project root (NOT backend folder)
cd Physical-AI-and-Humanoid-Robotics

# Start Docusaurus
npm start

# Should open browser at: http://localhost:3000
```

**KEEP THIS TERMINAL OPEN!**

## Testing the Chatbot

1. Open http://localhost:3000 in browser
2. Click the chat bubble (💬) in bottom right
3. Type a question: "What is ROS?"
4. Click "Send"

### Expected Result

- ✅ You should see an answer about ROS
- ✅ Sources will be shown
- ✅ Confidence score will appear

### If Still Not Working

#### Issue: "Backend is not reachable"

**Check 1: Is backend running?**
```bash
# Terminal 1 should show something like:
# Uvicorn running on http://0.0.0.0:8000
```

**Check 2: Is port 8000 free?**
```bash
# Check if something is already using port 8000
# Windows:
netstat -ano | findstr :8000

# macOS/Linux:
lsof -i :8000

# If blocked, kill it or use different port:
uvicorn src.app:app --reload --port 8001
```

**Check 3: Test backend directly**
```bash
curl -X POST "http://localhost:8000/chat/query" \
  -H "Content-Type: application/json" \
  -d "{\"query\":\"What is ROS?\"}"

# Should return a JSON response with answer
```

#### Issue: "Cannot find module"

```bash
cd backend
pip install -r requirements.txt --force-reinstall
```

#### Issue: CORS Error

Check browser console (F12 → Console). If you see CORS error:
- Backend CORS is configured correctly
- Reload page and try again
- Clear browser cache

#### Issue: Docusaurus won't start

```bash
# Install dependencies
npm install

# Clear cache
npm run clear

# Try again
npm start
```

## What Was Fixed

The updated `ChatWidget.js` now:
- ✅ Detects backend URL automatically
- ✅ Checks if backend is healthy
- ✅ Sends correct field names (`user_context` not `selected_text`)
- ✅ Shows detailed error messages with solutions
- ✅ Logs to browser console for debugging (F12)

## Production Deployment

For GitHub Pages or Railway, update the API URL in `ChatWidget.js`:

```javascript
// Line 17 - Change from:
return 'http://localhost:8000'; // Local only

// To your production backend:
return 'https://your-backend-url.railway.app';
```

## File Changes

- **Modified**: `src/components/ChatWidget.js`
  - Added backend health check
  - Fixed API field names
  - Added detailed error messages
  - Added console logging

## Next Steps

Once local testing works:

1. **Git commit**
```bash
git add src/components/ChatWidget.js
git commit -m "fix: Fix ChatWidget API connection and error handling"
git push origin 1-rag-chatbot-backend
```

2. **Deploy to Railway** (see FINAL_SETUP.md)

3. **Test on GitHub Pages** (need to deploy backend first)

## Debug Tips

Open browser console (F12 → Console) to see:
- Backend health check results
- API request/response logs
- Detailed error messages

Look for logs starting with `[ChatWidget]`

## Still Stuck?

1. Check all 3 terminals are running
2. Backend terminal should show API requests being made
3. Docusaurus terminal should show no errors
4. Check `http://localhost:8000/health` works via curl
5. Clear browser cache (Ctrl+Shift+Delete)
6. Reload page (Ctrl+R)

---

**Backend**: http://localhost:8000/api/docs (interactive API docs)
**Frontend**: http://localhost:3000

Both should be accessible while terminals are running.
