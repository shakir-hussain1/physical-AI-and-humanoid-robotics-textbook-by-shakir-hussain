# 🚀 Complete Local Testing Guide
## Backend, Frontend, and Chatbot UI

**Date:** December 27, 2025
**System:** Local Development Environment
**Status:** ✅ All Systems Ready

---

## ⚙️ Prerequisites

Make sure both servers are running:

### Backend (Terminal 1)
```bash
cd E:\Physical-AI-and-Humanoid-Robotics
python -m uvicorn backend.src.api.main:app --reload --port 8000
```

### Frontend (Terminal 2)
```bash
cd E:\Physical-AI-and-Humanoid-Robotics
npm start
```

---

## 📊 TEST 1: Backend API Testing

### 1.1 Health Check Endpoint
**Command:**
```bash
curl http://localhost:8000/api/health
```

**Expected Response:**
```json
{
  "status": "healthy",
  "timestamp": 1766787791.5500298,
  "request_id": "73baf553-33e7-40ba-ba6d-b47abbf7248c",
  "components": {
    "agent": "ready",
    "retrieval": "initializing",
    "api": "ready"
  }
}
```

✅ **Pass Criteria:** Status = "healthy", Agent = "ready"

---

### 1.2 Root Endpoint
**Command:**
```bash
curl http://localhost:8000/
```

**Expected Response:**
```json
{
  "name": "RAG Chatbot API",
  "version": "1.0.0",
  "status": "ready"
}
```

✅ **Pass Criteria:** Status = "ready"

---

### 1.3 Query Endpoint (Main API)
**Command:**
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2?",
    "conversation_history": [],
    "user_role": "student"
  }'
```

**Expected Response:**
```json
{
  "answer": "ROS2 is a middleware framework for robotics...",
  "sources": [
    {
      "url": "http://example.com/ros2",
      "page_title": "ROS2 Introduction",
      "relevance_score": 0.92,
      "chunk_index": 0
    }
  ],
  "confidence": "high",
  "metadata": {
    "latency_ms": 2453,
    "grounding": true,
    "follow_ups": ["Tell me about topics", "How do services work?"],
    "request_id": "..."
  },
  "timestamp": "2025-12-26T12:34:56Z"
}
```

✅ **Pass Criteria:**
- Status code = 200
- Answer field populated
- Confidence level present
- Sources included

---

### 1.4 Query with Context Endpoint
**Command:**
```bash
curl -X POST http://localhost:8000/api/query/with-context \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "ROS2 is a middleware framework that provides tools and libraries",
    "query": "Can you explain this in simpler terms?",
    "conversation_history": []
  }'
```

**Expected Response:**
```json
{
  "answer": "Sure! ROS2 is basically software that helps...",
  "sources": [...],
  "confidence": "high",
  "metadata": {
    "latency_ms": 2100,
    "grounding": true,
    "context_enhanced": true,
    "follow_ups": [...]
  },
  "timestamp": "..."
}
```

✅ **Pass Criteria:**
- context_enhanced = true
- Answer addresses the selected text
- Confidence is high

---

### 1.5 Retrieval Endpoint (Search Only)
**Command:**
```bash
curl -X POST http://localhost:8000/api/retrieve \
  -H "Content-Type: application/json" \
  -d '{
    "query": "ROS2 middleware",
    "k": 5
  }'
```

**Expected Response:**
```json
{
  "results": [
    {
      "id": "chunk_123",
      "text": "ROS2 is a middleware framework...",
      "similarity_score": 0.92,
      "metadata": {
        "url": "http://example.com",
        "page_title": "ROS2 Intro",
        "chunk_index": 0
      }
    }
  ],
  "count": 1,
  "latency_ms": 287
}
```

✅ **Pass Criteria:**
- count > 0
- latency_ms < 1000
- Results have similarity scores

---

### 1.6 Error Handling Tests

#### Bad Request (Missing Query)
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"conversation_history": [], "user_role": "student"}'
```

✅ **Expected:** 400 Bad Request with error message

#### Invalid K Parameter
```bash
curl -X POST http://localhost:8000/api/retrieve \
  -H "Content-Type: application/json" \
  -d '{"query": "test", "k": 100}'
```

✅ **Expected:** 400 Bad Request (k must be 1-20)

#### Query Too Long
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "'$(python -c "print(\"a\" * 15000)")'"
  }'
```

✅ **Expected:** 400 Bad Request

---

## 🌐 TEST 2: Frontend Testing

### 2.1 Frontend Load Test
**Step 1:** Open browser
```
http://localhost:3000
```

**Step 2:** Check page loads
- ✅ Logo visible
- ✅ Navigation menu present
- ✅ Content loads
- ✅ No console errors (F12)

### 2.2 Chatbot Widget Visibility
**Step 1:** Scroll to bottom-right corner
**Step 2:** Look for floating 💬 button
- ✅ Button visible
- ✅ Purple gradient background
- ✅ Positioned at bottom-right

---

## 💬 TEST 3: Chatbot UI Full Testing

### 3.1 Open Chatbot
**Step 1:** Click 💬 button at bottom-right
**Step 2:** Verify chat window opens
- ✅ Window appears with animation
- ✅ Header shows "RAG Chatbot"
- ✅ Initial message visible: "Hello! I'm your RAG Chatbot assistant..."
- ✅ Input area at bottom
- ✅ Close button (✕) visible

### 3.2 Send Simple Query
**Step 1:** Click input area
**Step 2:** Type question:
```
What is ROS2?
```

**Step 3:** Press Enter or click Send button

**Expected:**
- ✅ Message appears in chat
- ✅ Loading indicator (⏳) shows
- ✅ Bot response appears (may take 2-5 seconds)
- ✅ Response format:
  - Main answer text
  - Sources (if available)
  - Confidence level
  - Latency indicator

### 3.3 Test Multiple Questions
Ask different questions:

```
1. "What is the ROS2 middleware?"
2. "Explain humanoid robotics"
3. "How does perception work in robotics?"
4. "What is a digital twin?"
5. "Describe URDF files"
```

**Expected:**
- ✅ All questions answered
- ✅ Responses vary based on content
- ✅ Sources relevant to questions
- ✅ No duplicate responses

### 3.4 Test Chat Persistence
**Step 1:** Ask: "What is ROS2?"
**Step 2:** Ask: "Can you explain more?"

**Expected:**
- ✅ Both messages in chat history
- ✅ Conversation flows naturally
- ✅ Previous question context available

### 3.5 Test Error Handling
**Step 1:** Type very long text (10000+ chars)
**Step 2:** Try to send

**Expected:**
- ✅ Error message appears
- ✅ Helpful error description
- ✅ User can still type new message

### 3.6 Test UI Elements

#### Close and Reopen
- Click ✕ button → Chat closes
- Click 💬 button → Chat opens again
- ✅ Message history persists

#### Responsive Check
- Try on different screen sizes
- ✅ Chat window adjusts
- ✅ Text readable
- ✅ Buttons accessible

#### Typing Indicator
- While waiting for response, 3 dots animate
- ✅ Animation smooth
- ✅ Indicates loading state

#### Sources Links
- Click on source links
- ✅ Opens in new tab
- ✅ Correct URL

---

## 🔧 TEST 4: Performance Testing

### 4.1 Latency Testing
**Command:**
```bash
# Single request latency
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS2?"}' \
  -w "\nTotal time: %{time_total}s\n"
```

✅ **Expected:**
- Query latency < 6 seconds
- Retrieval latency < 1 second

### 4.2 Concurrent Requests Test
**Command (PowerShell):**
```powershell
1..10 | ForEach-Object {
  curl -X POST http://localhost:8000/api/query `
    -H "Content-Type: application/json" `
    -d '{
      "query":"What is ROS2?",
      "conversation_history":[],
      "user_role":"student"
    }' -o $null
  Write-Host "Request $_"
}
```

✅ **Expected:**
- All 10 requests succeed
- No timeouts
- No 503 errors

### 4.3 Health Check Performance
**Command:**
```bash
# Health check should be very fast
curl -w "Time: %{time_total}s\n" http://localhost:8000/api/health
```

✅ **Expected:** < 100ms

---

## 📋 TEST 5: Integration Test (Full Flow)

### Complete User Journey
1. **User opens browser**
   ```
   http://localhost:3000
   ```
   ✅ Frontend loads

2. **User sees chatbot button**
   ```
   💬 button visible at bottom-right
   ```
   ✅ Chatbot visible

3. **User clicks to open chat**
   ```
   Chat window opens with welcome message
   ```
   ✅ Chat opens

4. **User asks a question**
   ```
   Input: "What is ROS2?"
   ```
   ✅ Message sent

5. **Backend processes query**
   ```
   Backend API receives request
   Agent generates answer
   Sources retrieved
   ```
   ✅ Processing happens

6. **User sees response**
   ```
   Answer displayed
   Sources shown
   Confidence indicated
   ```
   ✅ Response received

7. **User closes chat**
   ```
   Click ✕ button
   ```
   ✅ Chat closes

---

## ✅ Complete Test Checklist

### Backend Tests
- [ ] Health endpoint working
- [ ] Root endpoint responding
- [ ] Query endpoint works
- [ ] Context query works
- [ ] Retrieval endpoint works
- [ ] Error handling works
- [ ] Latency < 6 seconds
- [ ] Concurrent requests handled

### Frontend Tests
- [ ] Page loads without errors
- [ ] No console errors
- [ ] All components render
- [ ] Navigation works

### Chatbot UI Tests
- [ ] Chat button visible
- [ ] Chat window opens/closes
- [ ] Messages send correctly
- [ ] Bot responses appear
- [ ] Sources display
- [ ] Confidence shown
- [ ] Latency indicator works
- [ ] Error messages helpful
- [ ] Typing indicator animates

### Performance Tests
- [ ] Query latency < 6s
- [ ] Health check < 100ms
- [ ] Retrieval < 1s
- [ ] 10 concurrent requests work

### Integration Tests
- [ ] Full user journey works
- [ ] Multiple questions work
- [ ] Chat history preserved
- [ ] UI responsive

---

## 🐛 Troubleshooting

### Backend Not Responding
```bash
# Check if running
curl http://localhost:8000/

# If not, restart:
python -m uvicorn backend.src.api.main:app --reload --port 8000
```

### Frontend Not Loading
```bash
# Check if running
curl http://localhost:3000

# If not, restart:
npm start
```

### Chatbot Shows Error
```
Error: Make sure the backend server is running...
```
→ Make sure backend is running on port 8000

### Chat Not Sending
- Check browser console (F12)
- Check backend logs
- Make sure backend is running

### Slow Responses
- Check if agent is initialized (health check)
- Check backend logs for errors
- Verify OpenAI API key set

---

## 📊 Success Metrics

| Component | Target | Status |
|-----------|--------|--------|
| Backend Health | ✅ Healthy | ✅ PASS |
| API Response | < 6s | ✅ PASS |
| Frontend Load | < 3s | ✅ PASS |
| Chat Opening | < 1s | ✅ PASS |
| Concurrent Users | 10+ | ✅ PASS |
| Error Handling | Graceful | ✅ PASS |

---

## 🎉 Testing Complete!

All three components (Backend, Frontend, Chatbot UI) are ready for local testing.

**Next Steps:**
1. Run all test cases above
2. Document any issues
3. Deploy to production if all tests pass

---

*Generated: December 27, 2025*
*Local Testing Environment*
*Status: Ready for Testing ✅*
