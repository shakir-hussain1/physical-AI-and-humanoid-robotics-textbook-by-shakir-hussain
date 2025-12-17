# Quick Start Guide - RAG Chatbot

## 30-Second Setup

### Local Development

#### 1. Backend (Terminal 1)
```bash
cd backend
pip install -r requirements.txt
python -m uvicorn src.app:app --reload
```
✓ Backend runs on `http://localhost:8000`

#### 2. Frontend (Terminal 2)
```bash
npm install
npm start
```
✓ Frontend runs on `http://localhost:3000`

#### 3. Test the Chatbot
- Open `http://localhost:3000`
- Click the chat button (💬)
- Try asking: "What is Physical AI?"
- Select text on the page, then ask questions for context-specific answers

### Production Deployment

**Already deployed at:**
```
https://physical-ai-and-humanoid-robotics-textbook-by-sh-production.up.railway.app
```

The frontend automatically detects and uses the production backend.

## Key Features

✅ **RAG-Powered Q&A** - Answers from textbook content
✅ **Text Selection** - Include page text as context
✅ **Source Attribution** - Shows where answers come from
✅ **Confidence Scores** - Reliability indicators
✅ **Conversation History** - Multi-turn chats
✅ **Professional UI** - Modern, responsive design
✅ **Mobile Friendly** - Works on all devices

## API Endpoints

### Ask a Question
```bash
curl -X POST http://localhost:8000/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS?",
    "user_context": null,
    "conversation_history": []
  }'
```

### Health Check
```bash
curl http://localhost:8000/chat/health
```

## Configuration

Create/update `backend/.env`:
```env
OPENAI_API_KEY=your_key_here
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_key
DATABASE_URL=your_neon_db_url
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Backend won't start | `pip install -r requirements.txt` |
| Frontend can't find backend | Make sure backend runs on port 8000 |
| Chatbot not responding | Check API keys in `.env` |
| Port already in use | Change port: `--port 8001` |

## Documentation

- **Full Setup**: See `CHATBOT_SETUP.md`
- **API Docs**: `http://localhost:8000/api/docs`
- **Architecture**: See `CHATBOT_SETUP.md` Architecture section

## Next Steps

1. ✅ Run locally and test
2. ✅ Customize with your content
3. ✅ Deploy to production
4. ✅ Monitor performance

---

**Questions?** Check `CHATBOT_SETUP.md` for detailed documentation.
