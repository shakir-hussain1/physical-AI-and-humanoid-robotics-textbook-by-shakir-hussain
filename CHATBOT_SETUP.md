# Physical AI & Humanoid Robotics RAG Chatbot Setup

## Overview

This project includes a fully integrated **Retrieval-Augmented Generation (RAG) Chatbot** for the Physical AI & Humanoid Robotics textbook. The chatbot uses:

- **Backend**: FastAPI with Python
- **LLM**: OpenAI GPT-4o-mini
- **Embeddings**: OpenAI text-embedding-3-small
- **Vector Database**: Qdrant Cloud
- **Relational Database**: Neon PostgreSQL
- **Frontend**: React with professional UI
- **Deployment**: Railway

## Quick Start

### Prerequisites

- Python 3.8+
- Node.js 16+
- Git
- API Keys:
  - OpenAI API Key
  - Qdrant Cloud API Key
  - Neon PostgreSQL connection string

### 1. Backend Setup

```bash
cd backend

# Install dependencies
pip install -r requirements.txt

# Configure environment
# Update .env with your API keys

# Run locally
python -m uvicorn src.app:app --reload --port 8000
```

Backend will be available at: `http://localhost:8000`

API Documentation: `http://localhost:8000/api/docs`

### 2. Frontend Setup

```bash
# Install dependencies
npm install

# Start development server
npm start
```

Frontend will be available at: `http://localhost:3000`

The ChatWidget component will automatically connect to the backend.

## Architecture

### Backend Architecture

```
backend/
├── src/
│   ├── app.py                 # FastAPI application
│   ├── api/
│   │   ├── chat.py            # Chat endpoints
│   │   └── __init__.py
│   ├── services/
│   │   ├── rag_service.py     # RAG pipeline
│   │   └── __init__.py
│   ├── config.py              # Configuration
│   └── __init__.py
├── .env                        # Environment variables
├── requirements.txt            # Python dependencies
└── Dockerfile                  # Container configuration
```

### Frontend Architecture

```
src/
├── components/
│   ├── ChatWidget.jsx          # Main chat component
│   └── ChatWidget.module.css   # Styling
└── ...other components
```

## API Endpoints

### Chat Endpoints

#### POST `/chat/query`

Process a user query with RAG pipeline.

**Request:**
```json
{
  "query": "What is ROS?",
  "user_context": "Optional selected text",
  "conversation_history": [
    {
      "role": "user",
      "content": "Previous message"
    }
  ]
}
```

**Response:**
```json
{
  "answer": "ROS (Robot Operating System) is...",
  "sources": [
    {
      "title": "Robot Operating System",
      "source": "Chapter 2",
      "category": "robotics",
      "score": 0.95
    }
  ],
  "confidence": 0.92,
  "confidence_level": "high",
  "status": "success"
}
```

#### GET `/chat/health`

Health check endpoint.

**Response:**
```json
{
  "status": "healthy",
  "service": "RAG Chatbot Backend"
}
```

#### POST `/chat/context`

Save user context (selected text for enhanced RAG).

**Request:**
```json
{
  "content": "Selected text content",
  "metadata": {...}
}
```

## Environment Variables

```env
# OpenAI API
OPENAI_API_KEY=sk-...
OPENAI_EMBED_MODEL=text-embedding-3-small
OPENAI_CHAT_MODEL=gpt-4o-mini

# Qdrant Vector Database
QDRANT_URL=https://...
QDRANT_API_KEY=...
QDRANT_COLLECTION=textbook_embeddings

# Neon PostgreSQL
DATABASE_URL=postgresql://...

# FastAPI
FASTAPI_HOST=0.0.0.0
FASTAPI_PORT=8000
FASTAPI_ENV=development

# Logging
LOG_LEVEL=INFO

# RAG Configuration
RETRIEVAL_TOP_K=5
CONFIDENCE_THRESHOLD=0.60
RAG_TEMPERATURE=0.7
```

## Deployment

### Local Testing

1. Start the backend:
```bash
cd backend
python -m uvicorn src.app:app --reload
```

2. Start the frontend:
```bash
npm start
```

3. Open `http://localhost:3000` and test the chatbot

### Deploy to Railway

1. Push code to GitHub
2. Connect GitHub repository to Railway
3. Set environment variables in Railway dashboard
4. Railway will automatically detect and deploy

Environment variables in Railway:
- All variables from `.env` should be configured in Railway's environment settings

## Features

### Chat Interface
- **Clean, modern UI** with professional styling
- **Real-time message updates**
- **Loading indicators**
- **Error handling and user feedback**

### RAG Pipeline
- **Semantic search** using vector embeddings
- **Context-aware responses** based on textbook content
- **Source attribution** showing where information comes from
- **Confidence scores** for answer reliability
- **User-selected text context** for focused queries

### Text Selection
- Users can select text on the page
- Selected text is sent as context to the chatbot
- Enhanced responses based on specific content

### Conversation History
- Maintains conversation context
- Multi-turn conversations supported
- Clear messages for better understanding

## Development

### Adding New Features

1. **Backend**: Update `src/services/rag_service.py` for RAG logic
2. **API**: Add endpoints in `src/api/chat.py`
3. **Frontend**: Update `ChatWidget.jsx` for UI changes

### Testing Locally

```bash
# Test backend endpoints
curl -X POST http://localhost:8000/chat/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?"}'

# Test health
curl http://localhost:8000/chat/health
```

## Troubleshooting

### Backend Issues

**Error: "Cannot import qdrant_client"**
```bash
pip install qdrant-client
```

**Error: "OPENAI_API_KEY not found"**
- Ensure `.env` file exists in `backend/` directory
- Check that `OPENAI_API_KEY` is properly set

**Server won't start**
- Check port 8000 is not in use
- Run: `python -m uvicorn src.app:app --port 8001`

### Frontend Issues

**Chatbot not responding**
- Verify backend is running on `http://localhost:8000`
- Check browser console for errors
- Verify CORS is enabled in backend

**Text selection not working**
- Ensure JavaScript is enabled
- Try selecting text again
- Check browser console for errors

## Performance Optimization

- Vector search: ~100ms per query
- LLM inference: ~2-5 seconds
- Total response time: ~2-6 seconds

## Security Considerations

- API keys stored in environment variables
- CORS enabled for development (restrict in production)
- Input validation on all endpoints
- No sensitive data stored in browser

## Contributing

Contributions welcome! Follow these steps:

1. Create a feature branch
2. Make changes
3. Test locally
4. Submit pull request

## License

See LICENSE file for details.

## Support

For issues or questions:
1. Check Troubleshooting section
2. Review API documentation at `/api/docs`
3. Check backend logs for errors

---

**Built with ❤️ for Physical AI & Humanoid Robotics**
