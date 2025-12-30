# Physical AI & Humanoid Robotics: Interactive Learning Platform

[![Status](https://img.shields.io/badge/Status-Production%20Ready-brightgreen)](.)
[![Frontend](https://img.shields.io/badge/Frontend-React%2018%20%2B%20Docusaurus%203.6-blue)](.)
[![Backend](https://img.shields.io/badge/Backend-FastAPI%20%2B%20PostgreSQL-green)](.)
[![AI](https://img.shields.io/badge/AI-Cohere%20RAG%20%2B%20Translation-orange)](.)

A comprehensive interactive learning platform for Physical AI and Humanoid Robotics with AI-powered chatbot, personalized learning, and multi-language support.

---

## 🎯 Quick Start

### Prerequisites
- **Node.js** 16+ & npm
- **Python** 3.9+
- **Git**

### Local Development (30 seconds)

```bash
# 1. Clone repository
git clone https://github.com/shakir-hussain1/physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain.git
cd physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain

# 2. Backend Setup
cd backend
pip install -r requirements.txt
python -m uvicorn src.api.main:app --host 0.0.0.0 --port 8001

# 3. Frontend Setup (NEW TERMINAL)
cd ../
npm install
npm start
```

**Access:**
- Frontend: http://localhost:3000
- Backend API: http://localhost:8001/api
- Swagger Docs: http://localhost:8001/api/docs

---

## ✨ Core Features

### 1. 🔐 User Authentication System
```
✅ JWT-based authentication (HS256)
✅ Secure signup/signin flow
✅ User profile management
✅ Email validation
✅ Token refresh mechanism
```

**Usage:**
- Click "Sign Up" in header
- Enter email, password, name
- Get JWT token automatically
- Token valid for 15 minutes (refresh for 7 days)

---

### 2. 📚 Content Personalization Panel (NEW!)

**Three-Level Hierarchy:**
```
Global Settings (all chapters)
    ↓ Override with
Module Settings (specific modules)
    ↓ Override with
Chapter Settings (specific chapters)
```

**Customizable Options:**
```
Content Level:        Beginner → Intermediate → Advanced
Show Math:            ☑️ Yes / ☐ No
Show Code:            ☑️ Yes / ☐ No
Show Diagrams:        ☑️ Yes / ☐ No
Show Advanced Topics: ☑️ Yes / ☐ No
```

**How to Use:**

1. **Global Settings:**
   - Click "Settings" button in header
   - Go to "Global" tab
   - Select content level
   - Toggle content types
   - Click "Save Preferences"

2. **Module Settings:**
   - Click "Settings" → "Modules" tab
   - Click "+ Create new module preference..."
   - Enter module ID (e.g., "robotics_101")
   - Configure settings
   - Click "Save Module Settings"

3. **Chapter Settings:**
   - Click "Settings" → "Chapters" tab
   - Click "+ Create new chapter preference..."
   - Enter chapter ID + optional module ID
   - Configure settings
   - Click "Save Chapter Settings"

**Features:**
- Persistent preferences (saved in database)
- Immediate application
- Override system at any level
- Reset to defaults option

---

### 3. 🤖 Simplified RAG Chatbot (NEW!)

**Enhanced Features:**
```
✅ Direct, concise answers
✅ One highly relevant source (>70% match)
✅ Clean UI (no clutter)
✅ Fast response times
✅ Multi-turn conversations
```

**How It Works:**

1. Click 💬 button (bottom-right corner)
2. Ask any question about robotics/AI:
   - "What is ROS?"
   - "Explain digital twins"
   - "How do humanoid robots move?"
3. Get direct answer + 1 relevant source

**Improvements:**
- Removed confidence badges
- Removed latency metrics
- Removed multiple irrelevant sources
- Focused on answer clarity
- Mobile-friendly design

---

### 4. 🌍 Urdu Translation Support (NEW!)

**Automatic Translation:**
```
20+ robotics/AI terms translated
English → Urdu automatically
Database caching for performance
Fallback to original if no translation
```

**Supported Terms:**
```
Robot             → روبوٹ
Robotics          → روبوٹکس
ROS               → روبوٹ آپریٹنگ سسٹم
Digital Twin      → ڈیجیٹل جڑواں
Humanoid          → انسان نما
Machine Learning  → مشین لرننگ
Artificial Intelligence → مصنوعی ذہانت
... and more
```

**Usage:**
- Automatic on page load
- Click translation button to toggle
- Works with Personalization system

---

### 5. 📖 Interactive Textbook Platform

**Built with Docusaurus 3.6.3:**
```
✅ 6 core modules
✅ Chapter-based structure
✅ Real-time personalization
✅ Dark/Light theme
✅ Mobile responsive
✅ Search functionality
✅ Quick navigation
```

**Modules:**
1. Introduction to Physical AI
2. Robotic Nervous System (ROS 2)
3. Digital Twins (Gazebo, Unity)
4. AI-Robot Brain (NVIDIA Isaac)
5. Vision-Language-Action (VLA)
6. Capstone: Autonomous Humanoid Robot

---

## 🏗️ Architecture

### System Diagram
```
┌─────────────────────────────────────┐
│     Frontend (React + Docusaurus)   │
│  - Auth Widget                      │
│  - Personalization Panel            │
│  - RAG Chatbot Widget               │
│  - Interactive Chapters             │
└────────────┬────────────────────────┘
             │ HTTPS
┌────────────▼────────────────────────┐
│   Backend (FastAPI + PostgreSQL)    │
│  - Auth Service (JWT)               │
│  - Personalization Service          │
│  - RAG Chatbot Service              │
│  - Translation Service              │
│  - Retrieval Service                │
└────────────┬────────────────────────┘
             │
┌────────────▼────────────────────────┐
│  External Services                  │
│  - Cohere API (RAG)                 │
│  - OpenAI/Anthropic (Fallback)      │
│  - Qdrant (Vector DB)               │
│  - PostgreSQL (User Data)           │
└─────────────────────────────────────┘
```

### Tech Stack

**Frontend:**
```
React 18.2.0          - UI library
Docusaurus 3.6.3      - Documentation framework
CSS Modules           - Component styling
Fetch API             - HTTP requests
React Context         - State management
```

**Backend:**
```
FastAPI 0.109.0       - Web framework
Uvicorn 0.27.0        - ASGI server
SQLAlchemy 2.0.23     - ORM
PostgreSQL            - Database
Alembic               - Migrations
```

**AI & LLM:**
```
Cohere 4.37           - RAG (primary)
OpenAI 1.3.9          - LLM (fallback)
Anthropic 0.7.10      - LLM (fallback)
google-trans-new      - Translation API
```

**Vector Database:**
```
Qdrant-client 2.7.3   - Vector storage
Cohere Embeddings     - 1024-dim vectors
```

---

## 📁 Project Structure

```
Physical-AI-and-Humanoid-Robotics/
├── backend/                              # FastAPI backend
│   ├── src/
│   │   ├── api/
│   │   │   ├── endpoints/               # API routes
│   │   │   ├── auth_service.py          # Authentication
│   │   │   ├── translation_service.py   # Urdu translation
│   │   │   ├── personalization_service.py # Preferences
│   │   │   └── ...
│   │   ├── agent/                       # RAG chatbot logic
│   │   │   ├── orchestrator.py          # Main workflow
│   │   │   ├── prompt_builder.py        # LLM prompts
│   │   │   ├── retrieval_tool.py        # Vector search
│   │   │   └── ...
│   │   └── retrieval/                   # Vector database
│   ├── tests/                           # Test suite
│   ├── requirements.txt
│   └── .env.example
│
├── src/                                  # React components
│   ├── components/
│   │   ├── PersonalizationPanel.jsx     # Settings panel
│   │   ├── ChatbotWidget.jsx            # Chatbot UI
│   │   ├── UrduTranslationPanel.jsx     # Translation panel
│   │   └── AuthForm.jsx                 # Login/signup
│   ├── hooks/
│   │   ├── useAuth.js                   # Auth logic
│   │   ├── usePersonalization.js        # Preferences logic
│   │   └── ...
│   ├── theme/
│   │   └── Layout/                      # Theme customization
│   ├── css/
│   │   └── custom.css
│   └── docs/                            # Markdown chapters
│
├── docs/                                 # Documentation
├── specs/                                # Feature specs
├── history/                              # Development records
├── .claude/                              # Claude Code config
├── .specify/                             # Spec-driven templates
├── .env.example                          # Environment template
├── docusaurus.config.js
├── package.json
├── CLAUDE.md
├── README.md
├── CONTENT_PERSONALIZATION_GUIDE.md      # Feature documentation
├── RAG_CHATBOT_SIMPLIFIED.md             # Feature documentation
└── URDU_TRANSLATION_COMPLETE.md          # Feature documentation
```

---

## 🚀 Deployment

### Frontend Deployment (Docusaurus)

**Build Static Site:**
```bash
npm run build
# Output: build/ directory (production-ready)
```

**Deploy to:**
- GitHub Pages
- Vercel
- Netlify
- Any static hosting

### Backend Deployment (Hugging Face Spaces)

**Method 1: Using Dockerfile**
```bash
# Build Docker image
docker build -t robotics-api .

# Run container
docker run -p 8001:8001 --env-file .env robotics-api
```

**Method 2: Hugging Face Spaces**
1. Create new Space on huggingface.co
2. Set Docker runtime
3. Push repository
4. Add environment variables via Space settings
5. Automatic deployment!

**Required Environment Variables:**
```
COHERE_API_KEY          # For RAG chatbot
OPENAI_API_KEY          # Fallback LLM
DATABASE_URL            # PostgreSQL connection
JWT_SECRET_KEY          # Token signing
QDRANT_URL              # Vector database
QDRANT_API_KEY          # Vector DB auth
```

---

## 🧪 Testing

### Backend Tests
```bash
cd backend

# Run all tests
pytest

# Run specific test
pytest tests/test_auth.py

# Coverage report
pytest --cov=src tests/
```

### Frontend Tests
```bash
# Run tests
npm test

# Coverage
npm test -- --coverage
```

### Manual Testing

**Authentication:**
```bash
# Signup
curl -X POST http://localhost:8001/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test@123456",
    "first_name": "Test",
    "last_name": "User"
  }'
```

**Chatbot Query:**
```bash
curl -X POST http://localhost:8001/api/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -d '{
    "query": "What is ROS?",
    "conversation_history": [],
    "user_role": "student"
  }'
```

**Personalization:**
```bash
# Set global preference
curl -X POST http://localhost:8001/api/personalization/global-preference \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -d '{
    "content_level": "intermediate",
    "show_math": true,
    "show_code": true,
    "show_diagrams": true,
    "show_advanced_topics": false
  }'
```

---

## 📊 Performance

### Response Times
- Chatbot response: **< 2 seconds**
- Personalization update: **< 500ms**
- Authentication: **< 200ms**
- Translation: **< 1 second**

### Database
- PostgreSQL with optimized indexes
- Caching for translated terms
- Vector database (Qdrant) for semantic search

### Frontend
- Lazy loading of components
- Code splitting enabled
- CSS optimization
- Asset minification

---

## 🔐 Security

### Authentication
```
✅ JWT tokens (HS256)
✅ Secure password hashing (bcrypt)
✅ Token expiration (15 min access, 7 day refresh)
✅ HTTPS-only in production
✅ CORS configured
```

### Data Protection
```
✅ Hashed passwords in database
✅ No sensitive data in cookies
✅ Environment variables for secrets
✅ SQL injection prevention (SQLAlchemy ORM)
✅ Input validation on all endpoints
```

---

## 📝 Documentation

### Feature Guides
- **[Content Personalization Guide](./CONTENT_PERSONALIZATION_GUIDE.md)** - Complete feature documentation
- **[RAG Chatbot Usage](./RAG_CHATBOT_SIMPLIFIED.md)** - How to use the chatbot
- **[Urdu Translation Guide](./URDU_TRANSLATION_COMPLETE.md)** - Translation feature details

### Developer Documentation
- **[CLAUDE.md](./CLAUDE.md)** - Project development rules
- **[Architecture Decisions](./history/adr/)** - ADR records
- **[Development History](./history/prompts/)** - Prompt history records

### API Documentation
- **Swagger UI:** http://localhost:8001/api/docs
- **ReDoc:** http://localhost:8001/api/redoc

---

## 🤝 Contributing

### Development Workflow
1. Create feature branch: `git checkout -b feat/your-feature`
2. Make changes and test locally
3. Commit: `git commit -m "feat: description"`
4. Push: `git push origin feat/your-feature`
5. Create Pull Request
6. Get review and merge

### Code Style
- Python: Follow PEP 8
- JavaScript: Use ESLint configuration
- Commits: Conventional commits format

### Testing Requirements
- Backend: Minimum 80% coverage
- Frontend: Test critical paths
- Integration: Test API endpoints

---

## 📦 Dependencies

### Backend (`requirements.txt`)
- fastapi==0.109.0
- uvicorn==0.27.0
- sqlalchemy==2.0.23
- pydantic==2.5.0
- pyjwt==2.8.1
- python-jose==3.3.0
- cohere==4.37
- openai==1.3.9
- anthropic==0.7.10
- qdrant-client==2.7.3
- google-trans-new==1.1.9

### Frontend (`package.json`)
- react==18.2.0
- docusaurus==3.6.3
- @docusaurus/preset-classic==3.6.3

---

## 🐛 Troubleshooting

### Backend Issues

**Port already in use:**
```bash
# Find process using port 8001
lsof -i :8001
# Kill it
kill -9 <PID>
```

**Database connection error:**
```bash
# Check DATABASE_URL in .env
# Verify PostgreSQL is running
# Check network connectivity
```

**Cohere API error:**
```bash
# Verify COHERE_API_KEY is set
# Check API key validity
# Check rate limits
```

### Frontend Issues

**Module not found:**
```bash
# Clear node_modules and reinstall
rm -rf node_modules package-lock.json
npm install
```

**API connection error:**
```bash
# Check backend is running on port 8001
# Verify REACT_APP_API_URL in .env.local
# Check network connectivity
```

---

## 📈 Roadmap

### Phase 2 (Planned)
- [ ] Advanced analytics dashboard
- [ ] Peer recommendations
- [ ] Interactive exercises
- [ ] Code execution environment
- [ ] Real-time collaboration

### Phase 3 (Future)
- [ ] Mobile native app
- [ ] Video content integration
- [ ] Assessment system
- [ ] Certificate generation
- [ ] Advanced search filters

---

## 📄 License

This project is open source. See [LICENSE](./LICENSE) for details.

---

## 👨‍💻 Author

**Shakir Hussain**
- GitHub: [@shakir-hussain1](https://github.com/shakir-hussain1)
- Email: contact@example.com
- Website: https://humanoid-robotics.example.com

---

## 🙏 Acknowledgments

- **Cohere** - RAG and AI capabilities
- **Docusaurus** - Documentation platform
- **FastAPI** - Modern web framework
- **Qdrant** - Vector database
- **HuggingFace** - Deployment platform

---

## 📞 Support

### Getting Help

1. **Documentation**: Check [docs](./docs/)
2. **Issues**: Open [GitHub Issue](https://github.com/shakir-hussain1/physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain/issues)
3. **Discussions**: Start [GitHub Discussion](https://github.com/shakir-hussain1/physical-AI-and-humanoid-robotics-textbook-by-shakir-hussain/discussions)
4. **Email**: contact@example.com

---

## ⭐ Show Your Support

If you found this project helpful, please star it! ⭐

---

**Last Updated:** December 30, 2025
**Status:** ✅ Production Ready
**Version:** 2.0.0
