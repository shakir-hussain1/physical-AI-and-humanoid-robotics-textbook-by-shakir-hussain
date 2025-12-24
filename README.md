# Physical AI & Humanoid Robotics: Interactive Learning Platform

A comprehensive educational textbook on Physical AI and Humanoid Robotics with an interactive web platform, AI-powered chatbot, and personalized learning experience.

**Status:** ✅ Fully Functional (Local Testing Complete)

---

## 📚 Project Overview

This project is an integrated solution combining:
- **Textbook:** Docusaurus-based interactive learning platform
- **Backend:** FastAPI with RAG chatbot and user authentication
- **AI Features:** Retrieval-Augmented Generation (RAG), JWT authentication, personalized content

### Core Modules

The textbook covers six core modules of Physical AI and Humanoid Robotics:

1. **Introduction to Physical AI & Embodied Intelligence**
2. **The Robotic Nervous System (ROS 2)** - Nodes, Topics, Services, Actions; rclpy Python integration; URDF for humanoid robots
3. **The Digital Twin** - Gazebo physics simulation; Unity visualization; Sensor simulation (LiDAR, depth cameras, IMUs)
4. **The AI-Robot Brain (NVIDIA Isaac)** - Isaac Sim; Synthetic data generation; Isaac ROS and Nav2
5. **Vision-Language-Action (VLA)** - LLM-based planning; Voice-to-action pipelines; Natural language → ROS action graphs
6. **Capstone: Autonomous Humanoid Robot** - Voice command reception; Navigation and obstacle avoidance; Object detection

---

## ✨ Features Implemented

### ✅ User Authentication & Authorization
- JWT-based authentication system
- Secure signup/signin flow
- User profile management
- Background skill assessment (software/hardware levels)
- Token refresh mechanism

### ✅ Chapter Personalization System (NEW)
- Per-chapter content customization
- 4 personalization dimensions:
  - **Difficulty Level:** Beginner → Intermediate → Advanced → Expert
  - **Content Style:** Text-Heavy, Visual, Code-Heavy, Balanced
  - **Example Density:** Minimal (1-2), Moderate (3-5), Rich (6+)
  - **Learning Pace:** Concise, Standard, Detailed
- Persistent preferences saved per user per chapter
- Visual "Personalized" status indicator
- Reset to default option with confirmation

### ✅ RAG Chatbot Integration
- Retrieval-Augmented Generation powered by Cohere
- Local keyword-based search fallback
- Context-aware responses based on textbook content
- Confidence scoring for answer quality
- Multi-turn conversation support

### ✅ Interactive Textbook Platform
- Docusaurus v3.6.3 static site generator
- Chapter-based learning structure
- Integrated auth widget in navbar
- Real-time personalization application
- Responsive design (mobile, tablet, desktop)

---

## 🚀 Tech Stack

### Frontend
- **Framework:** React 18 + Docusaurus 3.6.3
- **State Management:** React Context API (Authentication)
- **Styling:** CSS Modules + Custom CSS
- **HTTP Client:** Fetch API with Bearer tokens

### Backend
- **Framework:** FastAPI (Python)
- **Database:** In-memory storage (ready for PostgreSQL migration)
- **Authentication:** JWT tokens (HS256)
- **AI Integration:** Cohere API for RAG
- **Server:** Uvicorn (ASGI)

### Deployment
- **Frontend:** Docusaurus static build
- **Backend:** Hugging Face Spaces (production-ready)
- **API Docs:** Swagger/OpenAPI at `/api/docs`

---

## 📦 Project Structure

```
Physical-AI-and-Humanoid-Robotics/
├── backend/
│   ├── src/
│   │   ├── api/
│   │   │   ├── auth.py                    # Authentication endpoints
│   │   │   ├── chat.py                    # Chatbot endpoints
│   │   │   ├── chapter_personalization.py # Personalization CRUD
│   │   │   └── ...
│   │   ├── models/
│   │   │   └── chapter_personalization.py # Data models
│   │   ├── services/
│   │   │   ├── auth_service.py            # JWT token handling
│   │   │   ├── rag_service.py             # Cohere RAG integration
│   │   │   └── ...
│   │   └── app.py                         # FastAPI main app
│   ├── tests/                             # Test suite (organized)
│   │   ├── test_app.py
│   │   ├── test_personalization_api.py
│   │   ├── test_translation_chapters.py
│   │   └── ...
│   ├── AUTH_IMPLEMENTATION.md             # Auth setup guide
│   ├── PERSONALIZATION_FEATURE_STATUS.md  # Personalization guide
│   └── requirements.txt
│
├── src/
│   ├── components/
│   │   ├── ChapterPersonalizeButton.jsx   # Personalization button
│   │   ├── PersonalizationModal.jsx       # Preferences modal
│   │   ├── ChatWidget.jsx                 # Chatbot widget
│   │   ├── AuthForm.jsx                   # Login/Signup forms
│   │   └── ...
│   ├── services/
│   │   ├── personalizationApi.js          # Personalization API client
│   │   ├── chatApi.js                     # Chatbot API client
│   │   ├── authApi.js                     # Auth API client
│   │   └── ...
│   ├── context/
│   │   └── AuthContext.jsx                # User auth state
│   ├── theme/
│   │   └── DocItem/Layout/                # Swizzled Docusaurus component
│   ├── css/
│   │   └── custom.css                     # Personalization CSS classes
│   └── docs/
│       └── chapters/                      # Textbook content
│
├── history/
│   ├── prompts/
│   │   ├── general/                       # General PHRs
│   │   ├── 1-rag-chatbot-backend/         # Feature-specific PHRs
│   │   └── urdu-translation/
│   ├── ARCHIVED/
│   │   └── backend/                       # Legacy prompts
│   └── adr/                               # Architecture Decision Records
│
├── specs/
│   └── 1-rag-chatbot-backend/             # Feature specifications
│
├── .specify/
│   ├── memory/
│   │   └── constitution.md                # Project principles
│   ├── templates/                         # SDD templates
│   └── ...
│
├── .gitignore                             # Git ignore patterns
├── package.json                           # Frontend dependencies
├── docusaurus.config.js                   # Docusaurus configuration
└── README.md                              # This file

```

---

## 🔧 Setup Instructions

### Prerequisites
- Node.js 16+ & npm
- Python 3.9+
- Git

### Backend Setup

```bash
# Navigate to backend
cd backend

# Install Python dependencies
pip install -r requirements.txt

# Create .env file with:
# - COHERE_API_KEY=your_cohere_key
# - DATABASE_URL=postgresql://...
# - JWT_SECRET_KEY=your_secret

# Run backend server
python -m uvicorn src.app:app --host 127.0.0.1 --port 8000 --reload
```

**Backend runs on:** `http://127.0.0.1:8000`
**API Docs:** `http://127.0.0.1:8000/api/docs`

### Frontend Setup

```bash
# Install dependencies
npm install

# Create .env.local file with:
# REACT_APP_API_URL=http://127.0.0.1:8000

# Run development server
npm start
```

**Frontend runs on:** `http://localhost:3000`

---

## 🧪 Testing

### Run Backend Test Suite

All tests are organized in `backend/tests/` directory:

```bash
# Run specific test file
python backend/tests/test_personalization_api.py

# Run all tests
python -m pytest backend/tests/ -v
```

**Available Tests:**
- `test_app.py` - Basic app functionality
- `test_personalization_api.py` - Personalization CRUD operations
- `test_system.py` - System integration tests
- `test_translation_chapters.py` - Translation feature tests
- `test_urdu_translation.py` - Urdu-specific translation tests
- `test_concurrent_endpoints.py` - Concurrency tests
- `verify_setup.py` - Setup verification
- `verify_translation_feature.py` - Translation feature verification

**Personalization Tests Include:**
- User signup & authentication
- Save personalization settings
- Retrieve personalization
- Update personalization (timestamps preserved)
- Delete/reset personalization
- Get all user personalizations

**Result:** ✅ All tests passing

### Manual Testing
1. Open `http://localhost:3000` in browser
2. Sign up with test credentials
3. Navigate to any chapter
4. Click "Personalize" button (⚙️)
5. Adjust preferences
6. Click "Save & Reload"
7. Verify preferences persist after F5 refresh

---

## 📖 How to Use

### For Students
1. **Sign Up:** Create account with email and password
2. **Browse:** Navigate through chapters
3. **Personalize:** Click ⚙️ button to customize content for your learning style
4. **Chat:** Ask questions to the AI chatbot
5. **Learn:** Content adapts to your difficulty level and preferences

### For Contributors
1. Create feature branch: `git checkout -b feature/your-feature`
2. Make changes following the project constitution
3. Write tests for new features
4. Commit with clear messages
5. Create pull request
6. Wait for review and approval

---

## 🎯 Current Status

| Feature | Status | Details |
|---------|--------|---------|
| User Authentication | ✅ Complete | JWT-based, fully functional |
| Chapter Personalization | ✅ Complete | 4 dimensions, persistent storage |
| RAG Chatbot | ✅ Complete | Cohere-powered, confidence scoring |
| Textbook Platform | ✅ Complete | Docusaurus integrated, responsive |
| Local Testing | ✅ Complete | All tests organized and passing |
| API Documentation | ✅ Complete | Swagger docs at `/api/docs` |
| Error Handling | ✅ Complete | Comprehensive logging & messages |
| Code Organization | ✅ Complete | Tests organized, docs consolidated |

---

## 🧹 Recent Cleanup & Refactoring (Dec 24, 2025)

**Codebase Optimization:**
- ✅ **Consolidated Documentation:** Merged duplicate docs into single sources of truth
  - `PERSONALIZATION_SETUP.md` → `backend/PERSONALIZATION_FEATURE_STATUS.md`
  - `SETUP_AUTH.md` → `backend/AUTH_IMPLEMENTATION.md`
- ✅ **Test Organization:** Moved 10 test files to `backend/tests/` directory
- ✅ **Removed Redundant Files:** Deleted obsolete docs and demo files (~9 files, 1.5MB)
- ✅ **Improved .gitignore:** Added Python cache patterns and organized by category
- ✅ **Archived Legacy:** Moved old prompt history to `history/ARCHIVED/`

**Impact:**
- Cleaner repository structure
- Easier test discovery and execution
- Single source of truth for documentation
- Better git hygiene

---

## 🚀 Deployment

### Production Checklist
- [ ] Configure environment variables
- [ ] Set up PostgreSQL database
- [ ] Deploy backend to Hugging Face Spaces
- [ ] Deploy frontend to GitHub Pages / Vercel
- [ ] Configure CORS for production domain
- [ ] Set up monitoring & logging
- [ ] Run security audit

### Environment Variables Required

**Backend (.env)**
```
COHERE_API_KEY=your_cohere_api_key
JWT_SECRET_KEY=your_secret_key
JWT_ALGORITHM=HS256
LOG_LEVEL=INFO
DATABASE_URL=postgresql://user:password@localhost/dbname
```

**Frontend (.env.local)**
```
REACT_APP_API_URL=https://your-backend-url
```

---

## 📚 Documentation

### Feature Guides
- **[Personalization Feature](./backend/PERSONALIZATION_FEATURE_STATUS.md)** - Complete feature overview & testing guide
- **[Authentication Implementation](./backend/AUTH_IMPLEMENTATION.md)** - Auth setup & API integration guide

### Project Documentation
- **[Project Constitution](./.specify/memory/constitution.md)** - Project principles & development guidelines
- **[API Docs](http://127.0.0.1:8000/api/docs)** - Interactive Swagger documentation (when backend running)

### Development
- **[Specification Files](./specs/)** - Feature specifications and requirements
- **[Architecture Decisions](./history/adr/)** - ADRs for significant decisions
- **[Prompt History](./history/prompts/)** - Development session records

---

## 🤝 Contributing

This project uses Spec-Driven Development (SDD) with the Spec-Kit Plus system.

### Templates Available
- **Plan Template:** `.specify/templates/plan-template.md`
- **Spec Template:** `.specify/templates/spec-template.md`
- **Tasks Template:** `.specify/templates/tasks-template.md`

### Governance
Changes require:
1. Feature proposal with rationale
2. Creation of spec & plan documents
3. Implementation with tests
4. Pull request with comprehensive description
5. Review by maintainers

---

## 📝 License

This project is part of the "Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World" educational initiative.

---

## 👥 Team

- **Project Lead:** Shakir Hussain
- **Development:** AI-assisted with Claude Code
- **Documentation:** Automated generation with templates

---

## 📞 Support

For issues, questions, or contributions:
1. Check existing GitHub issues
2. Create new issue with detailed description
3. Include error logs and steps to reproduce
4. Reference relevant documentation

---

## 🎓 Educational Value

This platform demonstrates:
- Full-stack web development best practices
- AI integration in educational platforms
- Personalized learning algorithms
- Secure authentication systems
- Responsive design patterns
- Backend API design and implementation

Perfect for students and educators interested in:
- Physical AI & Robotics
- Web application development
- Machine learning integration
- Educational technology
