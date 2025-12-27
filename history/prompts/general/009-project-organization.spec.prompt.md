---
ID: 009
TITLE: Project Organization and Structure
STAGE: spec
DATE_ISO: 2025-12-21
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: project-organization
BRANCH: 1-rag-chatbot-backend
USER: user
COMMAND: Organize project structure into frontend and backend folders
LABELS:
  - organization
  - structure
  - project
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
---

## Prompt

Organize project structure to separate frontend and backend work into distinct folders while maintaining shared resources.

## Response

### Project Organization

**Final Directory Structure:**
```
Physical-AI-and-Humanoid-Robotics/
│
├── frontend/                          # Frontend - Book & UI
│   ├── docs/                         # Textbook chapters
│   ├── src/                          # React components
│   ├── static/                       # Images & assets
│   ├── package.json
│   ├── package-lock.json
│   ├── docusaurus.config.js
│   └── sidebars.js
│
├── backend/                          # Backend - API & Services
│   ├── src/                          # Python source code
│   │   ├── app.py
│   │   ├── config.py
│   │   ├── models/
│   │   ├── services/
│   │   ├── api/
│   │   ├── db/
│   │   └── utils/
│   ├── tests/                        # Test files
│   ├── venv/                         # Virtual environment
│   ├── requirements.txt
│   ├── Dockerfile
│   ├── .env.example
│   └── [Backend docs]
│
├── .git/                             # Version control
├── .github/                          # GitHub workflows
├── .specify/                         # Project specs
├── history/                          # Prompt History Records & ADRs
│   └── prompts/
│       ├── frontend/                 # Frontend PHRs
│       ├── backend/                  # Backend PHRs
│       └── general/                  # General PHRs
│
├── specs/                            # Feature specifications
├── node_modules/                     # Frontend dependencies
├── README.md                         # Main documentation
├── CLAUDE.md                         # Project rules
└── QUICK_START.md                    # Quick reference
```

**Frontend Folder Contents:**
- Book content (docs/) - All chapters organized by module
- React components (src/) - UI components, pages, theme
- Assets (static/) - Images, logos, diagrams
- Config files - package.json, docusaurus.config.js
- Technologies: React, Docusaurus, JavaScript/JSX

**Backend Folder Contents:**
- Python source code (src/) - All backend logic
- Services - Translation, RAG, Authentication
- Database models (models/) - SQLAlchemy ORM
- API endpoints (api/) - FastAPI routes
- Tests (tests/) - Unit and integration tests
- Configuration - requirements.txt, .env
- Docker - Dockerfile, Procfile
- Documentation - Setup and implementation guides

**Shared Resources (Project Root):**
- history/ - All Prompt History Records and ADRs
- specs/ - Feature specifications
- .github/ - GitHub workflows and CI/CD
- .specify/ - Project templates and tools

**Separation Benefits:**
- Clear organization of concerns
- Easy to navigate
- Frontend devs work in frontend/
- Backend devs work in backend/
- No conflicts or confusion
- Scalable structure

**Development Workflow:**
1. Frontend work → frontend/ folder
2. Backend work → backend/ folder
3. Documentation → history/ for PHRs
4. Project decisions → specs/ and CLAUDE.md
5. CI/CD → .github/ workflows

---

## Outcome

✅ Project structure organized
✅ Frontend and backend separated
✅ Shared resources centralized
✅ Clear folder hierarchy
✅ Easy for team collaboration
✅ Scalable architecture

**Status**: Project organization complete

