---
ID: 002
TITLE: React Components Architecture
STAGE: plan
DATE_ISO: 2025-12-21
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: frontend-components
BRANCH: 1-rag-chatbot-backend
USER: user
COMMAND: Design React component architecture
LABELS:
  - frontend
  - react
  - components
  - ui
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
---

## Prompt

Design reusable React component architecture for textbook interface with translation, RAG chat, personalization, and reading features.

## Response

### Component Architecture

**Directory Structure:**
```
src/components/
├── Common/
│   ├── Header.jsx
│   ├── Footer.jsx
│   ├── Navbar.jsx
│   ├── Sidebar.jsx
│   └── LoadingSpinner.jsx
├── Book/
│   ├── ChapterView.jsx
│   ├── TableOfContents.jsx
│   ├── ChapterNav.jsx
│   └── CodeBlock.jsx
├── Translation/
│   ├── TranslationButton.jsx
│   ├── TranslationModal.jsx
│   └── LanguageSelector.jsx
├── Chat/
│   ├── ChatWidget.jsx
│   ├── ChatMessage.jsx
│   ├── ChatInput.jsx
│   └── RAGResponse.jsx
├── User/
│   ├── UserProfile.jsx
│   ├── PreferenceSettings.jsx
│   ├── BookmarkList.jsx
│   └── ProgressBar.jsx
└── Auth/
    ├── LoginForm.jsx
    ├── RegisterForm.jsx
    └── ProtectedRoute.jsx
```

**Core Components:**

1. **ChapterView** (Main)
   - Displays chapter content
   - Translation button visible
   - Progress tracker
   - Props: chapterId, content

2. **TranslationButton** (Feature)
   - Triggers translation modal
   - Shows loading state
   - Displays language selector
   - Caches results

3. **ChatWidget** (Feature)
   - Sidebar RAG chat
   - Auto-expandable
   - Message history
   - Confidence scoring

4. **UserProfile** (Feature)
   - User preferences
   - Theme settings
   - Language choice
   - Reading stats

**State Management:**
- React Context API for global state
- useContext for authentication
- useReducer for chat history
- localStorage for preferences

**Styling:**
- CSS Modules for component styles
- Theme system (light/dark)
- Responsive design (mobile-first)
- Tailwind CSS utilities

**Key Features:**
- Translation with visual feedback
- Real-time chat integration
- User progress tracking
- Bookmarking system
- Preference persistence

---

## Outcome

✅ Component structure designed
✅ Reusability maximized
✅ State management planned
✅ Styling approach defined
✅ Ready for implementation

**Status**: Component specs ready

