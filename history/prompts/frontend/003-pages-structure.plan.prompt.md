---
ID: 003
TITLE: Frontend Pages Structure
STAGE: plan
DATE_ISO: 2025-12-21
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: frontend-pages
BRANCH: 1-rag-chatbot-backend
USER: user
COMMAND: Design frontend pages structure
LABELS:
  - frontend
  - pages
  - routing
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
---

## Prompt

Design frontend pages structure with routing for book reading, authentication, user profile, and search features.

## Response

### Pages Structure

**Page Hierarchy:**

1. **Public Pages (No Auth Required)**
   - Home (`/`)
   - Browse Textbook (`/book`)
   - About (`/about`)
   - Login (`/login`)
   - Register (`/register`)

2. **Protected Pages (Auth Required)**
   - Read Chapter (`/book/module-{id}/chapter-{id}`)
   - User Profile (`/profile`)
   - My Bookmarks (`/bookmarks`)
   - Reading Progress (`/progress`)
   - Settings (`/settings`)

3. **Module Pages**
   - Module 1: ROS 2 Foundations (`/book/module-1`)
   - Module 2: Digital Twin (`/book/module-2`)
   - Module 3: NVIDIA Isaac (`/book/module-3`)
   - Module 4: VLA Systems (`/book/module-4`)

**Routing Configuration:**
```
/                     - Home page
/book                 - Textbook index
/book/:moduleId       - Module view
/book/:moduleId/:chapterId - Chapter view
/auth/login           - Login page
/auth/register        - Registration page
/profile              - User profile
/profile/settings     - Settings
/bookmarks            - Bookmarks list
/progress             - Reading progress
/search               - Search results
```

**Home Page (`/`):**
- Hero section with CTA
- Quick links to modules
- Recent activity
- Call to action to start learning

**Book Index (`/book`):**
- Module cards with descriptions
- Progress indicators
- Filter/search modules
- Quick access to last read

**Chapter Page (`/book/:moduleId/:chapterId`):**
- Chapter content from Docusaurus
- Translation button (top-right)
- Sidebar with TOC
- Chat widget (right panel)
- Progress tracker
- Bookmark button
- Navigation (prev/next)

**Profile Page (`/profile`):**
- User information
- Profile picture
- Learning statistics
- Recent bookmarks
- Reading streak
- Edit profile link

**Settings Page (`/profile/settings`):**
- Language preference
- Theme (light/dark/auto)
- Font size
- Notification preferences
- Export data option

**Search Page (`/search`):**
- Full-text search
- Filter by module
- Filter by topic
- Pagination
- Relevance sorting

---

## Outcome

✅ Page structure defined
✅ Routing hierarchy planned
✅ Protected routes configured
✅ User flows designed
✅ Feature integration mapped

**Status**: Pages ready for implementation

