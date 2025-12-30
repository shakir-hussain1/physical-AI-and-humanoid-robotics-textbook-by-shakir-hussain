---
ID: 006
TITLE: API Endpoints Design
STAGE: plan
DATE_ISO: 2025-12-21
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: api-design
BRANCH: 1-rag-chatbot-backend
USER: user
COMMAND: Design complete REST API endpoints
LABELS:
  - backend
  - api
  - rest
  - endpoints
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
---

## Prompt

Design complete REST API endpoints for textbook platform with full CRUD operations, filtering, and pagination.

## Response

### REST API Endpoints Design

**Base URL:** `/api/v1`

**Authentication Endpoints:**
```
POST   /auth/register          - User registration
POST   /auth/login             - User login
POST   /auth/refresh           - Refresh JWT token
POST   /auth/logout            - User logout
GET    /auth/profile           - Get current user
PUT    /auth/profile           - Update user profile
POST   /auth/change-password   - Change password
```

**Translation Endpoints:**
```
POST   /translation/translate           - Translate chapter
GET    /translation/languages           - List supported languages
GET    /translation/history/:chapter_id - Get translation history
DELETE /translation/cache/:chapter_id   - Clear translation cache
```

**RAG Chat Endpoints:**
```
POST   /rag/query              - Query RAG system
GET    /rag/conversations      - List user conversations
GET    /rag/conversations/:id  - Get conversation details
DELETE /rag/conversations/:id  - Delete conversation
POST   /rag/health            - Health check
```

**User Personalization Endpoints:**
```
GET    /personalization/profile               - Get user preferences
PUT    /personalization/profile               - Update preferences
GET    /personalization/progress/:chapter_id  - Get chapter progress
PUT    /personalization/progress/:chapter_id  - Update chapter progress
POST   /personalization/bookmarks            - Create bookmark
GET    /personalization/bookmarks            - List bookmarks
DELETE /personalization/bookmarks/:id        - Delete bookmark
GET    /personalization/reading-stats        - Get reading statistics
```

**Content Endpoints:**
```
GET    /content/modules                       - List all modules
GET    /content/modules/:id                   - Get module details
GET    /content/chapters/:chapter_id          - Get chapter content
GET    /content/search?q=<query>&module=<id>  - Search content
GET    /content/chapters/:chapter_id/metadata - Get chapter metadata
```

**Response Format:**
```json
{
  "status": "success|error",
  "data": {},
  "message": "Optional message",
  "timestamp": "2025-12-21T...",
  "path": "/api/v1/..."
}
```

**Pagination:**
```
GET /content/search?page=1&limit=10&sort=relevance
Response: {
  "data": [...],
  "pagination": {
    "page": 1,
    "limit": 10,
    "total": 150,
    "pages": 15
  }
}
```

**Error Responses:**
```json
{
  "status": "error",
  "error": {
    "code": "TRANSLATION_FAILED",
    "message": "Failed to translate content",
    "details": {...}
  }
}
```

**Rate Limiting:**
- 100 requests/minute for authenticated users
- 20 requests/minute for unauthenticated users
- Headers: X-RateLimit-Limit, X-RateLimit-Remaining, X-RateLimit-Reset

**API Versioning:**
- Current: v1
- Path-based: `/api/v1/...`
- Backward compatibility maintained

---

## Outcome

✅ All endpoints designed
✅ Request/response schemas defined
✅ Error handling standardized
✅ Pagination strategy defined
✅ Rate limiting configured
✅ API documentation prepared

**Status**: Endpoints ready for implementation

