---
ID: 005
TITLE: Frontend Backend API Integration
STAGE: plan
DATE_ISO: 2025-12-21
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: frontend-api-integration
BRANCH: 1-rag-chatbot-backend
USER: user
COMMAND: Design frontend-backend API integration
LABELS:
  - frontend
  - api
  - integration
  - services
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
---

## Prompt

Design frontend service layer for consuming backend APIs with authentication, error handling, and state management.

## Response

### Frontend API Integration

**Service Architecture:**
```
src/services/
├── api.js              - API client configuration
├── authService.js      - Authentication
├── translationService.js - Translation API
├── ragService.js       - RAG chat API
├── userService.js      - User management
└── personalizationService.js - User preferences
```

**API Client Configuration:**
```javascript
// src/services/api.js
import axios from 'axios';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8001';

const api = axios.create({
  baseURL: API_BASE_URL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Interceptor for JWT token
api.interceptors.request.use((config) => {
  const token = localStorage.getItem('authToken');
  if (token) {
    config.headers.Authorization = `Bearer ${token}`;
  }
  return config;
});
```

**Auth Service:**
```javascript
// Login
POST /auth/login
Body: { email, password }
Response: { access_token, refresh_token, user }

// Register
POST /auth/register
Body: { username, email, password }

// Refresh
POST /auth/refresh
Header: Authorization: Bearer <refresh_token>

// Profile
GET /auth/profile
Header: Authorization: Bearer <access_token>
```

**Translation Service:**
```javascript
// Translate chapter
POST /translation/translate
Headers: Authorization: Bearer <token>
Body: {
  content: "<p>Text to translate</p>",
  chapter_id: "chapter-1",
  target_language: "urdu"
}
Response: {
  status: "success",
  translated_content: "<p>...</p>",
  confidence_score: 0.95,
  from_cache: false
}
```

**RAG Chat Service:**
```javascript
// Query RAG
POST /rag/query
Headers: Authorization: Bearer <token>
Body: {
  query: "What is ROS?",
  context: { chapter_id: "..." }
}
Response: {
  answer: "...",
  sources: [...],
  confidence: 0.87
}
```

**User Service:**
```javascript
// Get user profile
GET /auth/profile
Header: Authorization: Bearer <token>

// Update profile
PUT /personalization/profile
Header: Authorization: Bearer <token>
Body: { preferred_language, theme, font_size }

// Get progress
GET /personalization/progress/:chapter_id
Header: Authorization: Bearer <token>

// Update progress
PUT /personalization/progress/:chapter_id
Body: { completion_percentage, last_read_at }
```

**Error Handling:**
- 400: Bad Request - User error
- 401: Unauthorized - Invalid/expired token
- 403: Forbidden - Insufficient permissions
- 404: Not Found - Resource doesn't exist
- 500: Server Error - Backend issue
- Network errors - Offline handling

**Request/Response Flow:**
1. Component makes API call via service
2. Service adds auth token from localStorage
3. API interceptor handles 401 (refresh token)
4. Response cached in Context/local state
5. Error handling with user-friendly messages
6. Loading states managed in component

**API Error Handling:**
```javascript
try {
  const response = await api.post('/translation/translate', data);
  return response.data;
} catch (error) {
  if (error.response?.status === 401) {
    // Refresh token and retry
  } else if (error.response?.status === 429) {
    // Rate limit error
  } else {
    // Show error to user
  }
}
```

---

## Outcome

✅ Service layer architecture designed
✅ API endpoints documented
✅ Authentication flow defined
✅ Error handling strategy planned
✅ Interceptor pattern implemented
✅ Token refresh mechanism designed

**Status**: Ready for service implementation

