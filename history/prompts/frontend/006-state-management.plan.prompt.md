---
ID: 006
TITLE: Frontend State Management
STAGE: plan
DATE_ISO: 2025-12-21
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: frontend-state
BRANCH: 1-rag-chatbot-backend
USER: user
COMMAND: Design frontend state management
LABELS:
  - frontend
  - state
  - context
  - management
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
---

## Prompt

Design comprehensive frontend state management using React Context API for authentication, user data, and application state.

## Response

### State Management Architecture

**Context Structure:**
```
src/context/
├── AuthContext.js              - Authentication state
├── UserContext.js              - User profile & preferences
├── BookContext.js              - Book/chapter state
├── ChatContext.js              - RAG chat history
├── TranslationContext.js       - Translation cache
└── ThemeContext.js             - Theme preferences
```

**AuthContext:**
```javascript
{
  user: {
    id: string,
    username: string,
    email: string,
    created_at: timestamp
  },
  token: string,
  refreshToken: string,
  isAuthenticated: boolean,
  isLoading: boolean,
  error: string | null,
  login: (email, password) => Promise,
  register: (username, email, password) => Promise,
  logout: () => void,
  refreshTokens: () => Promise
}
```

**UserContext:**
```javascript
{
  profile: {
    username: string,
    email: string,
    preferred_language: string,
    theme: string,
    font_size: string
  },
  preferences: {
    theme: 'light' | 'dark' | 'auto',
    language: string,
    notifications: boolean,
    auto_save: boolean
  },
  stats: {
    total_reading_time: number,
    chapters_completed: number,
    reading_streak: number,
    total_bookmarks: number
  },
  updateProfile: (data) => Promise,
  updatePreferences: (prefs) => Promise
}
```

**BookContext:**
```javascript
{
  currentModule: {
    id: string,
    title: string,
    description: string,
    chapters: []
  },
  currentChapter: {
    id: string,
    title: string,
    content: string,
    module_id: string,
    order: number
  },
  progress: {
    completion_percentage: number,
    last_read_at: timestamp,
    bookmarks: []
  },
  setCurrentModule: (moduleId) => void,
  setCurrentChapter: (chapterId) => void,
  updateProgress: (data) => Promise,
  addBookmark: (location) => Promise,
  removeBookmark: (id) => Promise
}
```

**ChatContext:**
```javascript
{
  conversations: [
    {
      id: string,
      question: string,
      answer: string,
      confidence: number,
      timestamp: timestamp,
      sources: []
    }
  ],
  currentConversation: object | null,
  isLoading: boolean,
  error: string | null,
  sendQuery: (query, context) => Promise,
  loadConversation: (id) => Promise,
  deleteConversation: (id) => Promise,
  clearHistory: () => void
}
```

**TranslationContext:**
```javascript
{
  cache: {
    'chapter-1_urdu': { content, confidence, timestamp },
    'chapter-2_spanish': { content, confidence, timestamp }
  },
  isTranslating: boolean,
  currentLanguage: string,
  translate: (chapterId, language) => Promise,
  clearCache: (chapterId) => void,
  setCurrentLanguage: (lang) => void
}
```

**ThemeContext:**
```javascript
{
  theme: 'light' | 'dark',
  setTheme: (theme) => void,
  toggleTheme: () => void,
  colors: {},
  fonts: {}
}
```

**Custom Hooks:**
```javascript
// src/hooks/
useAuth()              - Access auth context
useUser()              - Access user context
useBook()              - Access book context
useChat()              - Access chat context
useTranslation()       - Access translation context
useTheme()             - Access theme context
useLocalStorage()      - Persist state to localStorage
useAsync(fn, deps)     - Async state management
```

**State Persistence:**
- localStorage for user preferences
- sessionStorage for temporary data
- IndexedDB for large data (chat history)
- Automatic sync on login/logout

**Context Provider Hierarchy:**
```
<ThemeProvider>
  <AuthProvider>
    <UserProvider>
      <BookProvider>
        <ChatProvider>
          <TranslationProvider>
            <App />
          </TranslationProvider>
        </ChatProvider>
      </BookProvider>
    </UserProvider>
  </AuthProvider>
</ThemeProvider>
```

---

## Outcome

✅ Context structure designed
✅ State models defined
✅ Custom hooks planned
✅ Persistence strategy defined
✅ Provider hierarchy established
✅ State flow documented

**Status**: State management ready for implementation

