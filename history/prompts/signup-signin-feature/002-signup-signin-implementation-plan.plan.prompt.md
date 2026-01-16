---
id: 002
title: Signup/Signin Implementation Plan and Architecture
stage: plan
date: '2025-12-28'
model: claude-haiku-4-5-20251001
feature: signup-signin-feature
branch: feat/chatbot-ui-and-fastapi-integration
user: User
command: Create detailed implementation plan with architecture decisions, timeline, and risk analysis
labels:
  - implementation-plan
  - architecture
  - better-auth
  - fastapi
  - react
  - timeline
  - risk-analysis
  - deployment
links:
  SPEC: specs/signup-signin-feature/spec.md
  TICKET: null
  ADR: null
  PR: null
files:
  - specs/signup-signin-feature/plan.md
  - backend/app/database.py
  - backend/app/config.py
  - backend/app/services/auth_service.py
  - backend/app/services/password_service.py
  - frontend/src/hooks/useAuth.js
  - frontend/src/hooks/useSession.js
tests: []
---

# Signup/Signin Implementation Plan and Architecture

## Summary

Created comprehensive implementation plan for signup/signin authentication system with Better Auth integration. Plan includes detailed architecture design, 4-phase task decomposition (40+48+48+16 hours), realistic timeline, risk analysis, and deployment strategy.

## Plan Overview

**Project Scope**: User authentication and registration with background profiling
**Duration**: 2.5-3 weeks (120+ hours)
**Team Size**: 2-3 developers (1 backend, 1 frontend, 1 full-stack)
**Effort Breakdown**:
- Phase 1 (Foundation): 40 hours / 5 days
- Phase 2 (Backend): 48 hours / 6 days
- Phase 3 (Frontend): 48 hours / 6 days
- Phase 4 (Testing): 16+ hours / 2 days

## Key Architectural Decisions

### Decision 1: Authentication Library - Better Auth
**Chosen**: Better Auth v3 self-hosted library
**Rationale**:
- Self-hosted (no vendor lock-in)
- JWT support with refresh tokens
- Built-in password hashing
- Easy integration with FastAPI
- Good documentation and community support

**Alternatives Considered**:
- Auth0: Cloud-based, good but costly at scale
- Custom JWT: More control but higher maintenance burden
- Supabase: Good but requires PostgreSQL directly

**Trade-offs**:
- Better Auth: Small learning curve, but flexibility and control
- Time to integrate: 1-2 days (mitigated by spike task)

### Decision 2: Token Storage - HttpOnly Cookies
**Chosen**: HttpOnly secure cookies for JWT tokens
**Rationale**:
- Prevents XSS attacks (JavaScript cannot access)
- Prevents CSRF with SameSite flag
- Automatic cookie inclusion in requests
- Session persistence across page reload

**Alternatives Considered**:
- LocalStorage: Simpler but vulnerable to XSS
- SessionStorage: Same XSS vulnerability
- Custom secure storage: Overly complex

**Trade-offs**:
- HttpOnly cookies: Better security, but less flexible for cross-domain requests

### Decision 3: Password Hashing - Bcrypt Cost 12
**Chosen**: Bcrypt with cost factor 12
**Rationale**:
- Industry standard password hashing
- Cost factor 12 = ~250ms hashing time (acceptable balance)
- Built-in salt generation
- Future-proof (can increase cost factor as hardware improves)

**Alternatives Considered**:
- Cost 10: Faster (~100ms) but less secure
- Cost 14: More secure (~500ms) but too slow for user experience
- Argon2: Excellent but overkill for this project

**Trade-offs**:
- Cost 12: Good security without performance penalty

### Decision 4: Database Schema - Normalized Tables
**Chosen**: Two-table design (users + user_profiles with FK)
**Rationale**:
- Separation of concerns (auth data vs. profile data)
- Allows nullable profile fields
- Easier to add profile data later
- Better performance (no LEFT JOINs for auth queries)

**Alternatives Considered**:
- Single table: Simpler but mixing concerns
- Three tables (with sessions table): Over-engineered for Phase 1

**Trade-offs**:
- Two tables: Slight additional complexity, better design

## Architecture Design

### System Components

```
┌─────────────────────────────────────────────────────────────────┐
│                      Frontend (React/Docusaurus)                 │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────────────┐ │
│  │SignupForm   │  │SigninForm    │  │BackgroundQuestionnaire │ │
│  └─────────────┘  └──────────────┘  └────────────────────────┘ │
│         ↓                 ↓                     ↓               │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ useAuth Hook (session management, token refresh)            │ │
│  └────────────────────────────────────────────────────────────┘ │
└───────────────────────────↓────────────────────────────────────┘
                      HTTPS/Cookies
┌───────────────────────────↓────────────────────────────────────┐
│                  API Gateway / CORS Middleware                  │
└───────────────────────────↓────────────────────────────────────┘
                      RESTful API
┌───────────────────────────↓────────────────────────────────────┐
│                    FastAPI Backend                              │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │Auth Router (/api/auth/*)                                │  │
│  │├─ POST /register → Registration endpoint               │  │
│  │├─ POST /login → Authentication endpoint                │  │
│  │├─ POST /logout → Session termination                   │  │
│  │├─ GET /me → Current user info                          │  │
│  │├─ POST /profile → Background questionnaire             │  │
│  │├─ PUT /password → Password change                      │  │
│  │└─ PUT /profile → Profile update                        │  │
│  └─────────────────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │Services Layer                                           │  │
│  │├─ AuthService (registration, login, token management)  │  │
│  │├─ PasswordService (hashing, verification)              │  │
│  │├─ UserService (user CRUD operations)                   │  │
│  │└─ ProfileService (profile management)                  │  │
│  └─────────────────────────────────────────────────────────┘  │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │Middleware Layer                                         │  │
│  │├─ JWT Token Middleware (validation & refresh)          │  │
│  │├─ Rate Limiting Middleware (brute force protection)    │  │
│  │└─ CORS Middleware (cross-domain requests)              │  │
│  └─────────────────────────────────────────────────────────┘  │
└───────────────────────────↓────────────────────────────────────┘
                      SQLAlchemy ORM
┌───────────────────────────↓────────────────────────────────────┐
│                    Database Layer                               │
│  PostgreSQL (prod) / SQLite (dev)                              │
│  ┌──────────────────────┐  ┌────────────────────────────────┐  │
│  │users table           │  │user_profiles table             │  │
│  │├─ id (UUID)          │  │├─ id (UUID)                    │  │
│  │├─ email              │  │├─ user_id (FK)                 │  │
│  │├─ password_hash      │  │├─ programming_experience       │  │
│  │├─ status             │  │├─ python_proficiency           │  │
│  │├─ created_at         │  │├─ robotics_familiarity         │  │
│  │├─ updated_at         │  │├─ ai_ml_experience             │  │
│  │└─ last_login         │  │├─ hardware_interests (JSONB)   │  │
│  └──────────────────────┘  │├─ learning_goal                │  │
│                             │└─ created_at                   │  │
│                             └────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

### Data Flow

**Registration Flow:**
1. User fills signup form (email, password)
2. Frontend validates input client-side
3. POST /register → Backend validation
4. Create user record in `users` table
5. Return access token + redirect to questionnaire
6. User fills background questionnaire
7. POST /profile → Update `user_profiles` table
8. Auto-login and redirect to dashboard

**Login Flow:**
1. User enters email/password
2. POST /login → Backend authenticates
3. Verify email exists and password matches
4. Return JWT token in httpOnly cookie
5. Refresh token stored in 7-day cookie
6. Redirect to dashboard

### Key Interfaces

**Registration Request:**
```json
{
  "email": "user@example.com",
  "password": "SecurePass123!",
  "password_confirm": "SecurePass123!",
  "full_name": "John Doe (optional)",
  "organization": "MIT (optional)"
}
```

**Registration Response (201):**
```json
{
  "status": 201,
  "data": {
    "user": {
      "id": "uuid-123",
      "email": "user@example.com",
      "created_at": "2025-12-28T10:30:00Z"
    },
    "token": "eyJhbGciOiJIUzI1NiIs...",
    "token_expires_in": 900
  },
  "message": "User created, redirecting to questionnaire"
}
```

**Error Response (400/409):**
```json
{
  "status": 400,
  "error": "Invalid password format",
  "detail": "Password must contain uppercase, lowercase, number, and special character"
}
```

## 4-Phase Timeline

### Phase 1: Foundation & Setup (40 hours, 5 days)
- Day 1: Database schema design, migrations, environment setup
- Day 2: Better Auth integration spike, configuration
- Day 3: SQLAlchemy models, ORM mappings
- Day 4: Authentication middleware, JWT setup
- Day 5: Password service, error handling

**Key Deliverables**:
- Database ready with tables
- Better Auth configured
- JWT token generation/validation working
- Ready for endpoint implementation

### Phase 2: Backend Implementation (48 hours, 6 days)
- Day 1: Registration endpoint with validation
- Day 2: Login endpoint with rate limiting
- Day 3: Background profile endpoints
- Day 4: Logout, session management, GET /me
- Day 5-6: Integration tests, documentation

**Key Deliverables**:
- 7 REST API endpoints fully functional
- Rate limiting active
- Integration tests passing (>80% coverage)
- API documentation ready

### Phase 3: Frontend Implementation (48 hours, 6 days)
- Day 1: Signup form component
- Day 2-3: Background questionnaire (multi-step form)
- Day 4: Login form, protected routes
- Day 5: Profile page, session hooks
- Day 6: Styling, responsive design, frontend tests

**Key Deliverables**:
- All UI components functional
- Session persistence working
- Forms validate correctly
- Frontend tests passing (>80% coverage)
- Responsive design for mobile/tablet

### Phase 4: Testing & Documentation (16+ hours, 2 days)
- Security testing (OWASP, rate limiting, XSS, CSRF)
- Performance testing (load tests, response times)
- Documentation (API docs, setup guide, deployment)
- Code review, cleanup, final deployment

**Key Deliverables**:
- Zero security vulnerabilities
- Performance targets met
- Complete documentation
- Deployment ready

## Risk Analysis

### Risk 1: Better Auth Integration Complexity (HIGH - Score 2.5)
**Probability**: 50% (medium) | **Impact**: 3-5 days (major)
**Description**: Unexpected complexity or compatibility issues with Better Auth integration

**Mitigation**:
1. Spike task (Task 1.3, 6 hours) validates integration before main work
2. Dedicated developer for auth implementation
3. Reference Better Auth examples early
4. Pair programming on complex parts

**Contingency**:
- If integration fails, switch to custom JWT implementation (2-3 days more)
- Fallback to simpler auth library (Auth0, Supabase)

### Risk 2: Performance Degradation (MEDIUM - Score 2.0)
**Probability**: 30% | **Impact**: 2-3 days (medium)
**Description**: Password hashing (bcrypt cost 12) causes >500ms response times

**Mitigation**:
1. Load tests in Phase 4 catch performance issues early
2. Use background jobs for non-blocking password hashing
3. Monitor response times throughout development

**Contingency**:
- Adjust bcrypt cost factor down to 10 (faster, slightly less secure)
- Implement caching for frequently accessed data

### Risk 3: Database Schema Evolution (MEDIUM - Score 1.8)
**Probability**: 40% | **Impact**: 1-2 days (minor)
**Description**: Need to add fields to user_profiles during implementation

**Mitigation**:
1. Design schema carefully upfront (Phase 1, Task 1.1)
2. Use Alembic migrations for schema changes
3. Create generic JSONB field for extensibility

**Contingency**:
- Add migration task to timeline
- Backward-compatible schema changes only

### Risk 4: Token Refresh Logic Complexity (MEDIUM - Score 1.5)
**Probability**: 35% | **Impact**: 1-2 days (minor)
**Description**: Refresh token mechanism harder to implement than expected

**Mitigation**:
1. Better Auth library handles most refresh logic
2. Reference JWT best practices documentation
3. Comprehensive testing of refresh flow

**Contingency**:
- Extend Phase 2 by 1-2 days if needed
- Simplify to single token (less secure, simpler)

### Risk 5: Frontend-Backend Integration Issues (LOW - Score 1.2)
**Probability**: 20% | **Impact**: 1 day (minor)
**Description**: CORS, cookie handling, or session persistence issues

**Mitigation**:
1. Start frontend work after backend API stable (Phase 2 done)
2. Early integration testing between frontend and backend
3. Clear API contracts defined upfront

**Contingency**:
- Extend Phase 3 testing by 1 day

## Timeline & Capacity

**Team Capacity**:
- 1 Backend Developer: 78 hours/week (40 hours work + 30% overhead)
- 1 Frontend Developer: 78 hours/week
- 1 Full-stack Dev: 78 hours/week (optional, helps with integration)

**Realistic Schedule**:
- Phase 1: Days 1-5 (Mon-Fri) | 40 hours
- Phase 2: Days 6-11 (Mon-Fri) | 48 hours
- Phase 3: Days 12-17 (Mon-Fri) | 48 hours (can overlap with Phase 2 from Day 8)
- Phase 4: Days 18-19 (Mon-Tue) | 16+ hours

**Total**: 2.5 weeks with 2-3 developers (10-15 person-hours per developer)

**Buffer**: 15-20% built into estimates (included in total hours)

## Success Metrics

- ✅ All 138 tasks completed on schedule
- ✅ Zero security vulnerabilities in auth flow
- ✅ All auth endpoints meet <200ms p95 latency
- ✅ 100% user signup completion rate
- ✅ 99.9% authentication service uptime
- ✅ >80% test coverage for auth code
- ✅ All OWASP top 10 protections in place
- ✅ Complete documentation (API docs, setup guide, deployment)

---

PROMPT_TEXT: "Create detailed implementation plan with architecture decisions, 4-phase timeline, task decomposition, risk analysis, and deployment strategy for signup/signin with Better Auth and user profiling"

RESPONSE_TEXT: "Created comprehensive 500+ line implementation plan featuring: 4 major architectural decisions with trade-offs, system architecture diagram, 4-phase timeline (40+48+48+16 hours), detailed data flow, API contracts, 5 major risks with mitigation strategies, success metrics, and deployment checklist. Ready for task decomposition."
