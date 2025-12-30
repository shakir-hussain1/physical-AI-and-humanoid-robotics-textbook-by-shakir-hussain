# Implementation Plan Examples

Real-world examples of well-structured implementation plans for different types of projects.

---

## Example 1: Medium-Sized Feature - User Authentication (2 weeks)

# User Authentication Implementation Plan

## Executive Summary

Implement JWT-based user authentication system with login/register endpoints, password hashing, and token management. Core infrastructure for the platform.

Key metrics:
- **Estimated Duration:** 2 weeks
- **Team Size:** 2 developers
- **Total Effort:** 85 hours
- **Key Dependencies:** Database schema approval (day 1)
- **Top Risk:** JWT token refresh complexity (mitigated by early spike task)

---

## 1. Scope & Dependencies

### In Scope
- User registration endpoint with validation
- Login endpoint with password verification
- JWT token generation and validation
- Logout with token blacklist
- Authorization middleware
- Password hashing with bcrypt
- Error handling and edge cases

### Out of Scope
- Two-factor authentication (phase 2)
- Email verification (phase 2)
- Password reset flow (phase 2)
- Social login integration (phase 3)

### External Dependencies

| Dependency | Owner | Status | Impact |
|------------|-------|--------|--------|
| Database schema approval | DBA | Pending | Day 1 blocker |
| Redis access for token blacklist | DevOps | In progress | Start day 2 |
| Security review of auth approach | Security team | Not started | Day 3-4 |

---

## 2. Key Decisions

### Decision 1: Use JWT Tokens vs Sessions

**Options:**
1. JWT tokens - stateless, scalable, good for mobile
2. Session cookies - stateful, simpler, traditional
3. OAuth2 - complex, enterprise-grade

**Decision:** JWT tokens

**Rationale:**
- Stateless design enables horizontal scaling
- Works well with mobile apps (no cookie support issues)
- Aligns with microservices architecture
- Industry standard for modern APIs

**Trade-offs:**
- Pro: Scalable, works everywhere
- Pro: No server state to maintain
- Con: Token revocation requires blacklist (Redis)
- Con: Slightly more complex client code
- Con: Must manage token refresh carefully

**Risks:**
- Risk: Token refresh is complex to implement
  - Mitigation: Spike task (3 hours) before main work to validate approach
  - Use proven library (Python-jose) instead of custom implementation

---

### Decision 2: Use bcrypt for Password Hashing

**Options:**
1. bcrypt - industry standard, slow on purpose
2. Argon2 - newer, more secure, less adoption
3. PBKDF2 - older, still acceptable

**Decision:** bcrypt

**Rationale:**
- Well-tested, industry standard
- Built-in salt generation
- Slow iteration rate prevents brute force
- Available in all languages

---

## 3. Architecture Design

```
┌────────────────────────────────────┐
│      Client Application            │
│   (Web or Mobile)                  │
└───────────────┬────────────────────┘
                │
        ┌───────▼───────┐
        │  POST /login  │
        │ POST /register│
        │POST /logout   │
        └───────┬───────┘
                │
        ┌───────▼────────────────┐
        │  Auth Middleware       │
        │ Verify JWT token       │
        │ Check permissions      │
        └───────┬────────────────┘
                │
        ┌───────▼────────────────┐
        │  Auth Service          │
        │ - Generate tokens      │
        │ - Hash passwords       │
        │ - Verify credentials   │
        └───────┬────────────────┘
                │
    ┌───────────┴───────────┐
    │                       │
┌───▼────┐          ┌──────▼──┐
│Database│          │ Redis   │
│(users) │          │(blackl.)│
└────────┘          └─────────┘
```

### Component Interfaces

#### Component 1: Auth Service

**Responsibilities:**
- Generate JWT tokens with user claims
- Validate passwords against hashes
- Create new users
- Manage token blacklist

**Interfaces:**
```
register(email, password) -> User
  Input: email (string), password (plaintext)
  Output: User object (id, email, created_at)
  Error: ValidationError, DuplicateUserError

login(email, password) -> Token
  Input: email, password
  Output: Token object (token, expires_in, user)
  Error: InvalidCredentialsError

verify_token(token) -> User
  Input: JWT token string
  Output: User claims from token
  Error: TokenExpiredError, InvalidTokenError

logout(token) -> void
  Input: JWT token
  Effect: Add token to blacklist in Redis
  Error: RedisError (non-blocking)
```

#### Component 2: Auth Middleware

**Responsibilities:**
- Extract JWT token from request headers
- Validate token signature and expiration
- Reject unauthorized requests

**Interfaces:**
```
auth_required(request) -> User
  Input: HTTP request with Authorization header
  Output: User object from token claims
  Error: 401 Unauthorized if token invalid/missing

check_permission(user, required_role) -> bool
  Input: User from middleware, required role
  Output: Boolean if user has permission
  Error: 403 Forbidden if no permission
```

---

## 4. Task Decomposition

### Phase 1: Foundation (4 days, 32 hours)

**Objective:** Set up database, understand requirements, establish patterns

#### Task 1: Database Schema & User Model
- Create `users` table with: id, email, password_hash, created_at, updated_at
- Add unique index on email
- Create test fixtures
- Estimate: 4 hours (1h implementation + 1h testing + 1h doc + 1h review)
- Blocked by: DBA approval
- Includes: Schema migration script, rollback plan

#### Task 2: JWT Token Service Spike
- Research JWT implementation options
- Create proof-of-concept token generation/validation
- Document token refresh strategy
- Validate choice with team
- Estimate: 3 hours
- Blocked by: None
- Includes: Spike notes, recommended library, example code

#### Task 3: Password Hashing Service
- Implement password hashing with bcrypt
- Implement password verification
- Write unit tests (90% coverage)
- Estimate: 5 hours (2h code + 2h testing + 1h doc)
- Blocked by: Task 1 (User model)
- Acceptance Criteria:
  - [ ] Hashed password looks different each time (salt)
  - [ ] Verify correct password returns true
  - [ ] Verify wrong password returns false
  - [ ] 90%+ test coverage

#### Task 4: Auth Service Skeleton
- Create AuthService class structure
- Implement dependency injection (DB, cache)
- Write method signatures with docstrings
- Estimate: 4 hours
- Blocked by: Task 1, Task 3
- Includes: Logging, error handling patterns

#### Task 5: API Endpoint Scaffolding
- Create FastAPI app with auth routes
- Implement request validation with Pydantic
- Implement error response formatting
- Estimate: 6 hours
- Blocked by: Task 4
- Acceptance Criteria:
  - [ ] POST /register endpoint exists
  - [ ] POST /login endpoint exists
  - [ ] POST /logout endpoint exists
  - [ ] Request validation catches invalid inputs
  - [ ] Error responses follow consistent format

**Phase 1 Deliverables:**
- Database schema deployed
- Authentication services operational
- API endpoints scaffolded
- Team aligned on approach

---

### Phase 2: Core Implementation (5 days, 38 hours)

**Objective:** Implement full auth flow with all features

#### Task 6: User Registration Endpoint
- Implement POST /register endpoint
- Validate email format and uniqueness
- Hash password and store user
- Return created user object (no password)
- Write integration tests
- Estimate: 8 hours
  - Implementation: 3 hours
  - Integration tests: 3 hours
  - Manual testing: 1 hour
  - Documentation: 1 hour
- Blocked by: Task 5
- Acceptance Criteria:
  - [ ] Valid email/password creates user
  - [ ] Password stored as hash (not plaintext)
  - [ ] Duplicate email returns 409 Conflict
  - [ ] Invalid email returns 400 Bad Request
  - [ ] Response doesn't include password
  - [ ] Weak passwords rejected with helpful message
  - [ ] Integration test verifies database storage

#### Task 7: User Login Endpoint
- Implement POST /login endpoint
- Query user by email
- Verify password against hash
- Generate and return JWT token
- Estimate: 7 hours
- Blocked by: Task 3, Task 6
- Acceptance Criteria:
  - [ ] Valid credentials return JWT token
  - [ ] Token contains user claims
  - [ ] Invalid password returns 401 Unauthorized
  - [ ] Non-existent email returns 401 (same error for security)
  - [ ] Token expires in 1 hour (configurable)
  - [ ] Refresh token available for extending session

#### Task 8: JWT Verification Middleware
- Implement middleware to verify tokens
- Extract user claims from token
- Attach user to request
- Handle expired/invalid tokens
- Estimate: 6 hours
- Blocked by: Task 2, Task 7
- Acceptance Criteria:
  - [ ] Valid token allows request through
  - [ ] Expired token returns 401
  - [ ] Invalid signature returns 401
  - [ ] Missing Authorization header returns 401
  - [ ] User claims available in request context

#### Task 9: Logout & Token Blacklist
- Implement POST /logout endpoint
- Add tokens to Redis blacklist on logout
- Check blacklist in verification middleware
- Handle Redis failures gracefully
- Estimate: 5 hours
- Blocked by: Task 8
- Acceptance Criteria:
  - [ ] Logout removes valid token
  - [ ] Blacklisted token rejected even if valid signature
  - [ ] Blacklist entries expire after token expiration
  - [ ] Redis failure doesn't crash app (graceful degradation)
  - [ ] Logout returns 200 success

#### Task 10: Authorization & Permissions
- Implement role-based access control
- Create permission decorator for endpoints
- Test permission enforcement
- Estimate: 6 hours
- Blocked by: Task 8
- Acceptance Criteria:
  - [ ] Admin endpoints only accessible to admin users
  - [ ] User endpoints protected from anonymous access
  - [ ] 403 returned if user lacks permission
  - [ ] Permissions checked on every request

#### Task 11: Error Handling & Security
- Add rate limiting to prevent brute force
- Improve error messages (no info leaks)
- Add logging for security events
- Handle edge cases (null inputs, etc.)
- Estimate: 6 hours
- Blocked by: Tasks 6-10
- Acceptance Criteria:
  - [ ] 5+ failed login attempts triggers rate limit
  - [ ] Error messages don't reveal if email exists
  - [ ] All auth events logged with user/timestamp
  - [ ] Null/empty inputs handled gracefully

**Phase 2 Deliverables:**
- Complete auth flow working (register → login → access → logout)
- All API endpoints functional
- Token management working
- Integration tests passing

---

### Phase 3: Testing & Security (3 days, 15 hours)

**Objective:** Comprehensive testing and security hardening

#### Task 12: Integration Tests
- Test full registration → login → access flow
- Test error scenarios
- Test concurrent requests
- Test across multiple users
- Estimate: 6 hours
- Blocked by: Phase 2 complete
- Acceptance Criteria:
  - [ ] 10+ integration test scenarios
  - [ ] 80%+ code coverage
  - [ ] Edge cases tested (expired token, etc.)

#### Task 13: Performance & Load Testing
- Test login endpoint under load (1000 req/min)
- Verify password hashing doesn't slow down logins
- Monitor memory usage
- Estimate: 4 hours
- Acceptance Criteria:
  - [ ] Login completes in <200ms (p95)
  - [ ] No memory leaks under load
  - [ ] Token verification is <5ms

#### Task 14: Security Review & Documentation
- Security team reviews implementation
- Document security considerations
- Create API documentation with examples
- Create deployment checklist
- Estimate: 5 hours
- Acceptance Criteria:
  - [ ] Security review completed, no issues
  - [ ] API docs include auth examples
  - [ ] Deployment checklist has 15+ items
  - [ ] README has setup instructions

**Phase 3 Deliverables:**
- Comprehensive test coverage
- Security hardening complete
- Full documentation
- Production-ready

---

## 5. Timeline & Milestones

### Capacity Planning

```
Team: 2 developers (Alice & Bob)
Weekly capacity: 2 × 40 hours = 80 hours/week

Account for:
- Meetings/code review: 20% = 16 hours/week
- Unplanned issues: 10% = 8 hours/week

Realistic capacity: 56 hours/week

Total effort: 85 hours
Timeline: 85 ÷ 56 = 1.52 weeks
Rounded: 2 weeks
With buffer: 2.5 weeks (May 1-17)
```

### Week-by-Week Schedule

**Week 1 (May 1-5):**
- Mon: DBA approval of schema, Task 1 implementation
- Tue: Task 2 (spike), Task 3 (password service)
- Wed: Task 4 (Auth service), Task 5 (API scaffolding)
- Thu-Fri: Code review, refinement, Task 6 started
- **Milestone:** Foundation complete, ready for feature implementation

**Week 2 (May 8-12):**
- Mon-Wed: Tasks 6-9 (register, login, middleware, logout)
- Wed-Fri: Tasks 10-11 (permissions, security)
- Fri: Phase 2 code review
- **Milestone:** All features implemented, ready for testing

**Week 3 (May 15-17):**
- Mon-Tue: Tasks 12-14 (testing, documentation)
- Wed: Security review
- **Milestone:** Production ready, approved for deployment

### Critical Path Analysis

```
Task 1 (schema) → Task 3 (password) → Task 6 (register)
  → Task 7 (login) → Task 12 (integration tests) = 29 hours

Task 1 → Task 4 → Task 5 → Task 8 → Task 12 = 27 hours

Critical path: Schema → Password → Register → Login → Integration Tests
Length: 29 hours, determines minimum project duration
Must start immediately, monitor daily for delays
```

---

## 6. Risk Analysis

### Risk 1: JWT Token Refresh Complexity

**Probability:** 50% (known complexity, first time for this team)
**Impact:** 1-2 weeks delay (Major)
**Risk Score:** 2.5 (High)

**Mitigation:**
1. Task 2 (spike task) validates approach before main work
2. Use python-jose library (proven implementation)
3. Design clear error handling upfront
4. Pair programming with experienced developer on Task 7

**Contingency:**
If token refresh becomes blocked:
1. Use shorter token expiration (1 hour) instead of refresh tokens
2. Accept need for re-login more often
3. Defer refresh token feature to phase 2
4. Add 1-2 weeks to timeline if unavoidable

**Owner:** Alice (tech lead)
**Status:** Open (mitigated with spike task)

---

### Risk 2: Redis Availability for Token Blacklist

**Probability:** 25% (DevOps providing service, some variance possible)
**Impact:** 3-4 day delay (Moderate)
**Risk Score:** 0.875 (Medium)

**Mitigation:**
1. Get early access to Redis staging environment (done)
2. Test Redis integration early in Task 9
3. Design graceful degradation (logout works even if Redis unavailable)
4. Have DevOps number on speed dial

**Contingency:**
If Redis unavailable:
1. First 48 hours: Use in-memory blacklist (Python set)
2. Tokens expire naturally anyway (1 hour), limit exposure
3. Deploy Redis or use alternative (e.g., PostgreSQL blacklist table)
4. Add 2-3 days if we need to implement PostgreSQL fallback

**Owner:** Bob (DevOps coordinator)
**Status:** Monitoring (DevOps working on availability)

---

### Risk 3: Security Review Findings

**Probability:** 75% (always some issues in auth systems)
**Impact:** 3-5 days rework (Moderate)
**Risk Score:** 1.875 (High)

**Mitigation:**
1. Security team reviews during Task 14, not after
2. Common issues: rate limiting, timing attacks, session fixation
3. Plan for 1-2 days of rework built into Task 14 estimate
4. Have security team involved from start (Task 2 spike review)

**Contingency:**
If security review finds critical issues:
1. Address critical issues immediately (1-2 days)
2. Defer non-critical hardening to phase 2
3. Delay production deployment until critical fixed
4. Extend timeline by 3-5 days if needed

**Owner:** Security team + Alice
**Status:** Monitoring (security involved in spike review)

---

## 7. Testing Strategy

### Unit Testing (8 hours)
- Password hashing and verification functions
- JWT token generation and validation
- User model validation
- Authorization permission checks
**Target coverage:** 90%+

### Integration Testing (6 hours)
- Full registration flow
- Login → access → logout sequence
- Token refresh mechanism
- Concurrent login from multiple users
- Permission enforcement across endpoints

### Security Testing (3 hours)
- Rate limiting under brute force
- SQL injection prevention (input validation)
- Password strength validation
- Token expiration handling
- Cross-site request forgery prevention

---

## 8. Success Metrics

**Project Success Criteria:**
- [ ] All 14 tasks completed on time
- [ ] 80%+ test coverage
- [ ] Zero critical security issues in review
- [ ] API response time <200ms (p95)
- [ ] Zero production bugs in first week
- [ ] Full documentation complete

**Tracked Metrics:**
- Estimate vs actual per task
- Number of security issues found
- Test coverage percentage
- Code review feedback time
- Performance under load

---

## Example 2: Large Project - Multi-Phase RAG System

[Due to length, this would include:
- Larger scope (10+ developers, 3-4 months)
- Multiple integrated subsystems
- Complex dependencies between phases
- Detailed risk register
- Stakeholder communication plan
- Example of phased approach
- Resource allocation across teams]

---

## Writing Tips for Good Implementation Plans

### ✅ Good Examples
- **Specific tasks:** "Create POST /register endpoint with email validation and password hashing" (not "implement authentication")
- **Clear dependencies:** "Task 7 blocked by Task 3 and Task 6" (not "depends on other auth work")
- **Realistic estimates:** "8 hours (3h code + 3h test + 1h doc + 1h review)" (not "1-2 weeks")
- **Concrete risks:** "JWT token refresh may be complex; spike task for 3 hours" (not "technology might not work")

### ❌ Avoid
- **Vague tasks:** "Implement auth system" (too big, unclear)
- **No dependencies:** "All 20 tasks can be done in parallel" (unrealistic)
- **Overly optimistic:** "10 tasks, 5 developers, 1 week" (probably 2 weeks minimum)
- **Missing unknowns:** "No risks identified" (every project has risks)
- **No checkpoints:** "Done in 4 weeks, we'll see how it goes" (need weekly milestones)

### Implementation Plan Length
- Small projects (1 dev, 1-2 weeks): 5-10 pages
- Medium projects (2-3 devs, 1-2 months): 15-25 pages
- Large projects (5+ devs, 3+ months): 30-50+ pages

Keep plans concise but complete - readers should understand full scope without reading 100 pages.
