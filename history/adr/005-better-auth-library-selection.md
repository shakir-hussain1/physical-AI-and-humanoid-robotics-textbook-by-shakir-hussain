# ADR-005: Better Auth Library Selection for User Authentication

**Status:** Accepted
**Date:** 2025-12-28
**Deciders:** Development Team
**Affected Components:** Backend (FastAPI), Authentication System

---

## Context

The Physical AI and Humanoid Robotics project requires a robust user authentication system to support signup/signin with JWT token management and background profiling for content personalization. The team needed to select an authentication library that:

1. Integrates seamlessly with FastAPI backend
2. Supports JWT tokens with refresh token mechanism
3. Provides password hashing with bcrypt
4. Allows self-hosted deployment (no vendor lock-in)
5. Has good documentation and community support
6. Minimizes development time and security risks

Multiple options were evaluated with different trade-offs in terms of flexibility, maintenance burden, and cost.

---

## Decision

**We will use Better Auth v3 as the primary authentication library for the signup/signin system.**

Better Auth is a modern, self-hosted authentication library that provides:
- Seamless FastAPI integration
- Built-in JWT token support (access + refresh tokens)
- Bcrypt password hashing (configurable cost factor)
- Session management
- Extensible plugin architecture
- Active maintenance and community support

---

## Rationale

### Why Better Auth?

**Advantages:**
1. **Self-Hosted Control**: No vendor lock-in; full control over authentication flow and data
2. **JWT Native**: Built-in support for JWT tokens with configurable expiry and algorithms
3. **FastAPI Compatible**: Designed for Python async frameworks; integrates cleanly with FastAPI
4. **Bcrypt Built-In**: Password hashing with configurable cost factors (we chose 12)
5. **Refresh Token Support**: Native implementation of access + refresh token pattern
6. **Open Source**: Community-driven; can be extended if needed
7. **Good Documentation**: Better Auth has comprehensive docs and examples
8. **Learning Curve**: Moderate complexity; spike task (6 hours) validates integration

**Risk Mitigation:**
- Task 1.3 includes a spike to validate integration before main development
- If integration proves too complex, fallback options available
- Code is well-isolated in auth_service module

### Alternatives Considered

**Option 1: Auth0 (Cloud-Based)**
- ✅ Excellent security, enterprise features
- ✅ Minimal maintenance
- ❌ Vendor lock-in (cloud-dependent)
- ❌ Cost at scale (1000+ users)
- ❌ Less control over authentication flow
- **Decision: Rejected** - Cloud dependency and cost not aligned with project goals

**Option 2: Supabase (Backend-as-a-Service)**
- ✅ Good integration with PostgreSQL
- ✅ Built-in auth, real-time features
- ✅ Free tier available
- ❌ Vendor lock-in (requires Supabase infrastructure)
- ❌ Limited customization
- **Decision: Rejected** - Would lock project into Supabase ecosystem

**Option 3: Custom JWT Implementation**
- ✅ Complete control
- ✅ No dependencies
- ❌ Higher development effort (3-5 days more)
- ❌ Higher security risk (more attack surface)
- ❌ More maintenance burden
- ❌ Need to handle edge cases (token expiry, refresh logic)
- **Decision: Rejected** - Risk and effort not justified; Better Auth provides same flexibility with lower risk

**Option 4: Django-style Sessions**
- ✅ Familiar pattern
- ❌ Server-side state management (not stateless)
- ❌ Poor for distributed systems
- ❌ Doesn't fit JWT requirements
- **Decision: Rejected** - Incompatible with stateless API design

### Trade-Offs

| Aspect | Trade-Off |
|--------|-----------|
| **Flexibility vs. Speed** | Better Auth trades some flexibility for faster integration (6 hours spike vs 3-5 days custom) |
| **Control vs. Maintenance** | Better Auth provides control without full maintenance burden of custom implementation |
| **Vendor Lock-In vs. Reliability** | Better Auth is self-hosted (no vendor lock-in) but requires managing dependencies |
| **Learning Curve vs. Time-to-Market** | Moderate learning curve (6-hour spike) acceptable for shorter time-to-market |

---

## Consequences

### Positive Consequences

1. **Faster Development**: Leveraging existing library accelerates auth system development by 3-5 days
2. **Security**: Better Auth follows authentication best practices; reduces security implementation bugs
3. **Flexibility**: Self-hosted means we can extend, customize, or migrate if needed
4. **Maintenance**: Library is actively maintained; security patches available
5. **JWT Native**: First-class JWT support aligns with stateless API design
6. **Token Management**: Refresh token logic built-in; no custom implementation needed

### Negative Consequences

1. **New Dependency**: Adds external dependency to project; need to track security updates
2. **Integration Time**: Requires 6-hour spike to validate integration and approach
3. **Maintenance Risk**: If Better Auth project becomes unmaintained, need to migrate (low risk)
4. **Learning Curve**: Team needs to learn Better Auth patterns and APIs
5. **Version Lock**: Need to manage Better Auth versions carefully (pin to stable version 3.x)

### Long-Term Impact

- **Phase 1 (Signup/Signin)**: Use Better Auth as primary auth library
- **Phase 2 (Social Login)**: Extend Better Auth with OAuth providers
- **Phase 3 (2FA/MFA)**: Extend Better Auth with 2FA plugin
- **Future**: If requirements change dramatically, migration path exists but costs effort

---

## Implementation Details

### Configuration

```python
# backend/app/config.py
from better_auth import BetterAuth

auth = BetterAuth(
    secret_key=os.getenv("BETTER_AUTH_SECRET"),  # 256-bit key
    database_url=os.getenv("DATABASE_URL"),
    jwt_config={
        "algorithm": "HS256",
        "access_token_expire_minutes": 15,
        "refresh_token_expire_days": 7,
    },
    password_hash_config={
        "algorithm": "bcrypt",
        "cost_factor": 12,  # Balance security/performance
    }
)
```

### Integration Points

1. **Registration**: `auth.register(email, password)` → User created with hashed password
2. **Authentication**: `auth.authenticate(email, password)` → JWT tokens returned
3. **Validation**: `auth.validate_token(token)` → User info extracted
4. **Refresh**: `auth.refresh_token(refresh_token)` → New access token generated
5. **Middleware**: FastAPI dependency for token validation on protected routes

### Spike Task (Task 1.3: 6 hours)

The spike task validates:
- Better Auth installation and configuration
- JWT token generation and verification
- Password hashing with bcrypt cost 12
- Token refresh mechanism
- Error handling and edge cases
- Performance (hashing time ~250ms)
- Integration with FastAPI middleware

**Success Criteria for Spike:**
- [ ] Better Auth installed and configured
- [ ] Token generation/validation working
- [ ] Password hashing tested (~250ms per hash)
- [ ] Refresh token mechanism validated
- [ ] Error handling documented
- [ ] Integration approach documented
- [ ] Team comfortable with library

If spike succeeds, proceed with main development. If spike reveals critical issues, evaluate alternatives.

---

## Alternatives Not Chosen and Why

### OAuth2-Only (no Better Auth)
- OAuth2 is authorization framework, not authentication; would need additional implementation
- Better Auth provides both authentication and authorization
- Rejected: Incomplete solution

### FastAPI-Users Library
- Provides FastAPI-specific auth utilities
- Less flexible than Better Auth; more opinionated
- Rejected: Less suitable for our JWT + refresh token requirements

### Iron Session (Encrypted Sessions)
- Good for server-side sessions
- Not suitable for stateless JWT-based API
- Rejected: Incompatible with architecture

---

## Validation & Monitoring

### How We'll Know This Decision Was Right

1. **Task 1.3 Spike Completes Successfully**: Better Auth integrates cleanly with FastAPI
2. **Phase 1 Completes On Schedule**: Integration doesn't add unexpected delays
3. **Zero Auth Security Vulnerabilities**: System passes security tests
4. **Performance Targets Met**: <200ms p95 latency for auth endpoints
5. **Token Refresh Works Reliably**: No edge cases or bugs in refresh logic
6. **Team Productivity**: Development moves faster than custom implementation would

### Monitoring & Maintenance

**Quarterly Reviews:**
- Check for Better Auth security updates
- Review auth-related error rates
- Monitor performance metrics
- Assess team satisfaction with library

**Annual Review:**
- Evaluate if Better Auth still meets project needs
- Consider upgrades or alternatives
- Plan Phase 2 (social login) implementation

---

## Related Decisions

- **ADR-006**: Token Storage in HttpOnly Cookies (depends on this decision)
- **ADR-007**: Bcrypt Password Hashing Configuration (complements this decision)
- **ADR-008**: Database Schema Normalization (independent but integrated)

---

## References

- [Better Auth Documentation](https://www.better-auth.com/)
- [JWT Best Practices (RFC 7519)](https://tools.ietf.org/html/rfc7519)
- [OWASP Authentication Cheat Sheet](https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html)
- Task 1.3: Better Auth Integration Spike
- Implementation Plan: Section 3 - Architecture Design

---

## Approval

- **Proposed By:** Development Team
- **Reviewed By:** [To be assigned]
- **Approved By:** [To be assigned]
- **Approval Date:** [Date to be set]
- **Review Date:** 2026-01-28
