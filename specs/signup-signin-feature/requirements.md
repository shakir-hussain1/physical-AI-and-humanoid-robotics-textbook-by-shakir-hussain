# Signup/Signin Feature - Technical Requirements

Comprehensive technical requirements and dependencies for implementing the signup/signin feature with Better Auth and user background profiling.

**Date:** 2025-12-28
**Status:** Active
**Owner:** Development Team

---

## 1. System Requirements

### 1.1 Backend (Python/FastAPI)

**Operating System:**
- Linux (Ubuntu 20.04+ LTS) - Production
- macOS 10.15+ - Development
- Windows 10/11 with WSL2 - Development

**Python:**
- Version: 3.9 or higher (3.11+ recommended)
- Virtual environment: venv or conda
- Package manager: pip 21.0+

**Required System Tools:**
- Git 2.30+
- PostgreSQL 12+ (production) or SQLite 3.35+ (development)
- Python development headers (python3-dev on Ubuntu)

### 1.2 Frontend (React/Docusaurus)

**Node.js:**
- Version: 16.14+ or 18.12+ (LTS)
- Package manager: npm 8.0+ or yarn 1.22+

**Operating System:**
- Same as backend for development environments

### 1.3 Database

**Production:**
- PostgreSQL 12 or higher
- Storage: 10GB+ initial capacity (extensible)
- Connection pool: pgBouncer or built-in SQLAlchemy pooling

**Development:**
- SQLite 3.35+ (file-based, requires no separate installation)
- In-memory SQLite for testing

### 1.4 Hardware Requirements

**Development Machine (per developer):**
- CPU: 2+ cores (4+ cores recommended)
- RAM: 8GB minimum (16GB recommended)
- Storage: 5GB free space (SSD strongly recommended)
- Network: 10 Mbps+ internet connection for package downloads

**Production Server:**
- CPU: 2+ cores (dedicated or reserved)
- RAM: 4GB minimum (8GB recommended for 1000+ concurrent users)
- Storage: 20GB+ SSD
- Network: 100+ Mbps dedicated bandwidth
- Load balancer: For HA setup (optional for Phase 1)

---

## 2. Software Dependencies

### 2.1 Backend Python Dependencies

**Core Framework:**
```
fastapi==0.104.1          # Web framework
uvicorn==0.24.0           # ASGI server
python-multipart==0.0.6   # Form data parsing
```

**Authentication & Security:**
```
better-auth==3.0.0        # Authentication library (main dependency)
python-jose==3.3.0        # JWT token handling
passlib==1.7.4            # Password hashing utilities
bcrypt==4.1.0             # Bcrypt password hashing (cost factor 12)
cryptography==41.0.7      # Encryption/decryption support
python-dotenv==1.0.0      # Environment variable management
```

**Database & ORM:**
```
sqlalchemy==2.0.23        # SQL toolkit and ORM
alembic==1.12.1           # Database migrations
psycopg2-binary==2.9.9    # PostgreSQL adapter
```

**Data Validation:**
```
pydantic==2.5.0           # Data validation using Python type hints
email-validator==2.1.0    # Email validation
```

**Security & Utilities:**
```
slowapi==0.1.9            # Rate limiting for FastAPI
python-cors==4.0.0        # CORS middleware
```

**Testing:**
```
pytest==7.4.3             # Testing framework
pytest-asyncio==0.21.1    # Async test support
httpx==0.25.2             # Async HTTP client for testing
pytest-cov==4.1.0         # Coverage reporting
faker==21.0.0             # Test data generation
```

**Development:**
```
black==23.12.0            # Code formatter
flake8==6.1.0             # Linter
mypy==1.7.1               # Type checker
```

**Full requirements.txt:**
```txt
# Core
fastapi==0.104.1
uvicorn==0.24.0
python-multipart==0.0.6

# Authentication
better-auth==3.0.0
python-jose==3.3.0
passlib==1.7.4
bcrypt==4.1.0
cryptography==41.0.7
python-dotenv==1.0.0

# Database
sqlalchemy==2.0.23
alembic==1.12.1
psycopg2-binary==2.9.9

# Validation
pydantic==2.5.0
email-validator==2.1.0

# Security
slowapi==0.1.9
python-cors==4.0.0

# Testing
pytest==7.4.3
pytest-asyncio==0.21.1
httpx==0.25.2
pytest-cov==4.1.0
faker==21.0.0

# Development
black==23.12.0
flake8==6.1.0
mypy==1.7.1
```

### 2.2 Frontend JavaScript Dependencies

**Core Framework:**
```json
{
  "react": "18.2.0",
  "react-dom": "18.2.0",
  "docusaurus": "^3.0.0"
}
```

**Form Management & Validation:**
```json
{
  "react-hook-form": "^7.48.0",
  "zod": "^3.22.0",
  "@hookform/resolvers": "^3.3.0"
}
```

**HTTP Client:**
```json
{
  "axios": "^1.6.2",
  "js-cookie": "^3.0.1"
}
```

**UI Components & Styling:**
```json
{
  "classnames": "^2.3.2",
  "clsx": "^2.0.0"
}
```

**Authentication & Session:**
```json
{
  "better-auth": "^3.0.0",
  "jose": "^5.0.0"
}
```

**Testing:**
```json
{
  "@testing-library/react": "^14.1.0",
  "@testing-library/jest-dom": "^6.1.0",
  "@testing-library/user-event": "^14.5.0",
  "vitest": "^1.0.0",
  "jsdom": "^23.0.0"
}
```

**Development:**
```json
{
  "@types/react": "^18.2.0",
  "@types/react-dom": "^18.2.0",
  "@types/node": "^20.10.0",
  "typescript": "^5.3.0",
  "eslint": "^8.55.0",
  "eslint-config-prettier": "^9.1.0",
  "prettier": "^3.1.0"
}
```

**Full package.json snippet:**
```json
{
  "dependencies": {
    "react": "18.2.0",
    "react-dom": "18.2.0",
    "docusaurus": "^3.0.0",
    "react-hook-form": "^7.48.0",
    "zod": "^3.22.0",
    "@hookform/resolvers": "^3.3.0",
    "axios": "^1.6.2",
    "js-cookie": "^3.0.1",
    "classnames": "^2.3.2",
    "clsx": "^2.0.0",
    "better-auth": "^3.0.0",
    "jose": "^5.0.0"
  },
  "devDependencies": {
    "@types/react": "^18.2.0",
    "@types/react-dom": "^18.2.0",
    "@types/node": "^20.10.0",
    "typescript": "^5.3.0",
    "@testing-library/react": "^14.1.0",
    "@testing-library/jest-dom": "^6.1.0",
    "@testing-library/user-event": "^14.5.0",
    "vitest": "^1.0.0",
    "jsdom": "^23.0.0",
    "eslint": "^8.55.0",
    "eslint-config-prettier": "^9.1.0",
    "prettier": "^3.1.0"
  }
}
```

---

## 3. Database Requirements

### 3.1 PostgreSQL Production Setup

**Connection String Format:**
```
postgresql://username:password@host:port/database_name
```

**Example:**
```
postgresql://auth_user:SecurePassword123@db.example.com:5432/physical_ai_prod
```

**Database Configuration:**
- Encoding: UTF-8
- Locale: C (for consistency)
- Max connections: 100+ (for connection pooling)
- Shared buffers: 25% of system RAM
- Effective cache size: 75% of system RAM

**SQLAlchemy Pool Configuration:**
```python
{
    "pool_size": 10,              # Connections to maintain
    "max_overflow": 20,           # Additional connections allowed
    "pool_recycle": 3600,         # Recycle connections after 1 hour
    "pool_pre_ping": True         # Test connections before use
}
```

### 3.2 Required Tables

**users table:**
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) NOT NULL UNIQUE,
    email_verified BOOLEAN DEFAULT FALSE,
    password_hash VARCHAR(255) NOT NULL,
    full_name VARCHAR(255),
    organization VARCHAR(255),
    country VARCHAR(100),
    status VARCHAR(50) DEFAULT 'active',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP,
    deleted_at TIMESTAMP,
    INDEX idx_email (email),
    INDEX idx_status (status),
    INDEX idx_created_at (created_at)
);
```

**user_profiles table:**
```sql
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL UNIQUE,
    programming_experience VARCHAR(50),
    python_proficiency VARCHAR(50),
    robotics_familiarity VARCHAR(50),
    ai_ml_experience VARCHAR(50),
    primary_hardware_focus VARCHAR(50),
    current_hardware_projects VARCHAR(50),
    hardware_interests JSONB,
    computing_platform VARCHAR(50),
    learning_goal VARCHAR(50),
    years_robotics_experience INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);
```

**sessions table (optional, for advanced session tracking):**
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL,
    token_hash VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_activity TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    ip_address VARCHAR(45),
    user_agent TEXT,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
    INDEX idx_user_id (user_id),
    INDEX idx_expires_at (expires_at)
);
```

### 3.3 Migration Strategy

**Using Alembic:**
- Keep all migrations in `migrations/versions/` directory
- Name format: `YYYYMMDD_HHMMSS_description.py`
- Test each migration on development database first
- Backup production database before migration
- Have rollback plan ready
- Use `alembic upgrade head` for forward migration
- Use `alembic downgrade -1` to rollback one revision

---

## 4. Environment Configuration

### 4.1 Backend Environment Variables

**File:** `.env` (development) or `.env.production` (production)

**Required Variables:**
```env
# Database
DATABASE_URL=postgresql://username:password@localhost:5432/physical_ai_dev
SQLALCHEMY_ECHO=false

# Better Auth
BETTER_AUTH_SECRET=your-256-bit-secret-key-here
BETTER_AUTH_URL=http://localhost:8000

# JWT Tokens
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=15
JWT_REFRESH_TOKEN_EXPIRE_DAYS=7

# Security
CORS_ORIGINS=["http://localhost:3000", "http://localhost:8080"]
ALLOWED_HOSTS=["localhost", "127.0.0.1"]
SECURE_COOKIES=false  # true in production

# Rate Limiting
RATE_LIMIT_ENABLED=true
RATE_LIMIT_REQUESTS=5
RATE_LIMIT_WINDOW_SECONDS=900

# Email (Phase 2)
SMTP_SERVER=smtp.gmail.com
SMTP_PORT=587
SMTP_USERNAME=your-email@gmail.com
SMTP_PASSWORD=your-app-password

# Logging
LOG_LEVEL=INFO
LOG_FILE=logs/app.log
```

**Production-Only Variables:**
```env
ENVIRONMENT=production
DEBUG=false
SECURE_COOKIES=true
SECURE_PROXY_HEADER_NAME=X-Forwarded-Proto
SECURE_PROXY_HEADER_VALUE=https
```

### 4.2 Frontend Environment Variables

**File:** `.env.local` (development) or `.env.production.local` (production)

**Required Variables:**
```env
REACT_APP_API_URL=http://localhost:8000
REACT_APP_BETTER_AUTH_URL=http://localhost:8000
REACT_APP_ENVIRONMENT=development
REACT_APP_LOG_LEVEL=debug
```

**Production Variables:**
```env
REACT_APP_API_URL=https://api.example.com
REACT_APP_BETTER_AUTH_URL=https://api.example.com
REACT_APP_ENVIRONMENT=production
REACT_APP_LOG_LEVEL=warn
```

---

## 5. Integration Requirements

### 5.1 Integration with Existing RAG System

**Dependency:** The signup/signin system should integrate with existing RAG knowledge base system.

**Integration Points:**
1. **User Context:** Pass user_id and user_profiles to RAG queries for personalization
2. **Content Filtering:** Use hardware_interests and learning_goal to filter RAG results
3. **User Sessions:** RAG endpoints should validate JWT tokens from auth system
4. **Logging:** Log RAG queries with user_id for analytics

**API Integration Example:**
```python
# RAG endpoint should accept user context
POST /api/rag/query
{
    "query": "What is inverse kinematics?",
    "user_id": "uuid-123",
    "context": {
        "hardware_interests": ["humanoid", "arms"],
        "programming_experience": "intermediate",
        "learning_goal": "practical"
    }
}
```

### 5.2 Integration with FastAPI Backend

**File Structure:**
```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py
│   ├── config.py
│   ├── models/
│   │   ├── __init__.py
│   │   ├── user.py
│   │   └── user_profile.py
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── auth.py
│   │   └── user.py
│   ├── routers/
│   │   ├── __init__.py
│   │   └── auth.py
│   ├── services/
│   │   ├── __init__.py
│   │   ├── auth_service.py
│   │   ├── password_service.py
│   │   └── user_service.py
│   ├── middleware/
│   │   ├── __init__.py
│   │   ├── auth.py
│   │   └── cors.py
│   └── database.py
├── migrations/
│   ├── env.py
│   └── versions/
├── tests/
│   ├── __init__.py
│   ├── conftest.py
│   └── test_auth.py
├── requirements.txt
└── .env.example
```

### 5.3 Integration with Docusaurus Frontend

**File Structure:**
```
frontend/
├── docusaurus.config.js
├── src/
│   ├── components/
│   │   ├── Auth/
│   │   │   ├── SignupForm.jsx
│   │   │   ├── SigninForm.jsx
│   │   │   └── BackgroundQuestionnaire.jsx
│   │   ├── Profile/
│   │   │   ├── ProfilePage.jsx
│   │   │   └── ProfileSettings.jsx
│   │   └── Layout/
│   │       ├── UserMenu.jsx
│   │       └── ProtectedRoute.jsx
│   ├── hooks/
│   │   ├── useAuth.js
│   │   ├── useSession.js
│   │   └── useApi.js
│   ├── services/
│   │   ├── api.js
│   │   ├── authService.js
│   │   └── tokenService.js
│   └── styles/
│       └── auth.css
├── package.json
└── .env.example
```

---

## 6. Security Requirements

### 6.1 Password Security

**Requirements:**
- Minimum 8 characters
- At least 1 uppercase letter (A-Z)
- At least 1 lowercase letter (a-z)
- At least 1 number (0-9)
- At least 1 special character (!@#$%^&*)
- No dictionary words or common patterns
- Bcrypt hashing with cost factor 12 (min 10, max 14)

**Hash Verification Time:** ~200-300ms per hash (acceptable for auth endpoints)

### 6.2 Session Security

**Token Configuration:**
- Access Token: 15 minutes validity
- Refresh Token: 7 days validity
- Algorithm: HS256 (HMAC with SHA-256)
- Secret Key: 256-bit random string (min 32 bytes)

**Cookie Configuration:**
- HttpOnly flag: True (prevent JavaScript access)
- Secure flag: True (HTTPS only, except development)
- SameSite: Strict (prevent CSRF)
- Path: /
- Domain: Match application domain

### 6.3 OWASP Top 10 Compliance

**A1 - Broken Access Control:**
- ✅ Use JWT tokens for request authorization
- ✅ Validate user permissions on protected endpoints
- ✅ Never expose sensitive data in token payload

**A2 - Cryptographic Failures:**
- ✅ HTTPS only in production
- ✅ Use bcrypt for password hashing
- ✅ Encrypt user data at rest (database level)
- ✅ Never log passwords or sensitive tokens

**A3 - Injection (SQL, XSS):**
- ✅ Use SQLAlchemy ORM (parameterized queries)
- ✅ Validate and sanitize all input
- ✅ Use Content-Security-Policy headers
- ✅ React's built-in XSS protection

**A4 - Insecure Design:**
- ✅ Follow authentication best practices
- ✅ Rate limiting (5 failed attempts per 15 minutes)
- ✅ CORS properly configured
- ✅ Input validation on all endpoints

**A5 - Security Misconfiguration:**
- ✅ Use environment variables for secrets
- ✅ Disable debug mode in production
- ✅ Set secure headers (CSP, X-Frame-Options, etc.)
- ✅ Regular dependency updates

**A6 - Vulnerable and Outdated Components:**
- ✅ Lock dependency versions in requirements.txt and package.json
- ✅ Monthly security updates review
- ✅ Use tools like `npm audit` and `pip-audit`

**A7 - Authentication Failures:**
- ✅ Secure password hashing (bcrypt)
- ✅ Rate limiting on login attempts
- ✅ Generic error messages (don't reveal if email exists)
- ✅ Secure session management

**A8 - Software and Data Integrity Failures:**
- ✅ Use signed JWT tokens
- ✅ Verify token signature on every request
- ✅ Pin dependency versions

**A9 - Logging and Monitoring Failures:**
- ✅ Log authentication events
- ✅ Monitor failed login attempts
- ✅ Alert on suspicious activity

**A10 - SSRF:**
- ✅ Validate all external URLs
- ✅ Whitelist allowed domains
- ✅ No user-controlled redirects

### 6.4 Rate Limiting

**Configuration:**
- Failed Login Attempts: 5 per 15 minutes per IP
- Signup Attempts: 10 per hour per IP
- Password Reset: 3 per hour per email
- General API: 100 per minute per user

**Implementation:** Use SlowAPI library for FastAPI

### 6.5 CORS Configuration

**Development:**
```python
["http://localhost:3000", "http://localhost:8080", "http://localhost:8000"]
```

**Production:**
```python
["https://example.com", "https://www.example.com"]
```

**Allowed Methods:** GET, POST, PUT, DELETE, OPTIONS
**Allowed Headers:** Content-Type, Authorization
**Credentials:** true (allow cookies)
**Max Age:** 3600 seconds

---

## 7. Performance Requirements

### 7.1 Response Time Targets

| Endpoint | Target (ms) | Threshold (p95) | Note |
|----------|-----------|-----------------|------|
| POST /register | <150 | 200 | Email validation + DB insert |
| POST /login | <100 | 150 | Password comparison |
| POST /logout | <50 | 100 | Token invalidation |
| GET /me | <50 | 100 | Current user info |
| PUT /profile | <100 | 150 | Profile update |

### 7.2 Throughput Targets

- **Concurrent Users:** 1000+
- **Requests Per Second:** 100+ (at peak)
- **Database Connections:** 20-30 (pooled)
- **Memory Usage:** <500MB for 1000 concurrent users

### 7.3 Database Query Performance

**Required Indexes:**
```sql
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_status ON users(status);
CREATE INDEX idx_users_created_at ON users(created_at);
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
```

**Query Performance Targets:**
- Email lookup: <10ms
- Password verification: <300ms (bcrypt cost)
- Profile retrieval: <50ms
- User creation: <100ms

---

## 8. Compliance & Legal

### 8.1 Data Protection

**GDPR Compliance (if applicable):**
- User consent for data collection
- Right to access user data
- Right to delete user account and data
- Data breach notification (72 hours)
- Privacy policy required

**CCPA Compliance (if applicable):**
- Right to know data collection
- Right to delete personal information
- Right to opt-out of data sharing
- Non-discrimination clause

### 8.2 Required Documentation

- Privacy Policy
- Terms of Service
- Cookie Policy
- Data Retention Policy
- Security Documentation

---

## 9. Deployment Requirements

### 9.1 Pre-Deployment Checklist

**Code:**
- [ ] All tests passing (unit + integration)
- [ ] Security tests completed
- [ ] Performance tests completed
- [ ] Code review approved
- [ ] No hardcoded secrets or credentials

**Infrastructure:**
- [ ] Database backups configured
- [ ] Monitoring and alerting set up
- [ ] Load balancer configured (if applicable)
- [ ] SSL/TLS certificates valid
- [ ] Firewall rules configured

**Operations:**
- [ ] Runbooks created
- [ ] Incident response plan ready
- [ ] On-call rotation established
- [ ] Rollback plan documented
- [ ] Team trained on deployment

### 9.2 Production Deployment

**Recommended Stack:**
- **Container:** Docker with Python 3.11 slim image
- **Orchestration:** Kubernetes (optional) or Docker Compose
- **Reverse Proxy:** Nginx 1.24+
- **SSL:** Let's Encrypt with auto-renewal
- **Database:** Managed PostgreSQL (AWS RDS, Azure Database, etc.)
- **Monitoring:** Prometheus + Grafana
- **Logging:** ELK Stack or CloudWatch

**Deployment Options:**
1. **Platform as a Service (PaaS):** Heroku, PythonAnywhere, Replit
2. **Container as a Service:** AWS ECS, Google Cloud Run, Azure Container Instances
3. **Infrastructure as a Service:** AWS EC2, Google Compute Engine, DigitalOcean
4. **Self-Hosted:** On-premise servers with Docker

---

## 10. Maintenance Requirements

### 10.1 Regular Updates

**Monthly:**
- Review and update dependencies (npm audit, pip-audit)
- Review security advisories
- Analyze performance metrics

**Quarterly:**
- Full security audit
- Database optimization review
- Capacity planning

**Annually:**
- Architecture review
- Security penetration testing
- Disaster recovery drill

### 10.2 Monitoring & Observability

**Key Metrics:**
- Authentication success/failure rate
- Average response time
- Error rate
- Failed login attempts
- Database connection pool utilization
- Memory and CPU usage

**Logs to Maintain:**
- Authentication events (login, logout, registration)
- Password changes
- Profile updates
- Failed authentication attempts
- Rate limit violations
- Database errors

**Alerting Thresholds:**
- Login failure rate > 1% over 5 minutes
- Response time p95 > 500ms
- Error rate > 0.5%
- Database connection pool > 90%
- Disk space < 10% available

---

## 11. Acceptance Criteria

### 11.1 Installation Acceptance

- [ ] Python 3.9+ installed and verified
- [ ] PostgreSQL 12+ installed and accessible
- [ ] Node.js 16+ installed and verified
- [ ] All Python dependencies installed (`pip install -r requirements.txt`)
- [ ] All JavaScript dependencies installed (`npm install`)
- [ ] Database migrations executed successfully (`alembic upgrade head`)
- [ ] Environment variables configured correctly
- [ ] Application starts without errors

### 11.2 Configuration Acceptance

- [ ] Database connection string validated
- [ ] JWT secret key configured
- [ ] CORS origins configured
- [ ] Email SMTP settings configured (if applicable)
- [ ] Rate limiting thresholds set
- [ ] Logging levels configured
- [ ] Security headers configured

### 11.3 Security Acceptance

- [ ] No hardcoded secrets in code
- [ ] All passwords hashed with bcrypt
- [ ] HTTPS enforced in production
- [ ] CORS properly configured
- [ ] Rate limiting active
- [ ] Input validation on all endpoints
- [ ] SQL injection prevention verified
- [ ] XSS protection enabled

---

## 12. Support & Resources

### 12.1 Documentation References

- [Better Auth Documentation](https://www.better-auth.com/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [React Documentation](https://react.dev/)
- [SQLAlchemy Documentation](https://docs.sqlalchemy.org/)
- [JWT Best Practices](https://tools.ietf.org/html/rfc7519)
- [OWASP Top 10](https://owasp.org/www-project-top-ten/)

### 12.2 Contact & Escalation

- **Technical Lead:** [To be assigned]
- **Security Contact:** [To be assigned]
- **DevOps Contact:** [To be assigned]
- **Product Owner:** [To be assigned]

### 12.3 Issue Reporting

- **Bug Reports:** GitHub Issues with `[AUTH]` prefix
- **Security Issues:** Private email to security team
- **Performance Issues:** GitHub Discussion with `[PERF]` tag
- **Feature Requests:** GitHub Discussion with `[FEATURE]` tag

---

## 13. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-28 | Initial requirements document |

---

## 14. Appendices

### A. Glossary

- **JWT:** JSON Web Token - stateless authentication token
- **Bcrypt:** Password hashing algorithm with built-in salt
- **Rate Limiting:** Restricting number of requests within time window
- **CORS:** Cross-Origin Resource Sharing - allows cross-domain requests
- **OWASP:** Open Web Application Security Project
- **ORM:** Object-Relational Mapping - database abstraction layer
- **ASGI:** Asynchronous Server Gateway Interface
- **SQLAlchemy:** Python SQL toolkit and ORM

### B. Useful Commands

**Backend Setup:**
```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run migrations
alembic upgrade head

# Start development server
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Run tests
pytest tests/ -v --cov=app
```

**Frontend Setup:**
```bash
# Install dependencies
npm install

# Start development server
npm start

# Build for production
npm run build

# Run tests
npm test

# Lint code
npm run lint
```

**Database:**
```bash
# Create new migration
alembic revision --autogenerate -m "Add users table"

# Downgrade one revision
alembic downgrade -1

# View database
psql physical_ai_dev  # Connect to PostgreSQL
```

---

**Document Status:** Ready for Development
**Last Updated:** 2025-12-28
**Next Review Date:** 2026-01-28
