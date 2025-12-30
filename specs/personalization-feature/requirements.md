# Personalization Feature - Technical Requirements

**System Requirements, Dependencies, and Setup**

---

## 1. System Requirements

### Backend
- Python 3.9+, FastAPI, SQLAlchemy
- PostgreSQL 12+ OR SQLite 3.35+
- Redis (optional, for caching)
- 2GB RAM minimum, 4GB recommended

### Frontend
- Node.js 16+, React 18+, Docusaurus 3+
- Browser: Chrome 90+, Firefox 88+, Safari 14+
- TypeScript 5+

### Development
- Git 2.30+, Docker (optional)
- Python venv, npm/yarn
- 5GB disk space

---

## 2. Python Dependencies

```txt
# Core
fastapi==0.104.1
uvicorn==0.24.0
python-multipart==0.0.6

# Database & ORM
sqlalchemy==2.0.23
alembic==1.12.1
psycopg2-binary==2.9.9

# Data Validation
pydantic==2.5.0

# Caching (optional)
redis==5.0.0

# Testing
pytest==7.4.3
pytest-asyncio==0.21.1
httpx==0.25.2

# Development
black==23.12.0
flake8==6.1.0
```

---

## 3. JavaScript Dependencies

```json
{
  "dependencies": {
    "react": "18.2.0",
    "react-dom": "18.2.0",
    "docusaurus": "^3.0.0",
    "axios": "^1.6.2"
  },
  "devDependencies": {
    "@testing-library/react": "^14.1.0",
    "vitest": "^1.0.0"
  }
}
```

---

## 4. Database Schema

### user_personalization_preferences
```sql
CREATE TABLE user_personalization_preferences (
    id UUID PRIMARY KEY,
    user_id UUID NOT NULL UNIQUE (per chapter),
    chapter_id VARCHAR(100) NOT NULL,
    difficulty_level VARCHAR(50),
    show_theory BOOLEAN DEFAULT TRUE,
    show_exercises BOOLEAN DEFAULT TRUE,
    show_code_python BOOLEAN DEFAULT TRUE,
    show_code_cpp BOOLEAN DEFAULT FALSE,
    show_diagrams BOOLEAN DEFAULT TRUE,
    robot_types_enabled JSONB,
    learning_goal_filter VARCHAR(50),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE,
    UNIQUE (user_id, chapter_id)
);
```

### personalization_events
```sql
CREATE TABLE personalization_events (
    id UUID PRIMARY KEY,
    user_id UUID NOT NULL,
    chapter_id VARCHAR(100),
    event_type VARCHAR(50),
    preferences_from JSONB,
    preferences_to JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);
```

---

## 5. Environment Configuration

**Backend (.env)**
```env
DATABASE_URL=postgresql://user:pass@localhost/db
REDIS_URL=redis://localhost:6379
LOG_LEVEL=INFO
```

**Frontend (.env.local)**
```env
REACT_APP_API_URL=http://localhost:8000
```

---

## 6. Content Markup Format

**Markdown Section Markup:**
```markdown
<begin-section id="section-1" type="theory"
               difficulty="beginner,intermediate"
               languages="python"
               robot-types="humanoid,mobile">
Content here...
</end-section>
```

**HTML Data Attributes:**
```html
<div data-personalize="section"
     data-section-id="section-1"
     data-type="theory"
     data-difficulty="intermediate"
     data-languages="python"
     data-robot-types="humanoid,mobile"
     data-learning-goals="practical">
  Content...
</div>
```

---

## 7. API Contracts

### GET /chapters/{id}/personalization
```json
Response:
{
  "chapter_id": "module-1-chapter-01",
  "available_options": {
    "difficulty_levels": ["beginner", "intermediate", "advanced"],
    "robot_types": ["humanoid", "mobile", "arm", "drone", "digital-twin"]
  },
  "current_preferences": { ... },
  "user_profile_suggestions": { ... }
}
```

### POST /chapters/{id}/personalization
```json
Request:
{
  "difficulty_level": "intermediate",
  "show_theory": true,
  "show_exercises": true,
  "robot_types_enabled": ["humanoid"]
}

Response: { "status": 201, "saved": true }
```

---

## 8. Performance Targets

| Metric | Target |
|--------|--------|
| Panel load time | <100ms |
| Content filtering | <500ms |
| API response time | <150ms |
| Database query | <50ms |
| Animation (60fps) | Smooth |

---

## 9. Security

- Preferences tied to authenticated user_id
- No sensitive data in localStorage
- CSRF protection on API
- SQL injection prevention (ORM)
- XSS prevention (React escaping)

---

## 10. Testing

- **Unit Tests**: Business logic, hooks, components
- **Integration Tests**: API, database, workflows
- **E2E Tests**: Complete personalization flows
- **Target Coverage**: >80%

---

## 11. Deployment

- Docker container (Python 3.11 slim)
- Database migrations (Alembic)
- Environment-specific config
- Health check endpoint
- Monitoring and logging

---

## 12. Browser Support

| Browser | Min Version |
|---------|-------------|
| Chrome/Edge | 90+ |
| Firefox | 88+ |
| Safari | 14+ |
| Mobile | iOS 13+, Android 9+ |

---

## 13. Accessibility

- WCAG 2.1 Level AA compliance
- Keyboard navigation (Tab, Enter, Escape)
- Screen reader support (ARIA labels)
- Color contrast ratio >4.5:1
- Focus indicators visible

---

## 14. Integration Points

- **Signup/Signin**: Requires user authentication
- **User Profiles**: Uses background data from signup
- **RAG System**: Optional integration for recommendations
- **Docusaurus**: Chapter structure and content
- **Analytics**: Event tracking and metrics

---

## 15. Monitoring & Observability

**Logs:**
- API requests/responses
- Database queries
- Error events
- Performance metrics

**Metrics:**
- Button click rate
- Panel usage rate
- Preference change patterns
- Performance metrics
- Error rates

**Alerts:**
- High error rate (>1%)
- Slow API response (>500ms)
- Database connection issues
- Storage quota warnings

---

## 16. References

- [React Documentation](https://react.dev/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [SQLAlchemy Documentation](https://docs.sqlalchemy.org/)
- [Docusaurus Documentation](https://docusaurus.io/)
- [WCAG 2.1 Standards](https://www.w3.org/WAI/WCAG21/quickref/)

---

**Last Updated:** 2025-12-28
