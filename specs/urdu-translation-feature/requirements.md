# Urdu Translation Feature - Technical Requirements

**System Requirements, Dependencies, and Setup**

---

## 1. System Requirements

### Backend
- Python 3.9+, FastAPI
- PostgreSQL 12+ OR SQLite
- Redis (optional, for caching)
- Google Translate API credentials
- 2GB RAM minimum

### Frontend
- Node.js 16+, React 18+, Docusaurus 3+
- Browser: Chrome 90+, Firefox 88+, Safari 14+
- TypeScript 5+

---

## 2. Python Dependencies

```txt
google-cloud-translate==3.11.0
redis==5.0.0
fastapi==0.104.1
sqlalchemy==2.0.23
```

---

## 3. JavaScript Dependencies

```json
{
  "dependencies": {
    "react-i18next": "^13.0.0",
    "i18next": "^23.0.0"
  }
}
```

---

## 4. Database Schema

### chapter_translations
```sql
CREATE TABLE chapter_translations (
    id UUID PRIMARY KEY,
    chapter_id VARCHAR(100) NOT NULL,
    language VARCHAR(10) NOT NULL,
    status VARCHAR(50),
    content LONGTEXT,
    accuracy_score FLOAT,
    last_updated TIMESTAMP,
    UNIQUE (chapter_id, language)
);
```

### translation_terminology
```sql
CREATE TABLE translation_terminology (
    id UUID PRIMARY KEY,
    english_term VARCHAR(255) UNIQUE,
    urdu_translation VARCHAR(255),
    category VARCHAR(50),
    approved BOOLEAN,
    usage_count INTEGER
);
```

### user_language_preference
```sql
CREATE TABLE user_language_preference (
    id UUID PRIMARY KEY,
    user_id UUID UNIQUE,
    chapter_id VARCHAR(100),
    preferred_language VARCHAR(10),
    FOREIGN KEY (user_id) REFERENCES users(id)
);
```

---

## 5. Environment Configuration

```env
GOOGLE_TRANSLATE_API_KEY=your_key_here
GOOGLE_TRANSLATE_PROJECT_ID=your_project_id
REDIS_URL=redis://localhost:6379
DATABASE_URL=postgresql://user:pass@localhost/db
```

---

## 6. API Contracts

### GET /chapters/{id}/translation/{lang}
```json
Response: {
  "chapter_id": "...",
  "language": "ur",
  "status": "expert_verified",
  "content": "ترجمہ شدہ مواد...",
  "accuracy_score": 98
}
```

### POST /user/language-preference
```json
Request: {
  "preferred_language": "ur",
  "chapter_id": "module-1-chapter-01"
}
Response: { "status": 201, "saved": true }
```

---

## 7. Performance Targets

| Metric | Target |
|--------|--------|
| Translation toggle | <200ms |
| API response | <1s |
| Page load with translation | <2s |
| Cache hit | <50ms |

---

## 8. RTL Support

**CSS Changes Needed:**
- `direction: rtl`
- `text-align: right` (for body)
- Reversed margins/padding
- Reversed float/border direction

**Libraries:**
- RTL-aware CSS framework
- Proper Urdu font (e.g., Noto Sans Urdu)

---

## 9. Security

- API key management (environment variables)
- Rate limiting on translation endpoint
- Validate user authentication
- HTTPS for API calls

---

## 10. Accessibility

- WCAG 2.1 AA compliance
- RTL-aware labels (ARIA)
- Keyboard navigation in RTL
- Screen reader support
- Proper language declaration

---

## 11. Testing

- Unit tests (70+ for Phase 1-2)
- Integration tests (25+ for API)
- E2E tests (15+ for workflows)
- RTL layout tests
- Performance tests
- Accessibility tests

---

## 12. Browser Support

| Browser | Min Version |
|---------|-------------|
| Chrome/Edge | 90+ |
| Firefox | 88+ |
| Safari | 14+ |
| Mobile | iOS 13+, Android 9+ |

---

## 13. Urdu Font Requirements

**Recommended Fonts:**
- Noto Sans Urdu (free)
- Jura (supports Urdu)
- UdadanW3

**Font Loading:**
```css
@font-face {
  font-family: 'Urdu';
  src: url('noto-sans-urdu.woff2') format('woff2');
}

[lang="ur"] {
  font-family: 'Urdu', sans-serif;
  font-size: 16px;
}
```

---

## 14. Terminology Database File

**Format:** JSON or CSV
```json
[
  {"en": "Robot", "ur": "روبوٹ", "category": "general"},
  {"en": "Algorithm", "ur": "الگورتھم", "category": "ai"},
  {"en": "Sensor", "ur": "حسّاس", "category": "hardware"}
]
```

---

## 15. References

- [Google Translate API Docs](https://cloud.google.com/translate)
- [Docusaurus i18n](https://docusaurus.io/docs/i18n/introduction)
- [RTL Best Practices](https://rtlstyling.com/)
- [Urdu Unicode](https://unicode.org/charts/PDF/U0600.pdf)

---

**Last Updated:** 2025-12-28
