---
ID: 005
TITLE: Database Schema Design
STAGE: plan
DATE_ISO: 2025-12-21
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: database-design
BRANCH: 1-rag-chatbot-backend
USER: user
COMMAND: Design PostgreSQL database schema
LABELS:
  - backend
  - database
  - postgresql
  - schema
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
---

## Prompt

Design comprehensive PostgreSQL database schema for textbook platform with user management, content, translations, and metadata.

## Response

### Database Schema

**Core Tables:**

1. **users**
   - id (Primary Key)
   - username (Unique)
   - email (Unique)
   - password_hash
   - created_at
   - updated_at

2. **documents**
   - id (Primary Key)
   - chapter_id
   - title
   - content
   - doc_metadata (JSON)
   - created_at
   - updated_at

3. **user_chapter_translations**
   - id (Primary Key)
   - user_id (FK: users.id)
   - chapter_id
   - target_language
   - original_content
   - translated_content
   - confidence_score (0.0-1.0)
   - created_at
   - updated_at
   - expires_at (30-day TTL)
   - Unique: (user_id, chapter_id, target_language)

4. **user_profiles**
   - id (Primary Key)
   - user_id (FK: users.id, Unique)
   - preferred_language
   - theme_preference
   - font_size
   - accessibility_options (JSON)
   - created_at
   - updated_at

5. **user_reading_progress**
   - id (Primary Key)
   - user_id (FK: users.id)
   - chapter_id
   - current_section
   - completion_percentage
   - last_read_at
   - study_time_minutes
   - created_at
   - updated_at
   - Unique: (user_id, chapter_id)

6. **user_bookmarks**
   - id (Primary Key)
   - user_id (FK: users.id)
   - chapter_id
   - section_id
   - note (Optional)
   - created_at
   - updated_at

7. **conversations**
   - id (Primary Key)
   - user_id (FK: users.id)
   - question
   - answer
   - context (JSON)
   - confidence_score
   - created_at

**Indexes:**
- user_id on all user-related tables
- chapter_id for fast chapter lookups
- created_at for time-based queries
- Composite: (user_id, chapter_id, target_language) for translations

**ORM Framework:** SQLAlchemy
**Migrations:** Alembic

---

## Outcome

✅ Database schema comprehensive and normalized
✅ Relationships properly defined
✅ Indexing strategy for performance
✅ Ready for Alembic migrations

**Status**: Schema ready for implementation

