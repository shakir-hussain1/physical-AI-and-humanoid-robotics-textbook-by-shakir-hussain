---
id: 001
title: Update RAG Chatbot Backend Spec with Tech Stack
stage: spec
date: '2025-12-25'
model: claude-haiku-4-5-20251001
feature: 1-rag-chatbot-backend
branch: 1-rag-chatbot-backend
user: Claude
command: Analyze and update spec.md to align with unified book project requirements (OpenAI Agents, Neon Postgres, text selection)
labels:
  - rag-chatbot
  - openai-agents
  - neon-postgres
  - qdrant-cloud
  - text-selection
  - specification
links:
  SPEC: specs/1-rag-chatbot-backend/spec.md
  TICKET: null
  ADR: null
  PR: null
files:
  - specs/1-rag-chatbot-backend/spec.md
tests: []
---

# Update RAG Chatbot Backend Spec with Correct Tech Stack

## Summary
Updated the RAG Chatbot Backend specification (specs/1-rag-chatbot-backend/spec.md) to correctly reflect the unified Physical AI & Humanoid Robotics book project architecture. Replaced outdated technology references (Cohere API → OpenAI Agents/ChatKit SDKs, generic PostgreSQL → Neon Serverless Postgres) and added missing text selection query feature.

## Changes Made

### 1. Updated Header Metadata
- Updated **Feature Input** to explicitly reference: OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier
- Added **Last Updated** date (2025-12-25)
- Changed from generic "RAG Chatbot Backend" to complete platform specification

### 2. Added User Story 2: Text Selection Queries (Priority P1)
- **Feature:** Students can select text passages from the textbook and ask questions specific to that selection
- **Scope Control:** System ensures answers reference only selected text (no extraneous references)
- **Implementation Detail:** Uses full-text retrieval for small selections (<50 words), vector search for larger selections
- **Acceptance Criteria:** 4 BDD scenarios covering paragraph selection, code preservation, scope boundaries, follow-up context
- **Rationale:** Enables targeted learning on dense sections without full-chapter context

### 3. Renumbered User Stories (Consistency)
- User Story 2 (old) → User Story 3 (Translating Chapters to Urdu)
- User Story 3 (old) → User Story 4 (Managing User Progress and Preferences)
- User Story 4 (old) → User Story 5 (Administrator Monitoring System Health)

### 4. Enhanced Functional Requirements

**FR1 Update (RAG Query System)**
- Clarified: "Generate answers grounded in retrieved content using **OpenAI Agents/ChatKit SDKs**" (explicit LLM provider)

**FR1.5 Addition (Text Selection Query System)** - NEW REQUIREMENT
- Accept user-selected text passages with full passage + position metadata
- Query scoping: process only selected text (no broader context)
- Retrieval strategy: full-text for <50 words, vector search for larger selections
- Grounding guarantee: answers exclusively reference selected text
- History: maintain selection query history per user
- Context awareness: follow-ups respect selection scope boundaries

**FR5 Update (Database & Caching)**
- Specified: "**Neon Serverless Postgres**" instead of generic PostgreSQL
- Added: "(Neon handles serverless scaling)" for clarity on operational characteristics

### 5. Data Entities Enhancement

**New Entities Added:**
- **UserSelectedTexts:** id, user_id, chapter_id, selected_text (full passage), text_hash, start_position, end_position, timestamp, expires_at
- **SelectionQueries:** id, user_id, selection_id, question, answer, confidence_score, sources, created_at

**Rationale:** Necessary to track user text selections and associated queries for the text-selection feature and analytics.

### 6. Dependencies Section Rewrite

**External APIs (Corrected):**
- ❌ Removed: Cohere API
- ✅ Added: **OpenAI API (via Agents/ChatKit SDK)** for LLM reasoning, embeddings, answer generation (primary dependency)
- ✅ Specified: **Neon Serverless Postgres** for user/translation cache/conversation storage (required)
- ✅ Confirmed: **Qdrant Cloud (Free Tier)** for vector embeddings and semantic search (required)

**Technology Stack (Enhanced):**
- Added: **Alembic** for database migrations
- Added: **OpenAI Agents SDK / ChatKit SDK** explicitly for LLM integration
- Added: **psycopg[binary]** for Neon Postgres connections
- Added: **qdrant-client** for vector database interactions
- Updated: BeautifulSoup description now mentions "HTML parsing and preservation"
- Updated: JWT library fully specified as **PyJWT**

### 7. Success Criteria Updates

Added two new success criteria:
- ✅ "Students can select text passages and ask questions scoped to only that selection"
- ✅ "Text selection queries return answers grounded exclusively in selected text with confidence >0.85"

Updated existing criteria with specifics:
- Changed "RAG Query System" to "Students can ask questions...with source citations (**OpenAI Agents/ChatKit-powered**)"
- Changed "System maintains >99.5% uptime" to "...with automated backups **via Neon**"
- Expanded performance target clarity: "RAG <3s, translations <5s cached"

### 8. Edge Cases & Closing Statement

**Edge Case Fix:**
- Changed: "What happens if **Cohere API** is down?" → "What happens if **external translation API** is down?"
- Maintains validity of edge case without technology-specific references

**Updated Closing Statement:**
- Now references: Constitution **v1.2.0 (ratified 2025-12-25)**
- Emphasizes: alignment with PRINCIPLE_9 (RAG Chatbot Integration)
- Clarifies: specification governs unified learning platform (Docusaurus + RAG + hardware guidance)

## Alignment with Requirements

✅ **OpenAI Agents/ChatKit SDKs**: Referenced in header, FR1, Dependencies
✅ **Neon Serverless Postgres**: Specified in FR5, Dependencies, Success Criteria
✅ **Qdrant Cloud Free Tier**: Confirmed in Dependencies, FR1.5, NFR2
✅ **Text Selection Support**: New User Story 2 + FR1.5 + Data Entities
✅ **FastAPI**: Already correct (Python 3.11+, FastAPI framework)
✅ **Constitutional Alignment**: References v1.2.0 PRINCIPLE_9

## No Breaking Changes

- ✅ All existing user stories preserved (with renumbering)
- ✅ All existing functional/non-functional requirements intact
- ✅ Performance targets unchanged
- ✅ Security requirements unchanged
- ✅ Data privacy policies unchanged
- ✅ Out-of-scope boundaries unchanged
- ✅ Addition is purely additive (new feature + corrected tech stack)

## Validation Checklist

✅ No unresolved placeholders or TODOs
✅ All user stories (1-5) properly numbered and described
✅ All functional requirements (FR1-FR7 + FR1.5) internally consistent
✅ All non-functional requirements (NFR1-NFR5) still applicable
✅ Data entities fully specified with exact field names
✅ Dependencies match actual implementation requirements
✅ Success criteria measurable and aligned with acceptance scenarios
✅ Constitutional alignment explicitly stated
✅ No technology conflicts (OpenAI vs Cohere resolved)
✅ Database choice (Neon) consistent throughout

## Outcome

Specification now accurately reflects the unified Physical AI & Humanoid Robotics platform with:
1. Correct LLM provider (OpenAI Agents/ChatKit, not Cohere)
2. Correct database (Neon Serverless, not generic PostgreSQL)
3. Complete text selection query feature (User Story 2 + FR1.5 + data model)
4. All technology dependencies explicitly named and justified
5. Measurable success criteria for all features
6. Zero conflicts or outdated references

The spec is **production-ready** and **fully aligned** with the constitution v1.2.0 and user requirements.

---
PROMPT_TEXT: "Analyze the current RAG chatbot backend spec (specs/1-rag-chatbot-backend/spec.md) and check if it correctly uses: OpenAI Agents/ChatKit SDKs (not Cohere), Neon Serverless Postgres (not generic PostgreSQL), Qdrant Cloud Free Tier, and text selection query capability (answering questions based only on text selected by the user). If gaps exist, update the spec comprehensively without breaking anything."

RESPONSE_TEXT: "Analyzed spec and identified 3 major gaps: (1) LLM using Cohere instead of OpenAI Agents/ChatKit, (2) Database generic instead of Neon Serverless, (3) Text selection feature completely missing. Updated spec with: Header metadata, User Story 2 for text selection (P1), FR1.5 for text selection queries, corrected FR5 to specify Neon, added UserSelectedTexts + SelectionQueries entities, rewrote Dependencies section (OpenAI instead of Cohere, Neon instead of generic), updated Technology Stack with all necessary libraries, enhanced Success Criteria with text selection validation. All existing features preserved—only additive changes and corrections. Spec now production-ready and constitution-aligned (v1.2.0)."
