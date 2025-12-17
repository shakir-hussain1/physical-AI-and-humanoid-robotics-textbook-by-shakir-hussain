---
id: 003
title: RAG chatbot implementation plan
stage: plan
date: 2025-12-16
surface: agent
model: claude-haiku-4-5-20251001
feature: 1-rag-chatbot-backend
branch: 1-rag-chatbot-backend
user: shakir-hussain
command: /sp.plan
labels: ["rag", "backend", "implementation-plan", "architecture", "data-model", "api-design"]
links:
  spec: specs/1-rag-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-rag-chatbot-backend/plan.md
  - specs/1-rag-chatbot-backend/research.md
  - specs/1-rag-chatbot-backend/data-model.md
  - specs/1-rag-chatbot-backend/contracts/openapi.yaml
  - specs/1-rag-chatbot-backend/quickstart.md
tests:
  - none
---

## Prompt

Physical AI and Humanoid Robotics — RAG Chatbot Backend

You are a senior backend architect and AI systems planner.

Your task is to create a detailed EXECUTION PLAN for the RAG chatbot backend, strictly governed by the approved /sp.constitution and /sp.specify documents.

PLAN OBJECTIVE: Produce a clear, auditable, and phased execution plan for building a production-grade academic RAG chatbot backend that integrates with a Physical AI & Humanoid Robotics book platform.

The plan must be suitable for direct task decomposition in Claude Code.

## Response snapshot

**Implementation Plan Complete: Phase 0 & Phase 1 Delivered**

**Deliverables (5 documents)**:

1. **Implementation Plan (plan.md)** [12 sections]
   - Technical Context: Python 3.11, FastAPI, PostgreSQL, Qdrant, Cohere Embed, Claude 3.5 Sonnet
   - Constitution Check: ✅ PASS on all 6 principles
   - Project Structure: Backend + tests documentation/source layout
   - Phase 0: Research tasks (9 decision areas)
   - Phase 1: Design (data model, API contracts, quickstart, agent context)
   - Phase 2 (planned): Task decomposition (~50–57 tasks)
   - Complexity Justification: Trade-offs documented (PostgreSQL for audit, Cohere for RAG quality, Claude 3.5 for instruction-following)

2. **Research & Technical Decisions (research.md)** [9 decisions]
   - Decision 1: Retrieval Ranking (Hybrid 85% Cohere + 15% BM25; confidence >0.60)
   - Decision 2: Citation Format (Custom academic: [Source: Section (Page X) - excerpt])
   - Decision 3: Confidence Scoring (Two-component: 60% retrieval + 40% generation)
   - Decision 4: Timeout Handling (Hard 5-second → graceful rejection)
   - Decision 5: Prompt Injection (Whitelist validation, literal search, no jailbreak risk)
   - Decision 6: Content Chunking (512 tokens, 20% overlap, sentence boundaries)
   - Decision 7: Audit Logs (Denormalized PostgreSQL, 1-year retention)
   - Decision 8: Multi-Tenancy (Single book + trusted API client token auth)
   - Decision 9: LLM Prompt (Explicit RAG constraints + few-shot examples)
   - All decisions include rationale, alternatives considered, implementation details

3. **Data Model (data-model.md)** [7 entities + DDL]
   - Document: Book chapter/section metadata (document_id, chapter_id, title, text, page_range, metadata)
   - Chunk: 512-token atomic units (chunk_id, text, token_count, embedding_vector, status)
   - Query: User question + metadata (query_id, user_query_text, source_mode, selected_passage_id)
   - Answer: Generated response + provenance (answer_id, answer_text, source_chunks[], confidence_score, citations[])
   - Citation: Source references (citation_id, chunk_id, excerpt, page_number, section_heading)
   - AuditLog: Complete operation trace (denormalized, monthly partitioned, 1-year retention)
   - FactCheckGrade: Domain expert review (grade_id, accuracy_score, hallucination_detected, approved_for_production)
   - Full SQL DDL provided (CREATE TABLE statements with indexes, constraints, partitioning)
   - State machines: Query (submitted → processing → answered), Answer (pending_review → approved)
   - Performance considerations: Table sizes, query patterns, index strategies

4. **API Contracts (openapi.yaml)** [OpenAPI 3.0]
   - 4 REST endpoints:
     - POST /chat/query: Full-book semantic search (Bearer token auth)
     - POST /chat/context-restricted: Passage-restricted answer mode
     - POST /content/ingest: Book chapter upload + chunking + indexing
     - GET /health: Liveness + readiness checks
   - Request/response schemas with examples
   - Error responses with codes (INVALID_QUERY, TIMEOUT, LOW_CONFIDENCE, etc.)
   - Security: Bearer token authentication
   - Full machine-readable contract for code generation

5. **Quickstart Guide (quickstart.md)** [12 sections]
   - Local setup: Docker Compose (FastAPI + PostgreSQL + Qdrant)
   - Environment config: API keys, database URLs
   - Database schema: Table creation
   - Sample content indexing: Upload chapters via API
   - Making queries: cURL, Python, JavaScript examples
   - Context-restricted mode: User passage selection
   - Testing: Unit, integration, contract tests with pytest
   - Logging: Container logs for debugging
   - Fact-checking workflow: SQL queries for sampling
   - Deployment checklist: Pre-production verification (20+ items)
   - Troubleshooting: Common errors + fixes
   - Next steps: Implementation roadmap

**Constitutional Compliance**: ✅ VERIFIED

| Principle | Compliance | Evidence |
|-----------|-----------|----------|
| I. Knowledge Source Verification | ✅ PASS | Retrieval precedes generation; citation mandatory (FR-004); source tracking in all responses (entities: Citation, AuditLog) |
| II. Zero Hallucination Guarantee | ✅ PASS | Confidence thresholds (<0.70 rejected); retrieval-first architecture; out-of-scope detection; research decision 3 + FR-005 |
| III. Academic Integrity | ✅ PASS | [Source ID / Page / Section] citation format (research decision 2); no paraphrasing without source |
| IV. Test-First Validation | ✅ PASS | pytest unit + integration + contract tests; ≥80% coverage target; grading gate (FactCheckGrade entity) |
| V. FastAPI + Vector DB | ✅ PASS | FastAPI app, Qdrant retrieval, PostgreSQL audit logs, 5-second timeout (FR-011) |
| VI. Context-Restricted Answers | ✅ PASS | Two retrieval modes in API (query vs. context-restricted); passage-bounded answers enforced |

**Project Structure**:

```
backend/
├── src/ (to be created in Phase 2)
│   ├── main.py, config.py
│   ├── models/ (Pydantic schemas, entities)
│   ├── services/ (retrieval, generation, citations, audit)
│   ├── api/ (FastAPI endpoints)
│   ├── db/ (PostgreSQL + Qdrant clients)
│   └── utils/ (validation, constants, tracing)
├── tests/ (Unit + Integration + Contract)
├── docker-compose.yml, Dockerfile, requirements.txt
└── specs/1-rag-chatbot-backend/
    ├── spec.md (feature specification)
    ├── plan.md (this plan)
    ├── research.md (9 technical decisions)
    ├── data-model.md (7 entities + DDL)
    ├── contracts/openapi.yaml (API contract)
    └── quickstart.md (local setup + deployment)
```

**Phase Status**:
- ✅ Phase 0: Research (9 decisions documented)
- ✅ Phase 1: Design (data model, API contracts, quickstart)
- ⏳ Phase 2: Task decomposition (/sp.tasks; ~50–57 tasks)
- ⏳ Phase 3: Implementation & testing (/sp.implement)

**Estimated Effort**:
- Phase 2 task breakdown: 50–57 tasks (setup, data model, services, API, testing, deployment)
- MVP timeline: 4–6 weeks for full implementation + fact-checking review cycle

**Next Command**: `/sp.tasks` to decompose into actionable task cards

## Outcome

- ✅ Impact: Complete architectural blueprint for RAG chatbot backend; all technical decisions resolved; ready for task decomposition and implementation
- 🧪 Tests: Integration testing covered in Phase 2 (tasks); unit/contract test stubs in quickstart
- 📁 Files:
  - specs/1-rag-chatbot-backend/plan.md (implementation plan)
  - specs/1-rag-chatbot-backend/research.md (9 research findings)
  - specs/1-rag-chatbot-backend/data-model.md (7 entities + DDL)
  - specs/1-rag-chatbot-backend/contracts/openapi.yaml (API spec)
  - specs/1-rag-chatbot-backend/quickstart.md (setup guide)
- 🔁 Next prompts: `/sp.tasks` (task decomposition), `/sp.implement` (execution)
- 🧠 Reflection: Plan locked in all constitutional requirements, architectural patterns, and data structures. No ambiguity remains. Ready for developers to implement tasks autonomously. Research decisions provide clear rationale for all technical choices.

## Evaluation notes (flywheel)

- Failure modes observed: None (planning is deterministic given clarified spec)
- Graders run and results (PASS/FAIL): Constitution check 6/6 PASS; all 9 research decisions documented with rationale
- Prompt variant (if applicable): None
- Next experiment: Validate task decomposition alignment with Phase 1 design; confirm each task traces to at least one entity, API endpoint, or research decision
