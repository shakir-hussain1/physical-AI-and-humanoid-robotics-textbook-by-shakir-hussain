# Implementation Plan: RAG Chatbot Backend

**Branch**: `1-rag-chatbot-backend` | **Date**: 2025-12-16 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/1-rag-chatbot-backend/spec.md`

## Summary

Build a production-grade Retrieval-Augmented Generation (RAG) chatbot backend for the Physical AI & Humanoid Robotics academic book platform. The system must answer student/educator/researcher queries **strictly grounded in book content** with zero hallucination tolerance, full citation coverage, and complete audit trails for fact-checking. Architecture: FastAPI (Python) backend with Claude 3.5 Sonnet LLM, Qdrant vector database, Cohere Embed embeddings, and PostgreSQL for audit logs. Three retrieval modes: full-book semantic search, context-restricted passage mode, and out-of-scope rejection.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI, Pydantic, Claude SDK (Anthropic), Qdrant SDK, Cohere API, psycopg2 (PostgreSQL driver)
**Storage**: PostgreSQL (audit logs, metadata), Qdrant (vector embeddings), file-based (book content ingest)
**Testing**: pytest (unit + integration), coverage.py (≥80% target)
**Target Platform**: Linux server (Docker containerized)
**Project Type**: REST API backend (single project)
**Performance Goals**: <2s avg latency, p95 <3s for 100 concurrent users; ≥90% retrieval relevance; ≥95% factual accuracy
**Constraints**: 5-second query timeout (hard limit), 1000-char query length, <5% hallucination rate trigger
**Scale/Scope**: ≤500 concurrent users (MVP), 20–30 answers/month for fact-checking review, weekly batch indexing of new book content

## Constitution Check

**GATE: All checks MUST PASS before Phase 0. Re-evaluate after Phase 1 design.**

| Principle | Requirement | Plan Compliance | Status |
|-----------|-------------|-----------------|--------|
| I. Knowledge Source Verification | Every answer verifiable against stored sources | Retrieval precedes generation; citation injection mandatory; source chunk tracking in all responses | ✅ **PASS** |
| II. Zero Hallucination Guarantee | Retrieval-first, confidence thresholds, citation-required, out-of-scope detection | Confidence scoring <0.70 triggers "low confidence" flag; retrieval failure returns explicit out-of-scope rejection; no external knowledge injection | ✅ **PASS** |
| III. Academic Integrity & Citations | [Source ID / Page / Section] format, no paraphrasing without source | APA-style citations embedded in response; source excerpts included; citation validation in response schema | ✅ **PASS** |
| IV. Test-First Answer Validation | Unit tests for retrieval, integration tests end-to-end, manual domain expert review, grading gate | Phase 1 includes test strategy; Phase 2 (tasks) will define grading + manual review SOP; ≥80% coverage target | ✅ **PASS** |
| V. FastAPI + Vector DB Architecture | FastAPI endpoints, vector DB retrieval, source verification, structured logging, 5s timeout | FastAPI app structure defined; Qdrant retrieval pipeline designed; logging strategy (structured JSON) defined; timeout enforced per FR-011 | ✅ **PASS** |
| VI. Context-Restricted Answers | Passage selection mode bypasses vector search; answers bounded to selected text | Two retrieval modes: full-book (vector search) and passage-restricted (direct text analysis); mode switching in API contract | ✅ **PASS** |

**Gate Result**: ✅ **PASS** - Plan fully compliant with all 6 constitutional principles.

## Project Structure

### Documentation (this feature)

```text
specs/1-rag-chatbot-backend/
├── spec.md                  # Feature specification
├── plan.md                  # This file (implementation plan)
├── research.md              # Phase 0: Research findings + decision rationale (TO BE CREATED)
├── data-model.md            # Phase 1: Entity definitions + schemas (TO BE CREATED)
├── quickstart.md            # Phase 1: Developer quickstart + deployment guide (TO BE CREATED)
├── contracts/               # Phase 1: API contracts
│   ├── openapi.yaml         # OpenAPI 3.0 spec for all endpoints
│   ├── query-endpoint.md    # POST /chat/query contract + examples
│   ├── context-endpoint.md  # POST /chat/context-restricted contract
│   └── content-ingest.md    # POST /content/ingest contract
└── checklists/
    └── requirements.md      # Quality checklist (completed)
```

### Source Code Structure

```text
backend/
├── src/
│   ├── main.py              # FastAPI app entrypoint
│   ├── config.py            # Configuration (env vars, settings)
│   ├── models/              # Pydantic data models
│   │   ├── schemas.py       # Request/response schemas
│   │   ├── entities.py      # Domain entities (Document, Chunk, Query, Answer, etc.)
│   │   └── audit.py         # AuditLog + FactCheckGrade models
│   ├── services/            # Business logic
│   │   ├── retrieval.py     # Vector search + chunk ranking
│   │   ├── generation.py    # Claude LLM integration + response formatting
│   │   ├── citations.py     # Citation injection + validation
│   │   ├── content_ingest.py# Book parsing + chunking + embedding
│   │   └── audit_logger.py  # Structured logging + traceability
│   ├── api/                 # REST endpoints
│   │   ├── query.py         # POST /chat/query + GET /chat/{query_id}
│   │   ├── context.py       # POST /chat/context-restricted
│   │   ├── content.py       # POST /content/ingest
│   │   └── health.py        # GET /health (liveness + readiness)
│   ├── db/                  # Database + vector DB clients
│   │   ├── postgres.py      # PostgreSQL connection + audit log repository
│   │   ├── qdrant_client.py # Qdrant vector search client
│   │   └── migrations/      # SQL migration scripts
│   └── utils/
│       ├── validation.py    # Query + input validation (prompt injection prevention)
│       ├── constants.py     # Model names, thresholds, error messages
│       └── tracing.py       # Request tracing + correlation IDs
├── tests/
│   ├── unit/                # Unit tests (retrieval, generation, validation)
│   │   ├── test_retrieval.py
│   │   ├── test_generation.py
│   │   ├── test_citations.py
│   │   └── test_validation.py
│   ├── integration/         # Integration tests (end-to-end RAG flow)
│   │   ├── test_query_flow.py
│   │   ├── test_context_flow.py
│   │   └── test_fact_checking.py
│   ├── contract/            # API contract tests
│   │   └── test_endpoints.py
│   └── fixtures/            # Test data + mock responses
│       ├── sample_chunks.py
│       ├── sample_queries.py
│       └── mock_llm.py
├── Dockerfile               # Production image (Python 3.11 slim + dependencies)
├── docker-compose.yml       # Local dev: FastAPI + PostgreSQL + Qdrant
├── requirements.txt         # Python dependencies
└── .env.example             # Environment variables template
```

## Phase 0: Research & Technical Decisions

**Prerequisites**: Constitution Check ✅ PASS, Specification complete

**Deliverables**: `research.md` with all decisions documented

### Research Tasks

| Decision Area | Research Task | Expected Output |
|---------------|---------------|-----------------|
| **Retrieval Ranking** | How to rank retrieved chunks by relevance + confidence? Cohere Embed similarity score + BM25 hybrid? | Ranking formula + thresholds (default: top 5 chunks, confidence >0.60) |
| **Citation Rendering** | How to format citations in responses? APA, MLA, or custom? | Citation template + example rendering |
| **Confidence Calibration** | How to compute confidence score? Embedding similarity + Claude's self-assessment? | Confidence formula + validation strategy |
| **Timeout & Graceful Degradation** | How to handle 5-second timeout? Partial results or full reject? | Timeout handling strategy + error messages |
| **Prompt Injection Prevention** | What prompt injection patterns to detect? | Query validation rules + blocked patterns |
| **Content Chunking Strategy** | Chunk size (512 tokens), overlap strategy, semantic boundaries? | Chunking algorithm + parameters |
| **Audit Log Retention** | PostgreSQL schema for audit logs? Query + answer + retrieval context? | Schema design + retention policy |
| **Multi-tenancy / Access Control** | Single book or multiple books? Per-user permissions? | Access control model (default: trusted Docusaurus plugin only) |

**Phase 0 Output**: `research.md` containing decision + rationale + alternatives for each area.

---

## Phase 1: Design & Contracts

**Prerequisites**: `research.md` complete

### 1.1 Data Model Design (`data-model.md`)

Extract entities from spec + research decisions:

**Core Entities**:

| Entity | Purpose | Key Fields | Relationships |
|--------|---------|-----------|----------------|
| **Document** | Indexed book chapter/section | chapter_id, section_id, title, page_range, text, created_at | ← Chunk (1:M) |
| **Chunk** | Atomic searchable unit (≤512 tokens) | chunk_id, parent_document_id, text, start_page, end_page, embedding_vector, embedding_model | ← Document, → Chunk |
| **Query** | User question + metadata | query_id, user_query_text, source_mode (full-book \| context-restricted), selected_passage_id, submitted_at | → Answer (1:1) |
| **Answer** | Generated response + provenance | answer_id, answer_text, source_chunks[], confidence_score, citations[], generated_at, llm_model, audit_trail_id | ← Query, → Citation, → AuditLog |
| **Citation** | Reference to source chunk | citation_id, chunk_id, excerpt, page_number, section_heading, source_confidence | ← Answer |
| **AuditLog** | Complete operation trace | log_id, query_id, answer_id, retrieval_query, chunks_retrieved_count, retrieval_latency_ms, llm_prompt, llm_response, success, error_message, timestamp | ← Query, ← Answer |
| **FactCheckGrade** | Manual domain expert review | grade_id, answer_id, reviewer_id, accuracy_score (0-100%), hallucination_detected, comments, approved_for_production, reviewed_at | ← Answer |

**Validation Rules**:
- Answer.confidence_score: 0.0–1.0 (float)
- Citation.page_number: positive integer
- Query.user_query_text: 1–1000 characters
- Chunk.text: ≤512 tokens (enforce in ingest service)

**State Transitions**:
- Query: submitted → processing → answered (or error)
- Answer: generated → pending_review → approved (or rejected)

### 1.2 API Contracts (`contracts/`)

**Endpoint 1: POST /chat/query**

```yaml
Summary: Submit a natural language query to search full book content
Request:
  Body: application/json
    - query (string, 1-1000 chars): User question
    - mode (string): "full-book" (default) or "context-restricted"
    - conversation_id (string, optional): For future multi-turn support
Response:
  200 OK: application/json
    - answer_id (string): Unique response ID
    - answer (string): Generated response grounded in citations
    - sources (array):
        - id (string): Chunk ID
        - excerpt (string): Relevant passage
        - page (integer): Page number
        - section (string): Section heading
    - confidence (float): 0.0-1.0 confidence score
    - status (string): "success" | "partial" | "out_of_scope"
  400 Bad Request: Invalid query format
  504 Gateway Timeout: Query exceeded 5 second limit
```

**Endpoint 2: POST /chat/context-restricted**

```yaml
Summary: Submit query restricted to user-selected passage
Request:
  Body: application/json
    - query (string): User question
    - selected_passage (string): User-highlighted text
Response:
  200 OK: application/json
    (same as /chat/query, plus "source_mode": "context-restricted")
  400 Bad Request: Query unanswerable from passage
```

**Endpoint 3: POST /content/ingest**

```yaml
Summary: Upload and index book chapter(s)
Request:
  Body: multipart/form-data
    - file (file): Markdown or plaintext chapter
    - chapter_id (string): Unique chapter identifier
    - metadata (json): author, publication_date, etc.
Response:
  202 Accepted: Ingest queued
    - ingest_id (string): Job ID for status polling
  200 OK (sync mode): Ingest complete
    - chunks_created (integer): Number of chunks indexed
```

**Endpoint 4: GET /health**

```yaml
Summary: Liveness + readiness checks
Response:
  200 OK: application/json
    - status (string): "healthy" | "degraded"
    - postgres (bool): DB connectivity
    - qdrant (bool): Vector DB connectivity
    - claude_api (bool): LLM API reachable
```

**Error Response Format** (all endpoints):

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable description",
    "details": "Optional context"
  }
}
```

### 1.3 Development Quickstart (`quickstart.md`)

**Content**:
- Local setup (Docker Compose): PostgreSQL + Qdrant + FastAPI
- Environment variables (.env template)
- Running tests (pytest)
- Indexing sample book content
- Making first query via curl/Python
- Deployment checklist (Docker build, Kubernetes, environment secrets)

### 1.4 Agent Context Update

Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude` to register:
- Technology choices: Python 3.11, FastAPI, Qdrant, PostgreSQL, Claude 3.5 Sonnet, Cohere Embed
- Key patterns: RAG retrieval pipeline, citation injection, audit logging
- File structure: `backend/src/` layout

---

## Phase 2: Task Decomposition

**Prerequisites**: Phase 1 complete (data-model.md, contracts/*, quickstart.md)

**Deliverable**: `/sp.tasks` generates `tasks.md` with testable, independently deployable task cards

**Expected Task Categories** (rough breakdown):

| Category | Estimated Tasks | Priority |
|----------|-----------------|----------|
| Setup & Configuration | 3–4 | P0 (first) |
| Data Model & DB Schema | 4–5 | P0 |
| Content Ingest Pipeline | 5–6 | P1 |
| Retrieval Service (Vector Search) | 5–6 | P1 |
| Generation Service (Claude Integration) | 4–5 | P1 |
| Citation Service | 3–4 | P1 |
| REST API Endpoints | 4–5 | P1 |
| Audit Logging & Tracing | 3–4 | P2 |
| Validation & Error Handling | 3–4 | P2 |
| Testing (Unit + Integration) | 6–8 | P1 (ongoing) |
| Fact-Checking Grading Interface | 2–3 | P2 |
| Deployment & Monitoring | 3–4 | P3 |
| **Total** | **~50–57 tasks** | — |

---

## Complexity Justification

**Constitutional Violations**: None detected.

**Design Trade-offs**:

| Decision | Why Chosen | Trade-off |
|----------|-----------|-----------|
| Separate Citation Service | Centralize citation logic for consistency & audit | Extra microservice layer; simpler testing trade-off: maintainability > performance |
| PostgreSQL for Audit Logs | Queryable, durable, ACID compliance for legal audit trail | Slower than in-memory; justified by constitutional audit requirement |
| Batch Indexing (weekly) | Predictable, controlled updates; prevents mid-session knowledge shifts | Delayed content updates; acceptable for MVP (addressed in scope) |
| Claude 3.5 Sonnet (not Haiku) | Superior instruction-following for zero-hallucination requirement | Higher API cost; justified by constitutional academic integrity mandate |
| Cohere Embed (not open-source) | Specialized for RAG; better accuracy for academic text | Vendor lock-in + per-request cost; justified by SC-001 (≥90% retrieval relevance) |

---

## Next Steps

1. **Stakeholder Review**: Present plan to book author/editor for approval
2. **Phase 0 Execution**: Run research tasks; consolidate findings in `research.md`
3. **Phase 1 Execution**: Generate `data-model.md`, `contracts/`, `quickstart.md`; run agent context update
4. **Phase 2 (Next Command)**: Run `/sp.tasks` to decompose into implementable task cards
5. **Implementation**: Execute tasks in order (setup → data model → services → API → testing)

---

**Approval Status**: ⏳ Pending stakeholder review

**Created**: 2025-12-16
**Last Updated**: 2025-12-16
