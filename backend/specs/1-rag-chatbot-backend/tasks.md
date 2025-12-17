---
description: "Implementation tasks for RAG Chatbot Backend"
---

# Tasks: RAG Chatbot Backend

**Input**: Design documents from `/specs/1-rag-chatbot-backend/`
**Prerequisites**: plan.md (implementation plan), spec.md (feature spec), research.md (decisions), data-model.md (schema), contracts/openapi.yaml (API spec)

**Organization**: Tasks grouped by development phase and user story (US1, US2, US3) for independent implementation and testing.

**Total Tasks**: ~60 tasks across 6 phases
**MVP Scope**: Phase 1-2 (Setup + Foundational) + Phase 3 (User Story 1) = ~25 tasks

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and FastAPI application structure

**Completion Criteria**: All foundational files created, dependencies installed, project structure ready for development

- [ ] T001 Create project root directory structure per plan.md (src/, tests/, specs/, contracts/) in backend/
- [ ] T002 [P] Initialize requirements.txt with core dependencies: FastAPI, Pydantic, psycopg2, qdrant-client, anthropic, cohere, pytest, python-dotenv in backend/
- [ ] T003 [P] Create .env.example with configuration template (ANTHROPIC_API_KEY, COHERE_API_KEY, DATABASE_URL, QDRANT_URL, FASTAPI_ENV, LOG_LEVEL) in backend/
- [ ] T004 [P] Create Dockerfile for production image (Python 3.11 slim base, pip install -r requirements.txt) in backend/
- [ ] T005 [P] Create docker-compose.yml with FastAPI + PostgreSQL + Qdrant services for local development in backend/
- [ ] T006 Create src/main.py as FastAPI application entrypoint with GET /health endpoint in backend/src/
- [ ] T007 Create src/config.py for environment variable management (Pydantic BaseSettings, API keys, database URLs, timeouts) in backend/src/
- [ ] T008 [P] Setup pre-commit hooks and linting configuration (.flake8, pyproject.toml) in backend/
- [ ] T009 Initialize empty pytest configuration (pytest.ini or pyproject.toml [tool.pytest.ini_options]) in backend/

**Checkpoint**: Project structure ready - all files created, dependencies installable, FastAPI app runs locally

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until Phase 2 is complete

**Completion Criteria**: Database schema initialized, all services skeleton created, error handling & logging configured, validation framework ready

### Database & Persistence

- [ ] T010 Create database migration system using Alembic in backend/src/db/migrations/
- [ ] T011 Create PostgreSQL schema DDL from data-model.md in backend/src/db/migrations/versions/001_initial_schema.sql (Document, Chunk, Query, Answer, Citation, AuditLog, FactCheckGrade tables)
- [ ] T012 Create src/db/postgres.py with PostgreSQL connection pool, session management, transaction context manager in backend/
- [ ] T013 [P] Create src/db/qdrant_client.py with Qdrant vector database client (similarity search, vector upsert, delete operations) in backend/
- [ ] T014 Run database migrations against local PostgreSQL (docker-compose exec postgres) in backend/

### Models & Schemas

- [ ] T015 Create src/models/schemas.py with Pydantic request/response schemas (QueryRequest, QueryResponse, ContextRestrictedQueryRequest, ContextRestrictedQueryResponse, IngestRequest, HealthResponse, ErrorResponse) in backend/
- [ ] T016 [P] Create src/models/entities.py with SQLAlchemy ORM models (DocumentORM, ChunkORM, QueryORM, AnswerORM, CitationORM, AuditLogORM, FactCheckGradeORM) in backend/
- [ ] T017 [P] Create src/models/audit.py with AuditLog Pydantic model (for fast JSON serialization, independent of ORM) in backend/

### Services (Skeleton)

- [ ] T018 Create src/services/__init__.py (empty marker file) in backend/
- [ ] T019 [P] Create src/services/retrieval.py with skeleton: class RetrieverService (stub methods for retrieve_chunks, rank_by_confidence) in backend/
- [ ] T020 [P] Create src/services/generation.py with skeleton: class GenerationService (stub methods for generate_answer, estimate_confidence) in backend/
- [ ] T021 [P] Create src/services/citations.py with skeleton: class CitationService (stub methods for inject_citations, validate_sources) in backend/
- [ ] T022 [P] Create src/services/content_ingest.py with skeleton: class ContentIngestService (stub methods for parse_document, chunk_text, embed_chunks, index_in_qdrant) in backend/
- [ ] T023 [P] Create src/services/audit_logger.py with skeleton: class AuditLoggerService (stub method for log_operation) in backend/

### Validation & Error Handling

- [ ] T024 Create src/utils/validation.py with input validation functions: validate_query_length, validate_query_format, validate_utf8, detect_prompt_injection_patterns in backend/
- [ ] T025 Create src/utils/constants.py with application constants: QUERY_MIN_LENGTH, QUERY_MAX_LENGTH, CONFIDENCE_THRESHOLD, RETRIEVAL_TOP_K, TIMEOUT_SECONDS, MIN_TOKEN_COUNT, MAX_TOKEN_COUNT, COHERE_EMBED_MODEL, CLAUDE_MODEL, ERROR_CODES in backend/
- [ ] T026 [P] Create src/utils/exceptions.py with custom exception classes: QueryValidationError, RetrieverError, GenerationError, TimeoutError, Hallucinations DetectedError, LowConfidenceError in backend/
- [ ] T027 Create src/utils/tracing.py with request context management: generate_trace_id, get_trace_id, set_trace_id functions in backend/

### Middleware & Logging

- [ ] T028 Create src/middleware/__init__.py (empty marker file) in backend/
- [ ] T029 Create src/middleware/error_handler.py with global exception handler middleware (catches custom exceptions, returns proper error responses) in backend/
- [ ] T030 Create src/middleware/logging_middleware.py with structured JSON logging middleware (logs request method, path, latency_ms, trace_id, status_code, error_code) in backend/
- [ ] T031 Create src/middleware/api_key_auth.py with Bearer token authentication (validates Authorization header, extracts API key hash) in backend/
- [ ] T032 Update src/main.py to include all middleware and global error handler registration in backend/src/

### API Router Structure

- [ ] T033 Create src/api/__init__.py (empty marker file) in backend/
- [ ] T034 [P] Create src/api/query.py with router skeleton: POST /chat/query endpoint (stub implementation) in backend/
- [ ] T035 [P] Create src/api/context.py with router skeleton: POST /chat/context-restricted endpoint (stub implementation) in backend/
- [ ] T036 [P] Create src/api/content.py with router skeleton: POST /content/ingest endpoint (stub implementation) in backend/
- [ ] T037 [P] Create src/api/health.py with router: GET /health endpoint (returns status, service health checks) in backend/
- [ ] T038 Update src/main.py to register all API routers (app.include_router for query, context, content, health) in backend/src/

**Checkpoint**: Foundation phase complete - all infrastructure ready; database initialized; can now proceed to user stories in parallel

---

## Phase 3: User Story 1 - Student Queries Academic Content (Priority: P1) 🎯 MVP

**Goal**: Enable students to ask natural language questions and receive factually accurate answers with citations grounded in book content.

**Independent Test**:
1. Index sample book chapter via POST /content/ingest
2. Submit student query via POST /chat/query
3. Verify answer includes citations with page numbers
4. Verify confidence score ≥0.85 for relevant queries
5. Verify out-of-scope rejection for unanswerable queries

### Content Ingestion for User Story 1

- [ ] T039 Implement src/services/content_ingest.py: parse_document function (reads Markdown/text, extracts chapter metadata, title, page ranges) in backend/
- [ ] T040 Implement src/services/content_ingest.py: chunk_text function (splits document into 512-token chunks with 20% overlap, respects sentence boundaries, validates token count) in backend/
- [ ] T041 [P] Implement src/services/content_ingest.py: embed_chunks function (calls Cohere API for each chunk, stores embedding_vector + embedding_confidence, handles rate limiting) in backend/
- [ ] T042 [P] Implement src/services/content_ingest.py: index_in_qdrant function (upserts chunk vectors to Qdrant, stores metadata: chunk_id, document_id, page_number, section_heading) in backend/
- [ ] T043 Implement POST /content/ingest endpoint in src/api/content.py (accepts multipart file + chapter_id, calls ContentIngestService, returns 202 queued or 200 completed) in backend/

### Retrieval for User Story 1

- [ ] T044 Implement src/services/retrieval.py: process_query function (validates query length, UTF-8, detects prompt injection, returns normalized query) in backend/
- [ ] T045 Implement src/services/retrieval.py: retrieve_chunks function (calls Qdrant similarity search using Cohere embeddings, returns top-5 chunks with similarity scores) in backend/
- [ ] T046 Implement src/services/retrieval.py: rank_by_confidence function (hybrid ranking: 85% Cohere similarity + 15% BM25 lexical match, applies confidence threshold >0.60, filters low-confidence results) in backend/
- [ ] T047 Implement src/services/retrieval.py: filter_by_confidence function (rejects chunks with confidence <0.60, raises LowConfidenceError for out-of-scope queries) in backend/

### Generation for User Story 1

- [ ] T048 Implement src/services/generation.py: build_system_prompt function (creates system prompt from constitution: "ONLY answer using provided excerpts, cite all claims, no external knowledge") in backend/
- [ ] T049 Implement src/services/generation.py: build_user_prompt function (combines retrieved chunks + user query in template: "Book excerpts: {chunks} \n User question: {query}") in backend/
- [ ] T050 Implement src/services/generation.py: generate_answer function (calls Claude API with system + user prompts, temperature=0.2 for academic tone, extracts answer + confidence estimate via regex) in backend/
- [ ] T051 Implement src/services/generation.py: estimate_confidence function (parses Claude's confidence estimate, computes final confidence = 0.6*retrieval + 0.4*generation) in backend/

### Citations for User Story 1

- [ ] T052 Implement src/services/citations.py: inject_citations function (inserts citations into answer text after each factual claim using format "[Source: Section (Page X) - excerpt]") in backend/
- [ ] T053 Implement src/services/citations.py: extract_source_chunks function (identifies which chunks were referenced in answer, extracts excerpts 30-50 words, validates page numbers) in backend/
- [ ] T054 Implement src/services/citations.py: validate_sources function (ensures every claim is cited, checks no external knowledge added, raises error if citations missing) in backend/
- [ ] T055 Implement src/services/citations.py: build_citation_objects function (creates Citation models with: id, chunk_id, excerpt, page_number, section_heading, source_confidence) in backend/

### Audit Logging for User Story 1

- [ ] T056 Implement src/services/audit_logger.py: log_query function (writes QueryORM record to PostgreSQL: user_query_text, source_mode, submitted_at, status) in backend/
- [ ] T057 Implement src/services/audit_logger.py: log_retrieval function (writes to AuditLog: retrieval_query, chunks_retrieved_count, chunks_returned JSON, retrieval_latency_ms) in backend/
- [ ] T058 Implement src/services/audit_logger.py: log_answer function (writes to AuditLog: llm_prompt, llm_response, answer_text, confidence_score, citations JSON, timestamp) in backend/

### API Endpoint for User Story 1

- [ ] T059 Implement POST /chat/query endpoint in src/api/query.py:
  - Parse QueryRequest (validate query length ≤1000)
  - Call RetrieverService to retrieve chunks (calls research decision #1: hybrid ranking)
  - Call GenerationService to generate answer (research decision #9: system prompt)
  - Call CitationService to inject citations (research decision #2: format)
  - Compute confidence (research decision #3: two-component)
  - Call AuditLoggerService to log operation
  - Return QueryResponse with answer + sources + confidence + status
  - Handle TimeoutError (5-second limit per FR-011)
  - Handle LowConfidenceError (confidence <0.60)
  - Return proper error responses (400, 504)
  in backend/src/api/query.py

### Error Handling for User Story 1

- [ ] T060 Add error handling in query endpoint for:
  - Empty query → 400 "Query must be 1-1000 characters"
  - Query too long → 400 "Query must be 1-1000 characters"
  - Invalid UTF-8 → 400 "Query contains invalid characters"
  - Timeout (>5 seconds) → 504 "Query processing timed out"
  - Low confidence (<0.60) → 200 with status "out_of_scope"
  - Retrieval failure → 200 with status "out_of_scope" + message "This question is not covered in the Physical AI textbook"
  in backend/src/api/query.py

**Checkpoint**: User Story 1 complete - students can ask questions, receive cited answers, out-of-scope queries handled correctly. Ready to test independently.

---

## Phase 4: User Story 2 - Educator Uses Chatbot for Fact-Checking (Priority: P1)

**Goal**: Enable educators to verify facts and find citations, with complete audit trails for fact-checking review.

**Independent Test**:
1. Submit educator fact-check query via POST /chat/query
2. Verify answer includes exact page numbers + section headings
3. Verify audit logs capture complete trace (query, retrieval, LLM prompts, chunks)
4. Query audit logs: SELECT * FROM audit_logs WHERE query_id = '...' (confirms full traceability)
5. Manually sample 5 answers, grade via fact_check_grades table

### Audit Trail Enhancement for User Story 2

- [ ] T061 Enhance src/services/audit_logger.py: log_full_trace function (writes comprehensive AuditLog with: user_query_text, retrieval_query, chunks_retrieved_count, chunks_returned JSONB, llm_prompt, llm_response, answer_text, confidence_score, citations JSONB, success, error_code, response_time_ms) in backend/
- [ ] T062 Implement database query functions in src/db/postgres.py: get_audit_logs_for_query, get_audit_logs_by_date_range, count_successful_answers, count_hallucinations (supports fact-checking review queries) in backend/
- [ ] T063 [P] Add citation metadata enrichment: in src/services/citations.py, ensure all Citation objects include (id, chunk_id, excerpt, page_number, section_heading, source_confidence) for audit trail in backend/

### Fact-Checking Grading Interface

- [ ] T064 Create src/models/grading.py with FactCheckGrade Pydantic model (grade_id, answer_id, reviewer_id, accuracy_score 0-100%, hallucination_detected bool, approved_for_production bool) in backend/
- [ ] T065 Create src/db/grading_repository.py with functions:
  - create_grade (inserts FactCheckGrade record)
  - get_answers_pending_review (SELECT * FROM answers WHERE status='pending_review' ORDER BY generated_at DESC LIMIT N)
  - get_answers_by_accuracy (SELECT * FROM fact_check_grades WHERE accuracy_score >= X)
  - update_answer_approval (sets approved_for_production based on accuracy ≥95%)
  in backend/

### Response Enhancement for Educator Queries

- [ ] T066 Enhance citation format in src/services/citations.py: ensure format includes exact section heading + page number for easy reference-finding: "[Source: [section_heading] (Page [page_number]) - \"[excerpt]\"]" in backend/
- [ ] T067 Add optional fields to QueryResponse schema: audit_trail_id (UUID linking to audit log for quick access) in backend/src/models/schemas.py

### Educator-Specific Error Handling

- [ ] T068 Enhance error messages in src/api/query.py for educator use cases:
  - When retrieval fails: Return message "Unable to find relevant content. Check if [topic] is covered in indexed chapters."
  - When confidence low: Return message "Found possible match but low confidence. Manual verification recommended."
  in backend/src/api/query.py

**Checkpoint**: User Story 2 complete - educators can see full audit trails, fact-check answers, manually grade responses. Audit logs fully queryable.

---

## Phase 5: User Story 3 - Researcher Uses Context-Restricted Passages (Priority: P2)

**Goal**: Enable researchers to select specific passages and ask questions restricted to that context only, no external knowledge.

**Independent Test**:
1. User selects 200-word passage from chapter
2. Submit via POST /chat/context-restricted with selected_passage
3. Verify answer only references selected passage (no external knowledge)
4. Verify answer marked "Based on your selected passage..."
5. Ask unanswerable question → system returns "Cannot be fully answered from selected passage"

### Retrieval Mode: Context-Restricted

- [ ] T069 Implement src/services/retrieval.py: analyze_passage_content function (takes user-selected passage text, extracts key terms, prepares for direct text analysis without vector search) in backend/
- [ ] T070 Implement src/services/retrieval.py: retrieve_from_passage function (searches only within selected passage text using keyword matching, returns results limited to passage bounds) in backend/

### Generation Mode: Passage-Bounded

- [ ] T071 Implement src/services/generation.py: build_passage_restricted_prompt function (modifies system prompt to: "ONLY answer using the provided passage. Do NOT use external knowledge. If unanswerable, say so clearly.") in backend/
- [ ] T072 Implement src/services/generation.py: validate_passage_bounded_answer function (checks that answer only references passage content, no external knowledge injected, raises error if violation detected) in backend/

### Citations Mode: Passage-Only

- [ ] T073 Implement src/services/citations.py: inject_passage_citations function (for context-restricted mode, citations reference selected passage text only, format: "[From selected passage: excerpt]") in backend/

### API Endpoint for User Story 3

- [ ] T074 Implement POST /chat/context-restricted endpoint in src/api/context.py:
  - Parse ContextRestrictedQueryRequest (query + selected_passage)
  - Call RetrieverService.retrieve_from_passage (search within passage only)
  - Call GenerationService.generate_answer with passage-restricted prompt
  - Call GenerationService.validate_passage_bounded_answer (ensure no external knowledge)
  - Call CitationService.inject_passage_citations (passage-only format)
  - Return ContextRestrictedQueryResponse with source_mode="context-restricted"
  - Handle cases where question unanswerable from passage → return 400 "Cannot be fully answered from selected passage"
  in backend/src/api/context.py

### Response Enhancement for Context-Restricted

- [ ] T075 Enhance ContextRestrictedQueryResponse in src/models/schemas.py: add source_mode="context-restricted" field, add header_text="Based on your selected passage..." in backend/

### Error Handling for User Story 3

- [ ] T076 Add error handling in src/api/context.py for:
  - Empty passage → 400 "Selected passage cannot be empty"
  - Passage too short (<50 words) → 400 "Selected passage too short to analyze"
  - Question unanswerable from passage → 400 "This question cannot be fully answered from your selected passage. Would you like to expand to the full chapter?"
  - External knowledge detected → 500 "Internal validation error: external knowledge detected in passage-restricted answer"
  in backend/src/api/context.py

**Checkpoint**: User Story 3 complete - researchers can restrict queries to passages, answers bounded correctly, no external knowledge leakage.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements, testing, documentation, and production readiness

**Completion Criteria**: Full test suite passes, documentation complete, ready for deployment

### Testing (Unit + Integration)

- [ ] T077 Create tests/unit/test_validation.py: unit tests for validation functions (validate_query_length, validate_utf8, prompt_injection_detection) in backend/
- [ ] T078 Create tests/unit/test_retrieval.py: unit tests for RetrieverService methods (chunk ranking, confidence filtering, timeout handling) in backend/
- [ ] T079 Create tests/unit/test_generation.py: unit tests for GenerationService (prompt building, confidence estimation) in backend/
- [ ] T080 Create tests/unit/test_citations.py: unit tests for CitationService (citation injection, format validation) in backend/
- [ ] T081 [P] Create tests/integration/test_query_flow.py: end-to-end test for User Story 1 (index chapter → query → verify answer + citations + confidence) in backend/
- [ ] T082 [P] Create tests/integration/test_context_flow.py: end-to-end test for User Story 3 (select passage → query → verify passage-bounded answer) in backend/
- [ ] T083 [P] Create tests/integration/test_fact_checking.py: end-to-end test for User Story 2 (query → verify audit log + grading) in backend/
- [ ] T084 Create tests/contract/test_endpoints.py: API contract tests validating all 4 endpoints against openapi.yaml (request/response schemas, status codes, error handling) in backend/
- [ ] T085 Run pytest with coverage target ≥80%: `pytest tests/ --cov=src --cov-report=html` in backend/

### Documentation

- [ ] T086 Update README.md with project overview, setup instructions, quick reference to spec.md + plan.md + quickstart.md in backend/
- [ ] T087 Create CONTRIBUTING.md with development guidelines, commit message format, testing requirements in backend/
- [ ] T088 Update quickstart.md with verified local setup steps (Docker Compose, sample indexing, test queries) in backend/

### Performance & Optimization

- [ ] T089 Profile retrieval latency: add timing logs in RetrieverService, target <2 seconds for full retrieval pipeline in backend/
- [ ] T090 Profile generation latency: add timing logs in GenerationService, target <2.5 seconds for Claude API call in backend/
- [ ] T091 Optimize database queries: add indexes on frequently queried columns (audit_logs.timestamp, answers.status, chunks.document_id) in backend/
- [ ] T092 Implement connection pooling in PostgreSQL client: configure pool_size + max_overflow for 100 concurrent connections in backend/

### Monitoring & Alerting Setup

- [ ] T093 Add prometheus metrics to src/utils/metrics.py: track latency (p95, p99), hallucination_rate, retrieval_precision, confidence_score distribution in backend/
- [ ] T094 Configure structured JSON logging: all logs should include trace_id, latency_ms, status_code, error_code for monitoring in backend/
- [ ] T095 Create monitoring dashboard configuration (Grafana/Prometheus) template in backend/monitoring/

### Deployment & Container

- [ ] T096 Verify Dockerfile builds successfully: `docker build -t rag-chatbot:latest .` in backend/
- [ ] T097 Verify docker-compose.yml runs all services locally: `docker-compose up -d && docker-compose ps` in backend/
- [ ] T098 Create .dockerignore to exclude unnecessary files (tests/, __pycache__, .env) in backend/
- [ ] T099 Add health check to docker-compose.yml: FastAPI container health check via GET /health in backend/

### Code Quality

- [ ] T100 Run linting checks: `flake8 src/ tests/` in backend/
- [ ] T101 Run type checking: `mypy src/` in backend/
- [ ] T102 [P] Format code: `black src/ tests/` in backend/
- [ ] T103 Review code for security issues: check for SQL injection, hardcoded secrets, insecure deserialization in backend/

### Production Readiness Checklist

- [ ] T104 Verify environment variables required for production (.env template includes all keys) in backend/
- [ ] T105 Configure rate limiting: add rate limiting middleware for API key + IP address (e.g., 100 queries/min per API key) in backend/src/
- [ ] T106 Add request ID generation: every request gets unique trace_id for end-to-end debugging in backend/
- [ ] T107 Configure CORS if needed: update FastAPI app with appropriate CORS origins in backend/src/main.py
- [ ] T108 Add health check endpoint validation in GET /health (verifies PostgreSQL, Qdrant, Claude API connectivity) in backend/
- [ ] T109 Run quickstart.md validation script: verify all setup steps work end-to-end in backend/

**Checkpoint**: All tasks complete - production-ready, tested, documented, deployable

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational) ← MUST complete before any user story
    ↓
Phase 3 (User Story 1) ┐
Phase 4 (User Story 2) ├─→ Can execute in parallel
Phase 5 (User Story 3) ┘
    ↓
Phase 6 (Polish)
```

### Critical Path (MVP)

**Minimum to ship User Story 1**:
1. Phase 1: Setup (T001-T009)
2. Phase 2: Foundational (T010-T038)
3. Phase 3: User Story 1 (T039-T060)
4. Phase 6: Testing (T077, T081) + Basic Deployment (T096-T099)

**Estimated effort**: 25 tasks, 2-3 weeks

### Full Implementation (All User Stories)

**All phases**: 109 tasks
1. Phase 1: Setup (9 tasks)
2. Phase 2: Foundational (29 tasks)
3. Phase 3: User Story 1 (22 tasks)
4. Phase 4: User Story 2 (8 tasks)
5. Phase 5: User Story 3 (8 tasks)
6. Phase 6: Polish (33 tasks)

**Estimated effort**: 109 tasks, 6-8 weeks

### Parallel Opportunities

**Setup Phase** (Phase 1):
- All [P] tasks (T002, T003, T004, T005, T008) can run in parallel

**Foundational Phase** (Phase 2):
- Database tasks (T010-T014) can run in parallel
- Model/Schema creation (T015-T017) can run in parallel
- Services skeleton (T019-T023) can run in parallel
- Validation/Constants (T024-T026) can run in parallel
- API routes (T034-T036) can run in parallel

**Once Foundational Complete**:
- User Story 1, 2, 3 can be implemented in parallel by different team members
- Within each story, test writing, model creation, service implementation can run in parallel

**Example Parallel Run** (with 3 developers):

```
Day 1: Team completes Phase 1-2 together
Day 2+:
  Developer A: Phase 3 (User Story 1)
  Developer B: Phase 4 (User Story 2)
  Developer C: Phase 5 (User Story 3)
Day N: Team converges for Phase 6 (Testing, Deployment)
```

---

## Implementation Strategy

### MVP First (Recommended)

1. ✅ Complete Phase 1: Setup
2. ✅ Complete Phase 2: Foundational (CRITICAL BLOCKER)
3. ✅ Complete Phase 3: User Story 1
4. 🧪 Test User Story 1 independently
5. ✅ Deploy Phase 6: Minimal testing + Docker setup
6. 🎯 **Ship MVP**: Students can ask questions, receive cited answers

### Incremental Delivery

After MVP:
1. Add Phase 4: User Story 2 (educators get audit trails)
2. Add Phase 5: User Story 3 (researchers get passage-restricted mode)
3. Complete Phase 6: Full testing, monitoring, optimization

### Single Developer Timeline

- **Week 1**: Phase 1 + Phase 2 (37 tasks)
- **Week 2-3**: Phase 3 + Phase 4 + Phase 5 (38 tasks)
- **Week 4**: Phase 6 (33 tasks) + final validation

---

## Checkpoints & Validation

### After Phase 2 Completion
```
✓ Docker-compose up -d runs successfully
✓ Database schema created
✓ All imports work (no missing modules)
✓ src/main.py starts without errors
✓ GET /health returns 200
```

### After Phase 3 Completion (MVP)
```
✓ POST /content/ingest accepts sample chapter
✓ POST /chat/query returns answer with citations
✓ Answer confidence ≥ 0.85 for relevant queries
✓ Out-of-scope queries return explicit rejection
✓ Audit logs capture complete trace
```

### After Phase 6 Completion
```
✓ pytest tests/ --cov=src reports ≥80% coverage
✓ All integration tests pass
✓ API contract tests match openapi.yaml
✓ Docker build succeeds
✓ docker-compose.yml runs all services
✓ Performance: p95 latency <3 seconds
✓ No hallucinations in sample answers
```

---

## Notes

- All [P] tasks can execute in parallel (different files, no interdependencies)
- Remove [P] from tasks sharing files or having dependencies
- Each phase must complete before next phase starts
- Each user story is independently testable after its phase completes
- Stop at any checkpoint and validate independently
- Commit after each task or logical group (e.g., all models in one commit)
- Tasks reference exact file paths for clarity
- Update this tasks.md if requirements change
