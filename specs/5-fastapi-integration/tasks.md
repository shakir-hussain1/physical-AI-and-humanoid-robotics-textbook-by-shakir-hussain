# RAG Spec-4: Backend–Frontend Integration via FastAPI - Task Decomposition

**Feature:** Backend–Frontend Integration via FastAPI
**Version:** 1.0.0
**Created:** 2025-12-26
**Branch:** 5-fastapi-integration

---

## Overview

This task decomposition breaks down the 7-phase implementation plan into 28 executable, independently testable tasks organized by user story. Each task is concise, has a clear file path, and includes acceptance criteria.

### Task Organization by User Story

- **US1 (P1):** FastAPI Application & Configuration
- **US2 (P1):** Request/Response Models & Validation
- **US3 (P1):** Query Endpoint Implementation
- **US4 (P2):** Context Query & Retrieval Endpoints
- **US5 (P2):** Health Check & Monitoring
- **US6 (P2):** Middleware & Error Handling
- **US7 (P3):** Integration Testing
- **US8 (P3):** Documentation & Deployment

---

## Phase 1: Setup & Foundations (Days 1-2)

**Goal:** Initialize FastAPI project structure and core configuration

### Setup Tasks (No story label)

- [x] T001 Create backend/src/api package structure with __init__.py, main.py, models.py, validators.py, errors.py
- [x] T002 Create backend/src/api/endpoints/ directory with __init__.py, query.py, retrieve.py, health.py
- [x] T003 Create backend/src/api/middleware/ directory with __init__.py and core middleware modules
- [x] T004 Create backend/tests/api/ directory for integration tests with __init__.py

### Foundational Tasks (No story label)

- [x] T005 Create .env.example with FastAPI configuration variables (API_PORT, CORS_ORIGINS, REQUEST_TIMEOUT, LOG_LEVEL)
- [x] T006 Create backend/src/config.py for environment-based configuration management with ConfigManager class
- [x] T007 Update backend/requirements.txt with FastAPI>=0.100.0, Pydantic>=2.0.0, python-dotenv>=1.0.0, uvicorn>=0.23.0
- [x] T008 [P] Create backend/src/utils/logging.py for structured JSON logging with request context tracking

---

## Phase 2: User Story 1 - FastAPI Application & Configuration (Days 1-2)

**Goal:** Initialize FastAPI application with proper lifespan management and dependency injection

**Test Criteria:** App starts without errors, dependencies initialized, config loaded from environment

### US1 Tasks

- [x] T009 [US1] Create backend/src/api/main.py with FastAPI app initialization, lifespan events, and exception handling
- [x] T010 [US1] Initialize RAG agent in lifespan events via dependency injection in backend/src/api/main.py
- [x] T011 [US1] Initialize retrieval service in lifespan events in backend/src/api/main.py
- [x] T012 [US1] Add CORS middleware configuration in backend/src/api/main.py with environment-based origins
- [x] T013 [US1] Add request/response logging middleware in backend/src/api/main.py with structured JSON format
- [x] T014 [US1] Create backend/src/api/__init__.py to export app and key dependencies

**Acceptance Criteria:**
- FastAPI app starts with `uvicorn backend.src.api.main:app --reload`
- Health check endpoint responds (before implementation) with 200 OK
- Environment variables loaded from .env
- CORS headers present in responses

---

## Phase 3: User Story 2 - Request/Response Models (Days 2-3)

**Goal:** Define Pydantic models for all API request/response schemas with validation

**Test Criteria:** Models validate input correctly, reject invalid data with clear messages, serialize to/from JSON

### US2 Tasks

- [x] T015 [US2] Create QueryRequest model in backend/src/api/models.py with query (str, max 10K), conversation_history (Optional[List]), user_role
- [x] T016 [US2] Create MessageModel in backend/src/api/models.py with role, content, timestamp fields
- [x] T017 [US2] Create ContextQueryRequest model in backend/src/api/models.py with selected_text, query, conversation_history
- [x] T018 [US2] Create RetrievalRequest model in backend/src/api/models.py with query, k (int, 1-20)
- [x] T019 [US2] Create SourceInfo model in backend/src/api/models.py with url, page_title, relevance_score, chunk_index
- [x] T020 [US2] Create QueryResponse model in backend/src/api/models.py with answer, sources, confidence, metadata, timestamp
- [x] T021 [US2] Create RetrievalResult model in backend/src/api/models.py with id, text, similarity_score, metadata
- [x] T022 [US2] Create HealthResponse model in backend/src/api/models.py with status, timestamp, components (Dict)
- [x] T023 [US2] Create ErrorResponse model in backend/src/api/models.py with error, message, request_id, timestamp
- [x] T024 [US2] Create validation functions in backend/src/api/validators.py with validate_query_length(), validate_history_format(), validate_k_range()

**Acceptance Criteria:**
- All models use Pydantic v2 syntax with validators
- Invalid input raises ValidationError with clear messages
- Models serialize to JSON correctly with ISO8601 timestamps
- Field constraints enforced (max 10K chars, k in [1-20], etc.)

---

## Phase 4: User Story 3 - Query Endpoint (Days 3-5)

**Goal:** Implement POST /api/query endpoint with agent integration

**Test Criteria:** Endpoint accepts valid queries, returns formatted answers with sources, handles errors gracefully

### US3 Tasks

- [x] T025 [US3] Create POST /api/query route in backend/src/api/endpoints/query.py that accepts QueryRequest
- [x] T026 [US3] Implement request validation in /api/query with query length checks and history format validation
- [x] T027 [US3] Integrate RAG agent call in /api/query: call agent.query(query_text, history, role)
- [x] T028 [US3] Implement response formatting in /api/query: convert AgentResponse to QueryResponse model
- [x] T029 [US3] Add latency tracking in /api/query: measure end-to-end time, include in response metadata
- [x] T030 [US3] Implement error handling in /api/query: catch agent timeouts (504), retrieval errors (503), LLM errors (502), log with request_id
- [ ] T031 [US3] Register /api/query route in backend/src/api/main.py with proper tags and response_model
- [x] T032 [P] [US3] Create POST /api/query/with-context route in backend/src/api/endpoints/query.py with ContextQueryRequest
- [x] T033 [US3] Implement context injection in /api/query/with-context: prepend selected_text to conversation context
- [x] T034 [US3] Add conversation history support in /api/query: pass history array to agent.query() method

**Acceptance Criteria:**
- POST /api/query accepts valid request, returns 200 OK with answer, sources, confidence
- Invalid query returns 400 Bad Request with clear error message
- Agent timeout returns 504 with "Request took too long" message
- Latency_ms included in metadata (< 6 seconds P95)
- Selected text properly integrated in /api/query/with-context
- Conversation history correctly passed to agent

---

## Phase 5: User Story 4 - Retrieval & Context Endpoints (Days 5-7)

**Goal:** Implement POST /api/retrieve and POST /api/query/with-context endpoints

**Test Criteria:** Retrieval returns raw search results, context endpoint integrates selected text, both handle errors

### US4 Tasks

- [x] T035 [US4] Create POST /api/retrieve route in backend/src/api/endpoints/retrieve.py with RetrievalRequest
- [x] T036 [US4] Integrate retrieval service in /api/retrieve: call retrieval.search(query_text, k=5)
- [x] T037 [US4] Format retrieval results in /api/retrieve: map to RetrievalResult models with similarity scores
- [x] T038 [US4] Implement retrieval error handling in /api/retrieve: 503 if service unavailable, 400 if k out of range
- [x] T039 [US4] Add latency tracking in /api/retrieve: measure search time (target < 1 second)
- [x] T040 [US4] Register /api/retrieve route in backend/src/api/main.py with proper tags

**Acceptance Criteria:**
- POST /api/retrieve returns list of RetrievalResult objects with ids, text, scores
- k parameter validates to [1-20] range, returns 400 if invalid
- Latency < 1 second for typical queries
- Retrieval unavailable returns 503 with retry guidance

---

## Phase 6: User Story 5 - Health Check & Monitoring (Days 7-8)

**Goal:** Implement GET /api/health endpoint with component status checks

**Test Criteria:** Health endpoint reports agent, retrieval, and LLM component status, fast response (< 100ms)

### US5 Tasks

- [x] T041 [US5] Create GET /api/health route in backend/src/api/endpoints/health.py returning HealthResponse
- [x] T042 [US5] Implement component health checks in /api/health: check agent readiness, retrieval availability, LLM connection
- [x] T043 [US5] Add health check caching in /api/health: cache results for 30 seconds to avoid overhead
- [x] T044 [US5] Implement health status logic: return "healthy" if all OK, "degraded" if some components down, "error" if critical failure
- [x] T045 [US5] Register /api/health route in backend/src/api/main.py with fast_response tag

**Acceptance Criteria:**
- GET /api/health always returns 200 OK (regardless of component status)
- Response includes status, timestamp, components dict
- Response time < 100ms (caching ensures this)
- Component statuses accurately reflect actual state

---

## Phase 7: User Story 6 - Middleware & Error Handling (Days 8-10)

**Goal:** Implement CORS, logging, request ID tracking, and error response middleware

**Test Criteria:** CORS headers present, errors logged with context, all errors return consistent format

### US6 Tasks

- [ ] T046 [US6] Create CORS middleware in backend/src/api/middleware/__init__.py with environment-based origin configuration
- [ ] T047 [US6] Create request ID middleware in backend/src/api/middleware/__init__.py: generate unique ID per request, add to context
- [ ] T048 [US6] Create error handler middleware in backend/src/api/middleware/__init__.py: catch exceptions, return ErrorResponse with request_id
- [ ] T049 [P] [US6] Create logging middleware in backend/src/api/middleware/__init__.py: log all requests/responses with timing
- [ ] T050 [US6] Register all middleware in backend/src/api/main.py in correct order (CORS first, then logging, then error handling)
- [ ] T051 [US6] Create error response formatter in backend/src/api/errors.py: HTTPException subclasses for each error type (ValidationError, TimeoutError, ServiceUnavailable)
- [ ] T052 [US6] Implement global exception handler in backend/src/api/main.py: catch unhandled exceptions, log, return 500 with request_id

**Acceptance Criteria:**
- CORS headers (Access-Control-Allow-Origin, etc.) present in all responses
- Request ID included in error responses and logs
- All errors return ErrorResponse with consistent format (error, message, request_id, timestamp)
- Structured JSON logs include timestamp, level, request_id, endpoint, latency
- Unhandled exceptions don't leak stack traces to client

---

## Phase 8: User Story 7 - Integration Testing (Days 10-12)

**Goal:** Write integration tests for all endpoints and error scenarios

**Test Criteria:** All endpoints tested, error cases covered, 10+ concurrent requests handled, conversation history works

### US7 Tasks

- [ ] T053 [US7] Create backend/tests/api/test_query_endpoint.py with tests for /api/query happy path, invalid input, timeout, agent error
- [ ] T054 [US7] Create backend/tests/api/test_context_query_endpoint.py with tests for /api/query/with-context with selected text
- [ ] T055 [US7] Create backend/tests/api/test_retrieval_endpoint.py with tests for /api/retrieve with various k values
- [ ] T056 [US7] Create backend/tests/api/test_health_endpoint.py with tests for /api/health component status
- [ ] T057 [US7] Create backend/tests/api/test_conversation_history.py with tests for multi-turn conversations, history preserved
- [ ] T058 [P] [US7] Create backend/tests/api/test_error_handling.py with tests for all error types (400, 502, 503, 504, 500)
- [ ] T059 [US7] Create backend/tests/api/test_cors.py with tests for CORS headers on all endpoints
- [ ] T060 [US7] Create backend/tests/api/test_concurrent_requests.py with load test for 10+ concurrent requests, no errors
- [ ] T061 [US7] Create backend/tests/api/conftest.py with pytest fixtures for FastAPI test client, mock agent, mock retrieval

**Acceptance Criteria:**
- All unit tests pass (pytest backend/tests/api/)
- Integration tests cover happy path and 8+ error scenarios
- Concurrent request test confirms no degradation with 10+ requests
- Conversation history properly maintained across requests
- CORS headers validated on all endpoints
- Code coverage > 85% for api/ module

---

## Phase 9: User Story 8 - Documentation & Deployment (Days 12-14)

**Goal:** Create deployment guide, API documentation, and startup procedures

**Test Criteria:** Documentation complete, deployment steps clear, production readiness checklist signed off

### US8 Tasks

- [ ] T062 [US8] Create backend/API_DOCUMENTATION.md with endpoint descriptions, request/response examples for all 4 endpoints
- [ ] T063 [US8] Create backend/DEPLOYMENT_GUIDE.md with environment variable reference, startup command, shutdown procedure
- [ ] T064 [US8] Create backend/STARTUP_CHECKLIST.md with pre-startup validation steps (config, dependencies, service availability)
- [ ] T065 [US8] Create backend/.env.example with all required environment variables and example values
- [ ] T066 [US8] Add startup validation in backend/src/api/main.py: check OPENAI_API_KEY, Qdrant connection, agent initialization
- [ ] T067 [US8] Implement graceful shutdown in backend/src/api/main.py: finish pending requests, log shutdown event
- [ ] T068 [US8] Create backend/PERFORMANCE_TARGETS.md documenting latency SLAs, uptime targets, capacity constraints
- [ ] T069 [US8] Create backend/tests/api/test_production_readiness.py with startup tests, config validation, graceful shutdown

**Acceptance Criteria:**
- API_DOCUMENTATION.md includes curl examples for all endpoints
- DEPLOYMENT_GUIDE.md contains step-by-step setup for local and production
- All environment variables documented with types and constraints
- Startup validation catches missing config before server starts
- Graceful shutdown allows 30 seconds to complete pending requests
- Production readiness tests all pass

---

## Dependencies and Execution Order

```
Phase 1 (Setup & Foundations) → Phase 2 (US1) → Phase 3 (US2) → Phase 4 (US3) → Phase 5 (US4) → Phase 6 (US5) → Phase 7 (US6) → Phase 8 (US7) → Phase 9 (US8)

Parallelization Opportunities:
- US3 & US4 can run in parallel after US2 (separate endpoints)
- US5 & US6 can run in parallel (health check independent of middleware)
- US7 test writing can start after US3 (tests first development possible)
```

---

## Execution Strategy

### MVP Scope (Recommended for Sprint 1)
Execute **Phases 1-5** (Days 1-7) for core API:
- Setup & initialization
- Pydantic models with validation
- Query endpoint with agent integration
- Retrieval endpoint with search integration

This delivers **functional MVP** with both query and search capabilities.

### Full Scope (Phases 1-9, Days 1-14)
Add **Phases 6-9** for production readiness:
- Middleware & error handling (robust error responses)
- Integration testing (comprehensive test coverage)
- Documentation & deployment (operational readiness)

### Parallelization Examples

**Sprint 1 (Days 1-7):**
- Developer A: T001-T008, T009-T014, T015-T024 (Setup, US1, US2)
- Developer B: T025-T034 (US3) in parallel with A after T014
- Developer C: T035-T040 (US4) in parallel with B after T014

**Sprint 2 (Days 7-14):**
- Developer A: T041-T052 (US5, US6)
- Developer B: T053-T061 (US7 tests)
- Developer C: T062-T069 (US8 documentation)

---

## Task Checklist Format Reference

Each task follows this format for clarity:

```
- [ ] [TaskID] [P?] [Story?] Description with file path
       ^          ^    ^       ^
       checkbox   ID   label   description + file
```

**Legend:**
- `[ ]` = unchecked, `[x]` = completed
- `[P]` = parallelizable (can run independently)
- `[US#]` = user story this task belongs to (phases 2+)
- File path always included for context

---

## Success Criteria Summary

| Phase | Goal | Key Deliverable | Acceptance |
|-------|------|-----------------|-----------|
| 1 | Setup | Project structure, dependencies | Runs without errors |
| 2 | US1 | FastAPI app, config, DI | App starts, loads config |
| 3 | US2 | Pydantic models | Validates input/output |
| 4 | US3 | Query endpoint | 200 OK with answer + sources |
| 5 | US4 | Retrieval + context | Raw search results + context |
| 6 | US5 | Health check | < 100ms response |
| 7 | US6 | Middleware + errors | CORS + error logging |
| 8 | US7 | Integration tests | Coverage > 85%, all scenarios |
| 9 | US8 | Docs + deployment | Production ready |

---

**Total Tasks:** 69 (28 core + 40 supporting)
**Estimated Timeline:** 14 days (2 weeks)
**MVP Timeline:** 7 days (Phases 1-5)

---

*Task decomposition complete. Ready for `/sp.implement` to execute all tasks.*
