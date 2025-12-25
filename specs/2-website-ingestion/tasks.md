# Website URL Ingestion & Vector Storage - Implementation Tasks

## Version
**Tasks Version:** 1.0.0
**Created:** 2025-12-25
**Last Updated:** 2025-12-25

## Feature Overview
**Feature:** Website URL Ingestion & Vector Storage
**Module Alignment:** Unified Book RAG System (Backend Infrastructure)
**Plan Reference:** specs/2-website-ingestion/plan.md
**Specification Reference:** specs/2-website-ingestion/spec.md

## Constitutional Alignment
- PRINCIPLE_1: Technical Excellence and Accuracy
- PRINCIPLE_4: Practical Application
- PRINCIPLE_9: RAG Chatbot Integration
- PRINCIPLE_10: Deployment and Platform Accessibility

---

## Phase 1: Project Setup & Dependencies

**Objective:** Initialize backend project and configure environment

- [x] T001 Initialize backend project with UV and create project structure
  - **File:** `backend/pyproject.toml`, `backend/.env.example`
  - **Dependencies:** None
  - **Acceptance Criteria:**
    - `pyproject.toml` defines all dependencies (requests, beautifulsoup4, cohere, qdrant-client, tiktoken, python-dotenv)
    - `.env.example` documents required environment variables
    - `.gitignore` excludes `.env` and `logs/` directories
    - Virtual environment created and verified with `uv venv`
  - **Success Metric:** `uv pip install -e .` succeeds without errors

- [x] T002 Create logging configuration and directory structure
  - **File:** `backend/logs/.gitkeep`
  - **Dependencies:** T001
  - **Acceptance Criteria:**
    - `logs/` directory exists and is writable
    - `.gitkeep` allows directory tracking without log files
  - **Success Metric:** Directory created and verified with `ls -la backend/logs/`

- [x] T003 Create configuration loader from environment variables
  - **File:** `backend/main.py` (IngestionConfig class)
  - **Dependencies:** T001
  - **Acceptance Criteria:**
    - IngestionConfig class validates all required env vars (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
    - Defaults provided for optional values (max_retries=3, timeout=30)
    - Validation fails with clear error message if required vars missing
  - **Success Metric:** `IngestionConfig().validate()` returns True when valid .env loaded

---

## Phase 2: Core Components (Parallelizable)

**Objective:** Implement individual service classes (can be done in parallel)

- [x] T004 [P] Implement WebCrawler class for URL content extraction
  - **File:** `backend/main.py` (WebCrawler class)
  - **Dependencies:** T001, T003
  - **Acceptance Criteria:**
    - `fetch_url(url)` returns (text, title) tuple or (None, None) on failure
    - BeautifulSoup extracts text from main/article/body tags
    - HTTP errors and timeouts logged as warnings, not exceptions
    - Returns None for empty or unparseable content
  - **Success Metric:** Successfully fetch 3 sample URLs and extract text > 100 chars

- [x] T005 [P] Implement SemanticChunker class for text splitting
  - **File:** `backend/main.py` (SemanticChunker class)
  - **Dependencies:** T001
  - **Acceptance Criteria:**
    - `chunk_text(text, url, title)` returns list of chunk dicts with text, tokens, chunk_index, metadata
    - Splits on paragraph boundaries first, respects max 1024 tokens per chunk
    - Each chunk includes metadata (url, page_title)
    - Token count computed with tiktoken, no mid-sentence breaks
  - **Success Metric:** Chunk a 5KB text sample into 5-10 chunks, verify tokens <= 1024 each

- [x] T006 [P] Implement EmbeddingService class with retry logic
  - **File:** `backend/main.py` (EmbeddingService class)
  - **Dependencies:** T001, T003
  - **Acceptance Criteria:**
    - `embed_chunks(chunks)` returns list of 1024-dimensional embedding vectors
    - Batch API calls (max 100 chunks per call)
    - Exponential backoff retries on failure (1s, 2s, 4s max)
    - Returns None after max 3 retries, logs all attempts
  - **Success Metric:** Generate embeddings for 50 sample chunks with Cohere API (uses real API key)

- [x] T007 [P] Implement QdrantStorage class with vector persistence
  - **File:** `backend/main.py` (QdrantStorage class)
  - **Dependencies:** T001, T003
  - **Acceptance Criteria:**
    - `create_collection(name)` creates cosine-distance collection if not exists
    - `store_vectors(collection_name, embeddings, metadata)` upserts points with payload
    - `verify_vectors(collection_name, sample_size)` queries sample of vectors and validates retrieval
    - Handles quota limits gracefully with informative error messages
  - **Success Metric:** Store 50 test vectors and retrieve 100% by ID from Qdrant Cloud

---

## Phase 3: Pipeline Orchestration & Main Entry Point

**Objective:** Integrate all components into executable pipeline

- [x] T008 Implement main() function orchestrating full pipeline
  - **File:** `backend/main.py` (main function)
  - **Dependencies:** T001-T007
  - **Acceptance Criteria:**
    - `main()` returns 0 on success, 1 on failure
    - Loads config and validates all env vars at startup
    - Iterates over hardcoded URL list (editable)
    - For each URL: fetch → chunk → embed → store → log
    - Logs summary (total URLs, chunks, embeddings, duration)
    - All errors caught and logged, no unhandled exceptions
  - **Success Metric:** Run `python main.py` end-to-end with 3+ URLs, produces summary log

- [x] T009 Add script entry point and CLI interface
  - **File:** `backend/main.py` (__name__ == "__main__" block)
  - **Dependencies:** T008
  - **Acceptance Criteria:**
    - `if __name__ == "__main__": exit(main())`
    - Can run directly: `python main.py`
    - Exit code reflects success/failure
  - **Success Metric:** `python main.py && echo "Success"` works correctly

---

## Phase 4: Validation & Testing

**Objective:** Verify pipeline functionality with real data

- [x] T010 Test with real Docusaurus URLs and validate chunking quality
  - **File:** `backend/main.py` (test URLs list)
  - **Dependencies:** T008
  - **Acceptance Criteria:**
    - Successfully ingest 5+ real Docusaurus documentation URLs
    - Verify chunks have no mid-sentence breaks (manual spot check 10%)
    - Verify token counts <= 1024 for all chunks
    - Log shows 100% success rate
  - **Success Metric:** All 5+ URLs processed, chunks stored, logs show zero extraction failures

- [x] T011 Verify vector storage and retrieval accuracy
  - **File:** `backend/main.py` (QdrantStorage.verify_vectors)
  - **Dependencies:** T010
  - **Acceptance Criteria:**
    - Query random sample of 20 stored vectors by ID
    - Verify 100% retrieval success rate
    - Verify metadata (url, page_title) matches original chunks
    - Verify embedding dimensions are 1024
  - **Success Metric:** `verify_vectors("book_embeddings", 20)` returns True

- [x] T012 Test error handling for all failure modes
  - **File:** `backend/main.py` (test with invalid URLs)
  - **Dependencies:** T001-T009
  - **Acceptance Criteria:**
    - Invalid URLs (syntax, unreachable) logged as failures, pipeline continues
    - Cohere API failure triggers retries, then graceful failure
    - Qdrant quota exceeded handled with clear error message
    - All errors logged at ERROR level with context
  - **Success Metric:** Run with 1 invalid URL mixed with 3 valid ones, pipeline completes with 3 successes logged

---

## Phase 5: Documentation & Cleanup

**Objective:** Complete setup documentation and finalize MVP

- [x] T013 Document environment setup and configuration in README
  - **File:** `backend/README.md`
  - **Dependencies:** T001
  - **Acceptance Criteria:**
    - Step-by-step setup instructions (UV, venv, dependencies)
    - Environment variable documentation (.env.example reference)
    - Example URLs and customization instructions
    - Troubleshooting section for common failures
  - **Success Metric:** New user can follow README and run pipeline successfully

- [x] T014 Add inline code documentation and type hints
  - **File:** `backend/main.py` (all classes and functions)
  - **Dependencies:** T008
  - **Acceptance Criteria:**
    - All classes have docstrings explaining purpose and interface
    - All functions have docstrings with Args/Returns/Raises
    - All function parameters have type hints
    - All return values have type hints
  - **Success Metric:** `python -m pydoc backend.main` renders properly formatted docs

- [x] T015 Verify .gitignore and no secrets in version control
  - **File:** `backend/.gitignore`
  - **Dependencies:** T001, T003
  - **Acceptance Criteria:**
    - `.env` and `logs/` in .gitignore
    - `.gitignore` present and committed
    - No API keys in `main.py` or any committed files
    - `git status` shows clean working directory (no .env tracked)
  - **Success Metric:** `git log --name-only` shows no .env files ever committed

---

## Phase 6: Integration Validation

**Objective:** Confirm pipeline meets all specification requirements

- [x] T016 Validate pipeline meets all functional requirements
  - **File:** Review logs from T010
  - **Dependencies:** T010, T011
  - **Acceptance Criteria:**
    - REQ-1: URLs crawled and text extracted ✓
    - REQ-2: Content chunked (max 1024 tokens) ✓
    - REQ-3: Embeddings generated with Cohere ✓
    - REQ-4: Embeddings stored in Qdrant with metadata ✓
    - REQ-5: Ingestion status tracked in logs ✓
    - REQ-6: Vectors verified after storage ✓
    - REQ-7: Batch ingestion (multiple URLs) works ✓
    - REQ-8: Configuration via .env ✓
  - **Success Metric:** All 8 REQs verified and documented in summary

- [x] T017 Validate pipeline meets non-functional requirements
  - **File:** Measure performance from logs in T010
  - **Dependencies:** T010
  - **Acceptance Criteria:**
    - NFR-1: 20KB chapter processes in < 30 seconds (log timestamps)
    - NFR-2: Retries with exponential backoff work on simulated failure
    - NFR-3: Qdrant storage succeeds on Free Tier quota
    - NFR-4: All operations logged at appropriate levels (INFO/ERROR)
  - **Success Metric:** All 4 NFRs verified with log timestamps and metrics

- [x] T018 Create final summary and deployment checklist
  - **File:** `backend/DEPLOYMENT.md`
  - **Dependencies:** T013-T017
  - **Acceptance Criteria:**
    - Summarize what was built and verified
    - List all files created (main.py, .env.example, README.md, etc.)
    - Deployment checklist (set .env vars, test with `python main.py`)
    - Known limitations and Phase 2 roadmap
  - **Success Metric:** Deployment guide sufficient for new user to setup and run MVP

---

## Task Dependencies Graph

```
T001 (Project Setup)
  ├─→ T002 (Logging)
  ├─→ T003 (Config)
  │    ├─→ T004 (WebCrawler)
  │    ├─→ T006 (EmbeddingService)
  │    └─→ T007 (QdrantStorage)
  ├─→ T005 (SemanticChunker)
  │
  └─→ T004 + T005 + T006 + T007 (all parallelizable)
       └─→ T008 (Main orchestration)
            ├─→ T009 (CLI entry)
            ├─→ T010 (Real URL testing)
            │    ├─→ T011 (Vector validation)
            │    ├─→ T012 (Error handling)
            │    └─→ T016 (Functional req validation)
            ├─→ T013 (README)
            ├─→ T014 (Docs)
            ├─→ T015 (Security check)
            └─→ T017 (NFR validation)
                 └─→ T018 (Deployment guide)
```

## Parallel Execution Strategy

**Optimal Parallelization (after T001-T003):**

**Parallel Batch 1** (independent, can run simultaneously):
- T004 (WebCrawler) + T005 (SemanticChunker) + T006 (EmbeddingService) + T007 (QdrantStorage)
- **Estimated time:** 2-4 hours per component = 4 hours wall-clock (parallel)
- **Output:** 4 working service classes

**Sequential Batch 2** (depends on Batch 1):
- T008 (Integration) → T009 (CLI) → T010 (Testing)
- **Estimated time:** 2 hours

**Parallel Batch 3** (after T010):
- T011 (Verification) + T012 (Error testing) + T013 (README) + T014 (Docs) + T015 (Security)
- **Estimated time:** 1-2 hours wall-clock (parallel)

**Sequential Batch 4**:
- T016 + T017 + T018 (Validation & summary)
- **Estimated time:** 1 hour

**Total Wall-Clock Time (with parallelization):** ~11 hours
**Total Sequential Time (without parallelization):** ~20+ hours

---

## MVP Scope (Minimum Viable Product)

**Required for MVP (Tier 1):**
- ✅ T001-T009: Project setup + all 5 components + main orchestration
- ✅ T010: Real URL testing with 3-5 URLs
- ✅ T011: Vector retrieval validation
- ✅ T013: Basic README

**Nice-to-Have (Tier 2):**
- T012: Extensive error testing
- T014: Full documentation
- T017: NFR validation with metrics

**Post-MVP (Phase 2):**
- FastAPI endpoints
- Postgres audit logging
- Async batch processing

---

## Quality Assurance

### Technical Validation
- [ ] All 8 functional requirements met (verified in T016)
- [ ] All 4 non-functional requirements met (verified in T017)
- [ ] No unhandled exceptions in production path
- [ ] All API calls properly error-handled with retries

### Code Quality
- [ ] Type hints on all functions
- [ ] Docstrings on all classes and public methods
- [ ] Logging at appropriate levels (INFO, WARNING, ERROR)
- [ ] No hardcoded secrets or credentials

### Testing Coverage
- [ ] Happy path: fetch → chunk → embed → store (verified in T010)
- [ ] Error path: invalid URL, API failure, quota exceeded (verified in T012)
- [ ] Data integrity: vectors queryable by ID with correct metadata (verified in T011)

---

## Success Criteria

### Completion Checklist
- [ ] All 18 tasks completed and verified
- [ ] `python main.py` runs end-to-end without errors
- [ ] 5+ real URLs ingested, vectors stored in Qdrant
- [ ] 100% vector retrieval accuracy validated
- [ ] README provides clear setup and usage instructions
- [ ] No .env or secrets in version control

### MVP Definition
- ✅ Single `main.py` file with all components
- ✅ Processes multiple URLs in batch
- ✅ Generates and stores embeddings reliably
- ✅ Clear error handling and logging
- ✅ Configuration via environment variables
- ✅ Verified with real Docusaurus URLs

---

## References
- **Plan:** specs/2-website-ingestion/plan.md
- **Specification:** specs/2-website-ingestion/spec.md
- **Data Model:** specs/2-website-ingestion/data-model.md
- **API Contracts:** specs/2-website-ingestion/contracts/openapi.yaml
- **Quick Start:** specs/2-website-ingestion/quickstart.md
- **Constitution:** .specify/memory/constitution.md

---

*These tasks establish the Phase 1 MVP for Website URL Ingestion & Vector Storage, with clear parallelization opportunities and well-defined acceptance criteria for each component.*
