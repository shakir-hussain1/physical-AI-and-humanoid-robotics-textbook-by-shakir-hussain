# RAG Spec-2: Data Retrieval & Pipeline Validation - Implementation Plan

## Version
**Plan Version:** 1.0.0
**Created:** 2025-12-26
**Last Updated:** 2025-12-26

## Feature Overview
**Feature:** Data Retrieval & Pipeline Validation
**Module Alignment:** Backend Infrastructure for RAG Chatbot System
**Dependencies:**
- Spec-1 (Website Ingestion) - textbook_embeddings collection in Qdrant Cloud
- Cohere API (same instance as Spec-1)
- Python 3.10+ environment with qdrant-client, cohere libraries

## Constitutional Alignment
This plan aligns with the following constitutional principles:
- PRINCIPLE_1: Technical Excellence and Accuracy
- PRINCIPLE_4: Practical Application
- PRINCIPLE_9: RAG Chatbot Integration and Knowledge Accessibility
- PRINCIPLE_10: Deployment and Platform Accessibility

## Scope and Requirements

### In Scope
- Implement retrieval service for querying textbook vectors in Qdrant
- Create test query suite (8-10 queries) with expected relevance patterns
- Implement metadata validation and chunk verification
- Generate comprehensive validation reports
- Structured logging for all retrieval operations
- End-to-end pipeline validation from storage to retrieval

### Out of Scope
- OpenAI Agent or ChatKit SDK integration
- Frontend UI or web components
- Advanced retrieval strategies (reranking, fusion, cross-encoding)
- Full benchmarking suite or load testing infrastructure
- Database schema changes to Qdrant collections

### External Dependencies
- **Qdrant Cloud:** Existing textbook_embeddings collection with 26 vectors (Spec-1)
- **Cohere API:** Same account and API key from Spec-1
- **Python Ecosystem:** qdrant-client, cohere, requests, python-dotenv libraries

## Key Decisions and Rationale

### Options Considered

**Option 1: Minimal Retrieval (Search-Only)**
- Description: Implement only similarity search without validation
- Trade-offs: Fast to implement but no confidence in data integrity; misses validation opportunities

**Option 2: Full Validation Suite (Selected)**
- Description: Implement retrieval + metadata validation + deterministic testing
- Trade-offs: More code but provides production-ready confidence; enables downstream integration

**Option 3: Cloud-Native Retrieval Service**
- Description: Build FastAPI server with async retrieval endpoints
- Trade-offs: Over-engineered for MVP; defers to Phase 2 (chatbot integration)

### Selected Approach
**Decision:** Full Validation Suite (Option 2) with synchronous retrieval service
**Rationale:**
- Balances scope and confidence - provides both functionality and validation
- Enables Phase 2 (chatbot integration) by proving storage works correctly
- Deterministic test queries ensure reproducible validation
- Manageable scope for single-phase implementation

**Trade-offs:**
- Synchronous implementation (async deferred to Phase 2)
- Local logging instead of centralized observability (sufficient for MVP)
- No authentication/authorization (single-user validation context)

### Principles
- **Measurable:** Success criteria include specific latency targets (< 500ms) and accuracy thresholds (80%+ relevance)
- **Reversible:** Retrieval service designed to be compatible with future async/FastAPI refactoring
- **Smallest Viable Change:** Only implement what's needed to validate Spec-1 and enable Spec-3 (chatbot)

## Implementation Strategy

### Phase 1: Foundation & Service Setup (Days 1-2)

**Objective:** Build core retrieval infrastructure and validation framework

- [ ] **P1.1:** Initialize retrieval service module structure
  - Create `backend/src/retrieval/` package
  - Create `backend/src/retrieval/qdrant_client.py` (Qdrant wrapper)
  - Create `backend/src/retrieval/embedding_service.py` (Cohere query embedding)
  - Create `backend/src/retrieval/validator.py` (metadata/content validation)
  - Setup `backend/src/retrieval/__init__.py` with public interfaces

- [ ] **P1.2:** Implement Qdrant Query Client
  - Wrap qdrant-client library with retrieval-specific methods
  - `search_similar(query_embedding: List[float], k: int = 5) -> List[Dict]`
  - `get_vectors_by_filter(url_filter: str = None, limit: int = 100) -> List[Dict]`
  - `get_collection_stats() -> Dict` (total vectors, vector dimensions, memory usage)
  - Implement error handling for connection failures and timeouts

- [ ] **P1.3:** Implement Embedding Generation for Queries
  - Reuse Cohere client from Spec-1 configuration
  - `embed_query_text(query: str) -> List[float]` - single query embedding
  - `embed_batch_queries(queries: List[str]) -> List[List[float]]` - batch embedding
  - Verify embedding dimensions match stored vectors (1024)
  - Handle edge cases (empty query, very long query)

- [ ] **P1.4:** Setup Configuration and Environment
  - Reuse .env configuration from Spec-1 (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
  - Create `backend/src/retrieval/config.py` with retrieval-specific settings
  - Implement validation at module load time
  - Document all configuration options

### Phase 2: Validation Engine & Test Suite (Days 3-5)

**Objective:** Build validation framework and define reproducible test queries

- [ ] **P2.1:** Implement Metadata Validation
  - Create `backend/src/retrieval/validator.py` with validation methods
  - `validate_metadata_completeness(result: Dict) -> Tuple[bool, List[str]]`
    - Checks: URL present, page_title present, chunk_index present and valid
    - Returns: (is_valid, error_messages)
  - `validate_metadata_consistency(metadata: Dict) -> Tuple[bool, List[str]]`
    - Checks: URL is valid format, chunk_index is integer >= 0, page_title is string
  - `validate_similarity_score(score: float) -> bool`
    - Checks: score in range [0, 1]

- [ ] **P2.2:** Implement Content Verification
  - `verify_url_accessible(url: str, timeout: int = 5) -> Tuple[bool, str]`
    - HTTP HEAD request to verify URL resolves
    - Returns: (is_accessible, status_or_error_message)
  - `verify_chunk_exists_in_page(url: str, chunk_index: int) -> Tuple[bool, str]`
    - Fetch page and verify chunk_index matches content structure
    - Returns: (is_valid, status_or_error_message)

- [ ] **P2.3:** Define Test Query Suite (8-10 queries)
  - Create `backend/src/retrieval/test_queries.py` with deterministic test data
  - Define structure: `TestQuery(text, expected_primary_domain, expected_top_keywords, expected_modules)`
  - Query categories:
    - **Single keywords:** "ROS2", "embedding", "gazebo", "simulation"
    - **Phrases:** "robot communication", "digital twin", "perception pipeline"
    - **Complex:** "how to implement behavior planning in humanoid robots"
    - **Edge cases:** "a", "123", "special@chars!", very long query (> 500 chars)
  - Document expected relevance (high/medium/low) for top-3 results
  - Ensure queries are reproducible and deterministic

- [ ] **P2.4:** Implement Retrieval Validation
  - `validate_query_results(query: str, results: List[Dict]) -> Dict`
    - Check results are ranked by similarity score (descending)
    - Verify all results have required fields (id, similarity_score, metadata)
    - Detect determinism issues (same query should produce identical results)
    - Returns: validation_report with pass/fail for each check

### Phase 3: Retrieval API & Report Generation (Days 6-8)

**Objective:** Build user-facing retrieval API and validation reporting

- [ ] **P3.1:** Implement Main Retrieval Service
  - Create `backend/src/retrieval/retrieval_service.py`
  - `class RetrievalService`:
    - `__init__(config)` - Initialize with Qdrant and Cohere clients
    - `search(query: str, k: int = 5) -> Dict` - Main retrieval method
      - Embed query text
      - Execute similarity search
      - Enrich with metadata
      - Return ranked results with similarity scores
    - `batch_search(queries: List[str], k: int = 5) -> List[Dict]` - Batch retrieval
    - `validate_retrieval(query: str) -> Dict` - Execute query and validate results
    - `get_stats() -> Dict` - Collection statistics and health

- [ ] **P3.2:** Implement Logging and Instrumentation
  - Create `backend/src/retrieval/logging.py` with structured logging
  - Log format: JSON with timestamp, operation, query, result_count, latency, status
  - `log_retrieval_operation(operation, query, result_count, latency, status)`
  - `log_validation_result(query, validation_report)`
  - Configure logging to file and console (separate handlers)
  - Setup log rotation (1 week retention for validation logs)

- [ ] **P3.3:** Implement Validation Report Generator
  - Create `backend/src/retrieval/report_generator.py`
  - `generate_validation_report(test_queries: List[TestQuery]) -> Dict`
    - Execute all test queries
    - Collect results and validation metrics
    - Calculate statistics: success rate, avg latency, accuracy %
    - Identify failures and issues
    - Returns: comprehensive report with pass/fail summary
  - `format_report_for_display(report: Dict) -> str`
    - Human-readable text report with section headings
    - Include sample results with relevance scores
    - List any detected issues or failures

- [ ] **P3.4:** Implement CLI Entry Point
  - Create `backend/src/retrieval/cli.py`
  - `run_validation()` - Execute full validation suite
  - `search(query: str, k: int = 5)` - Interactive search
  - `get_collection_info()` - Display collection statistics
  - `test_query_set(custom_query: str = None)` - Run specific test or custom query
  - Format output for developer convenience

### Phase 4: Testing & Documentation (Days 9-10)

**Objective:** Comprehensive testing, documentation, and production readiness

- [ ] **P4.1:** Implement Unit Tests
  - Create `backend/tests/retrieval/`
  - Test Embedding Service:
    - `test_embed_query_returns_1024_dims()` - verify dimensions
    - `test_embed_handles_empty_query()` - edge case
    - `test_embed_handles_special_chars()` - special characters
  - Test Validation:
    - `test_validate_metadata_detects_missing_url()` - metadata check
    - `test_validate_similarity_score_bounds()` - score validation
  - Test Retrieval:
    - `test_search_returns_top_k_results()` - basic retrieval
    - `test_search_results_ranked_by_score()` - ranking validation
    - `test_batch_search_returns_all_queries()` - batch operation

- [ ] **P4.2:** Implement Integration Tests
  - Create `backend/tests/integration/`
  - End-to-end retrieval workflow:
    - `test_end_to_end_query_to_result()` - full pipeline
    - `test_deterministic_retrieval()` - same query produces same results
    - `test_metadata_integrity()` - metadata matches ingested data
    - `test_error_handling_on_qdrant_failure()` - graceful degradation
  - Performance tests:
    - `test_single_query_latency_under_500ms()` - latency target
    - `test_batch_retrieval_under_5_seconds()` - batch performance

- [ ] **P4.3:** Validation with Test Query Suite
  - Create `backend/tests/validation/`
  - For each test query:
    - `test_query_<name>_returns_relevant_results()` - semantic relevance
    - Verify top-3 results are from expected domains
    - Verify similarity scores are in expected range
  - Overall validation:
    - `test_all_26_vectors_retrievable()` - no data loss
    - `test_metadata_completeness_100_percent()` - all results have metadata

- [ ] **P4.4:** Create Documentation
  - Create `backend/README_RETRIEVAL.md`:
    - Retrieval service overview and architecture
    - How to use RetrievalService class
    - Running validation suite
    - Understanding validation reports
    - Troubleshooting common issues
  - Create `backend/RETRIEVAL_TESTING.md`:
    - Test query definitions and expected outcomes
    - How to run individual tests
    - Performance benchmarks and targets
    - Debugging failing tests
  - Add inline documentation:
    - Docstrings on all public methods
    - Type hints on all parameters and returns
    - Comments on complex validation logic

### Phase 5: Production Validation (Day 11)

**Objective:** Execute full validation and document results

- [ ] **P5.1:** Execute Validation Suite
  - Run all unit tests: `pytest backend/tests/retrieval/`
  - Run all integration tests: `pytest backend/tests/integration/`
  - Run validation tests: `pytest backend/tests/validation/`
  - Document test results (pass/fail counts)

- [ ] **P5.2:** Execute Full Validation Report
  - Run `python -m backend.src.retrieval.cli run-validation`
  - Execute all 8-10 test queries
  - Generate comprehensive report
  - Document results in `backend/VALIDATION_RESULTS.txt`

- [ ] **P5.3:** Performance Benchmarking
  - Measure single query latency (average, p95, p99)
  - Measure batch retrieval latency for 10 vectors
  - Document results in `backend/PERFORMANCE_METRICS.txt`
  - Verify against NFR targets (< 500ms single, < 5s batch)

- [ ] **P5.4:** Final Verification
  - Verify all 26 vectors are retrievable
  - Verify metadata integrity (100% have required fields)
  - Verify URLs are still accessible
  - Verify retrieval is deterministic (repeated queries produce same results)
  - Document any issues or anomalies

## Interfaces and API Contracts

### Public Retrieval API

**Class: RetrievalService**

```python
class RetrievalService:
    def __init__(config: RetrievalConfig) -> None:
        """Initialize retrieval service with Qdrant and Cohere clients"""

    def search(query: str, k: int = 5) -> Dict:
        """
        Execute similarity search for query text

        Args:
            query: Query text (string, required)
            k: Number of top results to return (default 5, range 1-100)

        Returns:
            {
                "query": str,
                "results": [
                    {
                        "vector_id": str,
                        "similarity_score": float (0-1),
                        "metadata": {
                            "url": str,
                            "page_title": str,
                            "chunk_index": int
                        }
                    },
                    ...
                ],
                "total_results": int,
                "latency_ms": float,
                "status": "success" | "error"
            }
        """

    def batch_search(queries: List[str], k: int = 5) -> List[Dict]:
        """Execute similarity search for multiple queries"""

    def validate_retrieval(query: str) -> Dict:
        """Execute query and validate results against success criteria"""

    def get_stats() -> Dict:
        """
        Get collection statistics

        Returns:
            {
                "total_vectors": int,
                "vector_dimension": int,
                "collection_name": str,
                "vector_count": int,
                "memory_usage": str
            }
        """
```

### Error Handling

**Error Codes and HTTP Status Codes (for Phase 2 FastAPI):**

| Error Code | HTTP Status | Description |
|-----------|------------|-------------|
| `QUERY_EMPTY` | 400 | Query text is empty or whitespace |
| `QUERY_TOO_LONG` | 400 | Query exceeds max length (5000 chars) |
| `INVALID_K` | 400 | K value out of range (1-100) |
| `EMBEDDING_FAILED` | 500 | Failed to generate query embedding (Cohere API error) |
| `QDRANT_CONNECTION_ERROR` | 503 | Cannot connect to Qdrant Cloud |
| `QDRANT_TIMEOUT` | 504 | Qdrant query timed out (> 10 seconds) |
| `INVALID_RESULT` | 500 | Retrieved vector has invalid structure |
| `VALIDATION_FAILED` | 422 | Result validation detected issues |

### Versioning Strategy
- **API Versioning:** Single version for MVP (no versioning required)
- **Schema Versioning:** Metadata fields (url, page_title, chunk_index) must remain consistent
- **Backward Compatibility:** Must work with existing textbook_embeddings collection schema

### Implementation Requirements
- **Idempotency:** Search operations are idempotent (same query produces same results)
- **Timeouts:** Single query timeout: 10 seconds, Batch query timeout: 30 seconds
- **Retries:** Automatic retry on transient failures (max 3 retries with exponential backoff)
- **Connection Pooling:** Reuse Qdrant and Cohere client instances across requests

## Non-Functional Requirements (NFRs) and Budgets

### Performance
- **Single Query Latency:** < 500ms (p95), < 1 second (p99)
  - Includes: embedding generation + Qdrant search + metadata attachment
- **Batch Retrieval (10 queries):** < 5 seconds total
- **Collection Statistics:** < 100ms (should be cached)
- **Memory Usage:** < 256MB for retrieval service (excluding client libraries)

### Reliability
- **SLO:** 99% success rate for valid queries (excluded: invalid input errors)
- **Error Budget:** Allow up to 7.2 hours downtime per month
- **Degradation Strategy:** Graceful handling of Qdrant unavailability; clear error messages

### Security
- **AuthN/AuthZ:** None required (single-user validation context)
- **Data Handling:** No data stored locally; all data retrieved from Qdrant
- **Secrets Management:** Reuse .env configuration from Spec-1 (no new secrets)
- **Auditing:** Structured logging of all retrieval operations

### Cost
- **Qdrant Cloud:** Free Tier (< 5K vector operations/month included)
- **Cohere API:** Included in existing Spec-1 API key (shared quota)
- **Compute:** Local execution (no cloud costs)

## Data Management and Migration

### Source of Truth
**Qdrant Cloud (textbook_embeddings collection)**
- 26 vectors with metadata (url, page_title, chunk_index)
- Immutable during retrieval phase (Spec-1 may continue ingesting in parallel)

### Schema Compatibility
- Must read existing schema from Spec-1 ingestion
- No schema changes to Qdrant collections (read-only)
- Validate metadata fields match expected structure

### Data Retention
- No new data created by retrieval service (query logs only)
- Query logs retained for 7 days (sufficient for debugging)
- Test results documented in `VALIDATION_RESULTS.txt`

## Operational Readiness

### Observability
- **Logs:** Structured JSON logs with timestamp, operation, query, result_count, latency, status
  - File: `backend/logs/retrieval.log`
  - Console output for interactive debugging
- **Metrics:** Latency, result count, error rate (captured in logs)
- **Traces:** Operation timing breakdown (embed → search → validate)

### Alerting
- No automated alerting (MVP context)
- Manual validation checks: test suite results
- Critical thresholds documented in success criteria

### Runbooks
- **Common Tasks:**
  - Run validation suite: `python -m backend.src.retrieval.cli run-validation`
  - Interactive search: `python -m backend.src.retrieval.cli search "query text"`
  - Get collection info: `python -m backend.src.retrieval.cli stats`
- **Troubleshooting:**
  - "Qdrant connection failed" → Verify QDRANT_URL and QDRANT_API_KEY in .env
  - "Embedding dimension mismatch" → Ensure Cohere model matches (embed-english-v3.0)
  - "Results not relevant" → Check that vectors are from Spec-1 ingestion

### Deployment Strategy
- **Deployment:** No standalone deployment (library integrated with backend)
- **Feature Flags:** None required (MVP scope)
- **Rollback:** Revert to Spec-1 commit if retrieval code breaks

## Risk Analysis and Mitigation

### Top 3 Risks

**1. Qdrant Collection Schema Mismatch**
- **Description:** Metadata fields don't match expected structure (url, page_title, chunk_index)
- **Probability:** Low (Spec-1 explicitly sets these fields)
- **Impact:** Retrieval fails; validation reports incorrect results
- **Mitigation:**
  - Validate metadata schema at service initialization
  - Test with actual Spec-1 collection before deployment
  - Document expected schema in code comments
- **Blast Radius:** Entire retrieval service unusable
- **Kill Switch:** Abort service initialization if schema validation fails; provide clear error message

**2. Cohere Embedding Inconsistency**
- **Description:** Query embeddings don't match stored vector embeddings (different model versions)
- **Probability:** Medium (model versions can change)
- **Impact:** Query results have low relevance; test suite fails
- **Mitigation:**
  - Pin Cohere library version in pyproject.toml
  - Verify embedding dimensions match stored vectors (must be 1024)
  - Test with known Spec-1 vectors to verify consistency
  - Log embedding generation details for debugging
- **Blast Radius:** All query results affected; system appears broken
- **Guardrails:** Automatic dimension check; dimension mismatch causes immediate error

**3. Qdrant Connection Instability**
- **Description:** Qdrant Cloud Free Tier has occasional outages or rate limiting
- **Probability:** Medium (shared infrastructure)
- **Impact:** Validation tests intermittently fail; unclear if issue is code or service
- **Mitigation:**
  - Implement retry logic with exponential backoff (3 retries)
  - Implement circuit breaker (fail fast after repeated errors)
  - Document Qdrant status page; monitor before/during testing
  - Distinguish transient errors from permanent failures in logs
- **Blast Radius:** Validation blocked during outages but doesn't corrupt data
- **Monitoring:** Log all Qdrant errors; generate summary in validation report

## Evaluation and Validation

### Definition of Done
- [ ] All 5 component modules implemented (qdrant_client, embedding_service, validator, retrieval_service, report_generator)
- [ ] Comprehensive unit tests (12+ tests for individual components)
- [ ] Integration tests for end-to-end pipeline (5+ integration tests)
- [ ] Validation tests for all 8-10 test queries (all must return relevant results)
- [ ] All 26 vectors retrievable (100% success rate)
- [ ] Performance metrics meet targets (single query < 500ms p95, batch < 5s)
- [ ] Validation report generated and reviewed (no critical issues)
- [ ] All code documented (docstrings, type hints, README)
- [ ] Security review (no secrets in code, proper error handling)

### Output Validation
- **Format Validation:**
  - Search results have required fields (vector_id, similarity_score, metadata)
  - Metadata has required subfields (url, page_title, chunk_index)
  - Similarity scores are floats in range [0, 1]
  - Results ranked by similarity score (descending)
- **Requirements Validation:**
  - All 8 functional requirements verified (REQ-1 through REQ-8)
  - All 5 non-functional requirements met (NFR-1 through NFR-5)
  - Success criteria achieved (26/26 vectors, 80%+ relevance, < 500ms latency)
- **Safety Validation:**
  - No data corruption (metadata matches Spec-1 ingestion)
  - No secrets exposed in logs or error messages
  - Graceful handling of error conditions

## Architectural Decision Record (ADR)

**Title:** Synchronous Retrieval Service for MVP Validation

**Decision:** Implement synchronous (blocking) retrieval service for Phase 1, deferring async/FastAPI to Phase 2

**Context:** Spec-2 focuses on validating that Spec-1 ingestion works correctly. Minimal scope and quick turnaround time are priorities.

**Rationale:**
1. **Simpler Implementation:** Synchronous code is easier to test and debug than async
2. **Sufficient for MVP:** Single-threaded validation doesn't require async concurrency
3. **Reversible:** Can refactor to async in Phase 2 without changing API contracts
4. **Reduced Dependencies:** Don't need FastAPI, Uvicorn, or async frameworks

**Alternatives Considered:**
1. **Full Async Implementation:** Enables Phase 2 FastAPI directly; adds complexity; increases scope
2. **Cloud-Native Endpoints:** Requires deployment infrastructure; deferred for Phase 2

**Trade-offs:**
- Benefit: Simpler, faster to implement (MVP)
- Cost: Refactoring needed for Phase 2 FastAPI; not suitable for high-concurrency production

**Status:** **Accepted** (Phase 1 MVP)

**Follow-up:** Phase 2 (chatbot integration) will refactor to async/FastAPI with the same validation logic

## Next Steps
- [ ] **Immediate:** Create branch and begin Phase 1 development
- [ ] **Week 1:** Complete Phase 1-3 (foundation, validation, retrieval API)
- [ ] **Week 2:** Complete Phase 4-5 (testing, validation, documentation)
- [ ] **Follow-up:** Run `/sp.tasks` to decompose into 18-24 individual tasks
- [ ] **Integration:** Design Phase 3 (RAG Chatbot) after Spec-2 completion

---

*This plan provides a detailed roadmap for implementing data retrieval and validation for the RAG system. It balances scope, confidence, and production readiness for a compelling MVP demonstration.*
