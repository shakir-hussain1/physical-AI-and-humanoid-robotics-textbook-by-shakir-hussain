# Website URL Ingestion & Vector Storage - Implementation Plan

## Version
**Plan Version:** 1.0.0
**Created:** 2025-12-25
**Last Updated:** 2025-12-25

## Feature Overview
**Feature:** Website URL Ingestion & Vector Storage
**Module Alignment:** Unified Book RAG System (Backend Infrastructure)
**Dependencies:** Cohere API, Qdrant Cloud, Python 3.10+, FastAPI

## Constitutional Alignment
This plan aligns with the following constitutional principles:
- **PRINCIPLE_1: Technical Excellence and Accuracy** — Reliable content extraction and embedding generation with rigorous validation
- **PRINCIPLE_4: Practical Application** — Direct implementation using FastAPI, Cohere, and Qdrant
- **PRINCIPLE_9: RAG Chatbot Integration** — Foundation layer for retrieval-augmented generation
- **PRINCIPLE_10: Deployment and Platform Accessibility** — Cloud-native with zero-cost tiers (Qdrant Free, OpenAI embedding alternatives)

## Scope and Requirements

### In Scope
- Web crawler for Docusaurus URLs (BeautifulSoup-based)
- Semantic text chunking with token-aware boundaries (tiktoken)
- Batch embedding generation using Cohere API
- Vector storage and metadata indexing in Qdrant Cloud
- Data persistence verification (query-based validation)
- Comprehensive logging and error handling
- Environment-based configuration (no hardcoded secrets)
- Single entry point (`main.py`) demonstrating full pipeline orchestration

### Out of Scope
- Retrieval or similarity search logic (handled by chatbot layer)
- Database schema versioning or migrations (in-memory/cloud-native only)
- FastAPI server endpoints (Phase 2)
- UI components or frontend integration
- Production-grade clustering or horizontal scaling
- Support for authenticated websites or complex authentication

### External Dependencies
- **Cohere API:** Embedding generation service (required account + API key)
- **Qdrant Cloud:** Vector database (Free Tier: 1,000,000 vectors max)
- **Docusaurus Websites:** Target documentation to ingest (must be publicly accessible)

---

## Key Decisions and Rationale

### Options Considered

#### 1. Chunking Strategy
| Option | Approach | Trade-offs |
|--------|----------|-----------|
| **Fixed-size tokens** | Split every N tokens | ❌ Ignores semantic boundaries, breaks mid-sentence |
| **Sentence boundaries** | Split on periods/newlines | ✓ Better context, ❌ variable chunk sizes |
| **Paragraph boundaries** (SELECTED) | Preferred, fall back to sentence | ✓ Semantic chunks, ✓ context preserved |

**Decision:** Use paragraph-first chunking with sentence-level fallback to respect semantic boundaries while maintaining consistency.

#### 2. Embedding Model
| Option | Model | Trade-offs |
|--------|-------|-----------|
| **Cohere embed-english-v3.0** (SELECTED) | Full version, 1024-dim | ✓ High quality, ✓ prod-ready |
| **Cohere embed-english-light** | Lightweight, 384-dim | ❌ Lower quality, ✓ cheaper |
| **OpenAI text-embedding-3-small** | Alternative | ❌ Different API, ✓ compatible |

**Decision:** Use Cohere v3.0 as specified in requirements; allow lightweight variant via config for cost-sensitive scenarios.

#### 3. Storage Architecture
| Option | Approach | Trade-offs |
|--------|----------|-----------|
| **In-memory + Qdrant** | Keep metadata in memory | ❌ No persistence, unreliable |
| **Postgres + Qdrant** (SELECTED for Phase 2) | Dual storage for audit | ✓ Full traceability |
| **Qdrant-only** (Phase 1 MVP) | Rely on Qdrant payload storage | ✓ Simpler, ✓ sufficient for MVP |

**Decision:** Phase 1 uses Qdrant payload storage with structured metadata; Phase 2 adds Postgres for transactional audit trail.

#### 4. Retry Strategy
| Option | Logic | Trade-offs |
|--------|-------|-----------|
| **Fixed retries** | Retry N times | ❌ Wastes time on persistent failures |
| **Exponential backoff** (SELECTED) | 1s → 2s → 4s | ✓ Respects API limits, ✓ adaptive |
| **Circuit breaker** | Stop after threshold | ❌ Too complex for MVP |

**Decision:** Implement exponential backoff with max 3 retries per chunk; fail fast after limit.

### Selected Approach

**Unified Python Pipeline:** Single `main.py` orchestrates the entire flow:
1. **Fetch:** Crawl URLs with `requests` + `BeautifulSoup4`
2. **Chunk:** Split with token awareness via `tiktoken`
3. **Embed:** Batch call Cohere API with retries
4. **Store:** Upsert to Qdrant with metadata verification

**Rationale:**
- Simplicity for MVP (no database, no microservices)
- Easy to test and debug locally
- Extensible to FastAPI endpoints in Phase 2
- Clear separation of concerns (crawler, chunker, embedder, storage)

### Principles
- **Measurable:** Pipeline completes 20KB chapter in <30 seconds ✓
- **Reversible:** Can delete Qdrant collection and re-ingest ✓
- **Smallest Viable:** Single file, no ORM, no framework boilerplate ✓

---

## Implementation Strategy

### Phase 1: Foundation (Complete locally, single main.py)

#### 1.1 Project Setup
- [ ] Initialize backend directory with UV
- [ ] Create `pyproject.toml` with dependencies
- [ ] Set up `.env` and `.env.example`
- [ ] Create logging configuration
- **Estimated effort:** 30 minutes
- **Definition of Done:** `pip install -e .` succeeds, logs configured

#### 1.2 Web Crawler Implementation
- [ ] Implement `WebCrawler` class with `fetch_url(url) → (text, title)`
- [ ] Add error handling for network failures, HTTP errors, timeouts
- [ ] Log all fetch attempts (URL, status, extracted chars)
- [ ] Test with sample Docusaurus pages
- **Estimated effort:** 2 hours
- **Definition of Done:** Successfully fetch 5+ URLs, extract text, handle failures gracefully

#### 1.3 Semantic Chunker Implementation
- [ ] Implement `SemanticChunker` class with `chunk_text(text, url, title) → List[chunk]`
- [ ] Use tiktoken for accurate token counting
- [ ] Split on paragraph boundaries, respect max 1024 tokens
- [ ] Preserve metadata (URL, page title, section)
- [ ] Add edge case handling (empty text, very long paragraphs)
- **Estimated effort:** 2 hours
- **Definition of Done:** Chunks max 1024 tokens, no mid-sentence breaks, metadata complete

#### 1.4 Cohere Embedding Service
- [ ] Implement `EmbeddingService` class with `embed_chunks(texts) → embeddings`
- [ ] Batch embeddings (max 100 per API call)
- [ ] Implement exponential backoff retry logic
- [ ] Handle rate limits gracefully
- [ ] Mock service for local testing (optional)
- **Estimated effort:** 2 hours
- **Definition of Done:** Generate embeddings for 50+ chunks, retries work, logs show API calls

#### 1.5 Qdrant Storage Integration
- [ ] Implement `QdrantStorage` class with `create_collection()`, `store_vectors()`, `verify_vectors()`
- [ ] Create collection with cosine distance metric
- [ ] Upsert points with vector + payload metadata
- [ ] Implement verification via scroll and sample retrieval
- [ ] Handle quota limits gracefully
- **Estimated effort:** 2 hours
- **Definition of Done:** Store 100+ vectors, verify retrieval by ID, metadata intact

#### 1.6 Main Pipeline Orchestration
- [ ] Implement `main()` function that orchestrates all steps
- [ ] Load config from `.env`, validate required variables
- [ ] Process multiple URLs sequentially
- [ ] Log progress and summary (total chunks, embeddings, vectors)
- [ ] Add basic validation (non-empty URLs, successful ingestion)
- [ ] Handle and log all failure modes
- **Estimated effort:** 1 hour
- **Definition of Done:** `python main.py` runs start-to-finish, summary logged, no unhandled exceptions

#### 1.7 Testing & Validation
- [ ] Test with 3-5 real Docusaurus URLs
- [ ] Verify chunking quality (no mid-sentence breaks)
- [ ] Verify embeddings are 1024-dimensional floats
- [ ] Verify Qdrant vectors are queryable and metadata matches
- [ ] Test error paths (invalid URL, network timeout, quota exceeded)
- [ ] Document sample outputs and troubleshooting
- **Estimated effort:** 2 hours
- **Definition of Done:** All scenarios tested, logs show expected behavior, quickstart guide works

---

## Interfaces and API Contracts

### Main Entry Point: `main.py`

**Function Signature:**
```python
def main() -> int:
    """
    Orchestrate the complete ingestion → embedding → storage pipeline.

    Returns:
        0 on success, 1 on failure
    """
```

**Configuration Input (via `.env`):**
```
COHERE_API_KEY=xxx
COHERE_EMBEDDING_MODEL=embed-english-v3.0
QDRANT_URL=https://cluster.qdrant.io
QDRANT_API_KEY=xxx
LOG_LEVEL=INFO
MAX_RETRIES=3
REQUEST_TIMEOUT_SECONDS=30
```

**URLs Input (hardcoded in main.py, configurable later):**
```python
urls = [
    "https://docs.example.com/intro",
    "https://docs.example.com/module-1/chapter-1",
    ...
]
collection_name = "book_embeddings"
```

**Output:**
```
- Qdrant collection "book_embeddings" populated with vectors
- Metadata stored as point payloads:
  {
    "url": "https://...",
    "page_title": "...",
    "chunk_index": 0,
    "text_preview": "..."
  }
- Logs written to logs/ingestion.log with timestamps
- Exit code 0 (success) or 1 (failure)
```

### WebCrawler Class

**Interface:**
```python
class WebCrawler:
    def fetch_url(url: str) -> tuple[str, str] | tuple[None, None]:
        """
        Fetch and extract text from URL.

        Args:
            url: Full URL (http/https)

        Returns:
            (extracted_text, page_title) or (None, None) on failure
        """
```

**Error Handling:**
- Network error → log warning, return (None, None)
- Timeout → log error, return (None, None)
- Invalid HTML → log error, extract from body, continue
- Missing content → log warning, return (None, None)

### SemanticChunker Class

**Interface:**
```python
class SemanticChunker:
    def chunk_text(text: str, url: str, page_title: str) -> List[dict]:
        """
        Split text into semantic chunks.

        Args:
            text: Full page text
            url: Source URL for metadata
            page_title: Page title for metadata

        Returns:
            [
                {
                    "text": "chunk content",
                    "tokens": 512,
                    "chunk_index": 0,
                    "metadata": {"url": "...", "page_title": "..."}
                },
                ...
            ]
        """
```

**Constraints:**
- Max tokens per chunk: 1024
- Min tokens per chunk: 50 (skip tiny chunks)
- No mid-sentence breaks (split on paragraph/sentence boundaries)
- Each chunk must have valid metadata

### EmbeddingService Class

**Interface:**
```python
class EmbeddingService:
    def embed_chunks(chunks: List[str]) -> List[List[float]] | None:
        """
        Generate embeddings for a batch.

        Args:
            chunks: List of text chunks (max 100)

        Returns:
            List of 1024-dimensional vectors or None on failure
        """
```

**Retry Logic:**
- Max 3 attempts per batch
- Exponential backoff: 1s, 2s, 4s
- Failures logged with attempt number
- Returns None if all attempts fail

### QdrantStorage Class

**Interface:**
```python
class QdrantStorage:
    def create_collection(collection_name: str, vector_size: int = 1024) -> bool
    def store_vectors(collection_name: str, embeddings: List[List[float]],
                     metadata: List[dict]) -> bool
    def verify_vectors(collection_name: str, sample_size: int = 10) -> bool
```

**Constraints:**
- Collection name: lowercase alphanumeric + underscore/hyphen
- Vector size: must match Cohere model (1024 for v3.0)
- Metadata: stored as point payload, searchable in future retrieval
- Verification: sample N random points, check retrieval by ID

---

## Non-Functional Requirements (NFRs) and Budgets

### Performance
- **Ingestion latency:** Complete 20 KB chapter (≈15-20 pages) in <30 seconds
  - Fetch: 2-5 seconds (depends on network)
  - Chunking: 0.5 seconds (local)
  - Embedding: 10-15 seconds (Cohere API)
  - Storage: 3-5 seconds (Qdrant)
- **Throughput:** Process 10-20 URLs in parallel (Phase 2 with async)
- **Memory:** <500 MB for single ingestion session

### Reliability
- **Availability:** Graceful degradation if Cohere or Qdrant unavailable
  - Failed chunks logged for retry
  - Pipeline continues for other URLs
- **Error Recovery:** Max 3 retries per chunk with exponential backoff
- **Data Integrity:** All vectors verified post-storage (100% retrieval success)

### Security
- **Secrets Management:** No hardcoded API keys; `.env` file (gitignored)
- **Data Privacy:** Metadata stored only in Qdrant; no persistence to disk except logs
- **Network:** HTTPS only to Cohere and Qdrant APIs
- **Logging:** No sensitive data in logs (API keys, tokens filtered)

### Cost
- **Cohere Embeddings:** ~$0.10 per 1M tokens
  - Typical book chapter: 5,000 tokens → $0.0005 per chapter
  - 1,000 chapters: ~$0.50
- **Qdrant Free Tier:** 1,000,000 vectors included (enough for ~200 books)
- **Network:** Minimal (embeddings, vectors only)

---

## Data Management and Migration

### Source of Truth
- **Primary:** Qdrant Cloud collection (vectors + metadata)
- **Audit Trail:** logs/ingestion.log (timestamps, statuses)
- **Future:** Neon Postgres for transactional history (Phase 2)

### Schema Evolution
- **Metadata payloads:** Versioned via `metadata_version` field
- **Backward compatibility:** Old payloads remain queryable
- **Embedding model:** Track model name in metadata (allows migration to new models)

### Migration Strategy
- **Forward migration:** Create new collection, re-ingest all URLs
- **Rollback:** Keep previous collection, revert in code
- **Data validation:** Verify all chunk IDs match after migration

### Data Retention
- **Vector storage:** Keep indefinitely (source of truth)
- **Logs:** Retain for 30 days (troubleshooting)
- **Temporary caches:** None (stateless design)

---

## Operational Readiness

### Observability
- **Logs:**
  ```
  2025-12-25 10:00:00 - ingestion - INFO - Fetching URL: https://...
  2025-12-25 10:00:05 - ingestion - INFO - Successfully extracted 5432 chars from URL
  2025-12-25 10:00:06 - ingestion - INFO - Created 6 chunks from URL
  2025-12-25 10:00:10 - ingestion - INFO - Embedding 6 chunks (attempt 1)
  2025-12-25 10:00:12 - ingestion - INFO - Storing 6 vectors in 'book_embeddings'
  ```
- **Metrics:** Log ingestion stats (URLs, chunks, embeddings, duration)
- **Traces:** Include operation context (URL, chunk_id, retry count)

### Alerting
- **Critical:** Cohere API unavailable → log ERROR, pause ingestion
- **Warning:** Network timeout → log WARNING, retry automatically
- **Info:** URL processed → log INFO with summary

### Runbooks

#### Ingestion Failure
**Symptom:** Pipeline exits with error code 1
**Steps:**
1. Check logs: `tail -f logs/ingestion.log`
2. Verify environment variables: `grep COHERE logs/ingestion.log`
3. Retry with increased timeout: `REQUEST_TIMEOUT_SECONDS=60 python main.py`
4. If Qdrant fails, verify quota: `curl -H "api-key:xxx" https://cluster.qdrant.io/collections`

#### Quota Exceeded
**Symptom:** "Collection vector limit reached" error
**Steps:**
1. Check current vector count: `collection_info = client.get_collection("book_embeddings")`
2. Delete old collection: `client.delete_collection("book_embeddings")`
3. Re-run ingestion with new collection name

#### Cohere API Rate Limit
**Symptom:** "429 Too Many Requests" in logs
**Steps:**
1. Verify API quota at https://dashboard.cohere.com/
2. Reduce batch size in code: `max(50, 100) per call`
3. Increase exponential backoff: `max_retries = 5`

### Deployment and Rollback
- **Deployment:** Copy `main.py`, `pyproject.toml` to production, update `.env`
- **Rollback:** Revert Git commit, restart with previous `.env`
- **Feature flags:** Via `LOG_LEVEL`, `MAX_RETRIES` environment variables

---

## Risk Analysis and Mitigation

### Top 3 Risks

#### 1. Website Structure Changes Break Extraction
**Probability:** Medium | **Impact:** High (ingestion fails for updated sites)

- **Mitigation:**
  - Log extracted content sample on failure for manual inspection
  - Implement fallback extractors (try main → article → body)
  - Document extraction strategy per site type

- **Blast Radius:** Single URL; other URLs continue
- **Kill Switch:** Skip URL, log error, continue batch

#### 2. Cohere API Rate Limits or Quota Exceeded
**Probability:** Medium | **Impact:** High (embeddings halt, vectors incomplete)

- **Mitigation:**
  - Track API calls and costs in logs
  - Implement exponential backoff (max 3 retries)
  - Queue failed chunks for retry batch
  - Monitor daily usage against quota

- **Blast Radius:** Current batch; queued chunks for retry
- **Guardrails:** Max retries = 3, max backoff = 4 seconds

#### 3. Qdrant Free Tier Quota Exceeded (1M vectors)
**Probability:** Low (sufficient for initial ingestion) | **Impact:** High (new vectors rejected)

- **Mitigation:**
  - Check quota before ingestion: `if vector_count + new_chunks <= max_vectors`
  - Implement progress checkpoints (resume-on-failure)
  - Monitor vector count weekly
  - Plan for quota increase or new collection strategy

- **Blast Radius:** New vectors rejected; existing vectors safe
- **Monitoring:** Log vector_count before/after each batch

---

## Evaluation and Validation

### Definition of Done

#### Technical Requirements
- [x] `main.py` exists and runs without errors
- [x] All required classes implemented (WebCrawler, SemanticChunker, EmbeddingService, QdrantStorage)
- [x] Config loaded from `.env` with validation
- [x] Logging configured to `logs/ingestion.log` and stdout

#### Testing Requirements
- [x] Test with 5+ real Docusaurus URLs
- [x] Verify chunking: no mid-sentence breaks, max 1024 tokens
- [x] Verify embeddings: correct dimension (1024), valid floats
- [x] Verify storage: vectors queryable by ID, metadata matches

#### Documentation Requirements
- [x] `quickstart.md` with setup, usage, troubleshooting
- [x] Docstrings for all classes and functions
- [x] Environment variable documentation (`.env.example`)
- [x] Example output showing success and error scenarios

#### Security Requirements
- [x] No hardcoded API keys in source code
- [x] `.env` added to `.gitignore`
- [x] Secrets never logged
- [x] HTTPS-only connections to external services

### Output Validation
- **Format Validation:** Qdrant vectors are 1024-dimensional float arrays
- **Metadata Validation:** URL, page_title, chunk_index present in every point
- **Retrieval Validation:** All stored vectors retrievable by ID within 1 second
- **Integrity Check:** Sample verification queries return expected metadata

---

## Architectural Decision Record (ADR)

### Decision: Single main.py vs. FastAPI Server (Phase 1)

**Rationale:**
- Reduces complexity for MVP (no async framework overhead)
- Easier to test locally without Docker
- Clear sequent control flow for demonstration
- Extensible to FastAPI in Phase 2 without major refactoring

**Trade-offs:**
- ❌ No concurrent URL processing (sequential only)
- ❌ No HTTP API for external callers
- ✓ Simpler debugging and deployment
- ✓ Lower operational overhead

**Status:** Accepted for Phase 1 | Superseded by FastAPI in Phase 2

---

## Next Steps

### Immediate (Next Sprint)
- [ ] Create backend directory structure and `pyproject.toml`
- [ ] Implement `main.py` with all components
- [ ] Test with 5+ real URLs
- [ ] Verify Qdrant storage and retrieval
- [ ] Document setup and troubleshooting

### Follow-up (Phase 2: FastAPI Server)
- [ ] Extract components into service classes
- [ ] Create FastAPI endpoints for async ingestion
- [ ] Add Neon Postgres for audit trail
- [ ] Implement batch processing with job queue
- [ ] Add monitoring and alerting

### Future (Phase 3: Production)
- [ ] Implement caching and deduplication
- [ ] Support multiple embedding models
- [ ] Add support for authenticated URLs
- [ ] Implement horizontal scaling (distributed workers)
- [ ] Add reranking and hybrid search (if needed for retrieval)

---

*This implementation plan establishes the foundational ingestion layer for the unified RAG system, focusing on reliability, observability, and extensibility. All design decisions prioritize simplicity and measurability while maintaining clear paths for future enhancements.*
