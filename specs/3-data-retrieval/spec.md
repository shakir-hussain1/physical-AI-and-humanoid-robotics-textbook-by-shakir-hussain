# RAG Spec-2: Data Retrieval & Pipeline Validation

## Version
**Spec Version:** 1.0.0
**Created:** 2025-12-26
**Last Updated:** 2025-12-26

## Feature Overview
**Feature:** Data Retrieval & Pipeline Validation
**Module Alignment:** Backend Infrastructure for RAG Chatbot System
**Target Audience:** Developers and AI engineers validating the RAG ingestion pipeline

## Constitutional Alignment
This specification aligns with the following constitutional principles:
- PRINCIPLE_1: Technical Excellence and Accuracy
- PRINCIPLE_4: Practical Application
- PRINCIPLE_9: RAG Chatbot Integration
- PRINCIPLE_10: Deployment and Platform Accessibility

## Problem Statement

The RAG website ingestion pipeline (Spec-1) successfully stores 26 textbook pages as embeddings in Qdrant Cloud. However, without a validation and retrieval mechanism, we cannot:

1. **Verify Data Integrity** - Confirm that stored vectors match the source content
2. **Test End-to-End Functionality** - Validate that retrieval produces correct results
3. **Build Confidence for Production** - Ensure the system works before integration with chatbot
4. **Diagnose Issues** - Identify problems in the ingestion pipeline through retrieval tests
5. **Enable Downstream Development** - Provide working retrieval API for chatbot integration

This specification addresses the data validation and retrieval layer, ensuring that stored embeddings are correctly retrieved and can satisfy similarity queries with high relevance.

## Requirements

### Functional Requirements

**REQ-1: Vector Retrieval by Similarity Search**
- System must retrieve vectors from Qdrant using cosine similarity distance metric
- For a given query text embedding, return top-K results (default K=5)
- Include metadata (source URL, page title, chunk index) with each result
- Results must be ranked by similarity score (highest first)

**REQ-2: Metadata Preservation and Retrieval**
- All stored vectors must retain original metadata: source URL, page title, chunk index
- Metadata must be retrievable alongside vector similarity scores
- Must support filtering by URL or page title when retrieving results

**REQ-3: Chunk Content Verification**
- For each retrieved vector, the system must verify that the chunk content (text) is still accessible
- Content verification includes checking that source URL is valid and chunk index matches metadata
- Must detect and report mismatches between stored metadata and actual content

**REQ-4: Similarity Search Validation**
- Given a query string, system must generate embedding using same model as ingestion (Cohere)
- Execute similarity search against stored textbook embeddings
- Verify that returned results are semantically relevant to the query
- Must handle edge cases: empty queries, single-word queries, queries with special characters

**REQ-5: Batch Retrieval and Verification**
- System must support retrieving and verifying multiple vectors in batch
- Performance: retrieve and verify 10 vectors in under 5 seconds
- Document retrieval latency for different batch sizes

**REQ-6: Pipeline Validation Report**
- Generate comprehensive validation report showing:
  - Total vectors retrieved vs. total vectors stored
  - Metadata integrity (% of vectors with valid metadata)
  - Content accessibility (% of chunks still retrievable)
  - Retrieval consistency (same query produces same top-K results)
  - Sample query results with relevance scores

**REQ-7: Error Handling and Logging**
- System must gracefully handle Qdrant connection failures
- Log all retrieval operations with timestamps and query/result details
- Document failure cases with clear error messages
- Support retry logic for transient failures (max 3 retries)

**REQ-8: Deterministic Test Queries**
- Provide 5-10 predefined test queries with expected relevance patterns
- Each query must have clear expected outcomes documented
- Tests must be reproducible and deterministic across runs

### Non-Functional Requirements

**NFR-1: Retrieval Latency**
- Single vector similarity search: < 500ms latency (including embedding generation)
- Batch retrieval of 10 vectors: < 5 seconds total
- 95th percentile latency: < 1 second for single queries

**NFR-2: Retrieval Accuracy**
- Similarity search must return semantically relevant results
- Top-1 result relevance: 80%+ of test queries return highly relevant result
- Top-5 results relevance: 90%+ of test queries contain at least 2 highly relevant results

**NFR-3: Data Consistency**
- Vector retrieval results must be deterministic (same query → same top-K results)
- Metadata must match ingested data (URL, title, chunk index consistent)
- 100% of stored vectors must be retrievable without data corruption

**NFR-4: Reliability**
- System uptime for retrieval operations: 99%+
- Graceful handling of Qdrant service degradation
- Fallback mechanisms for transient failures
- Comprehensive logging for debugging failures

**NFR-5: Scalability**
- Support retrieval from collection with 100+ vectors without performance degradation
- Support batch operations with up to 50 concurrent requests

### Constraints

- **Language:** Python (consistent with Spec-1)
- **Vector Database:** Qdrant Cloud (same instance as Spec-1)
- **Embedding Model:** Cohere embed-english-v3.0 (must match Spec-1 for consistency)
- **Vector Dimension:** 1024 (fixed by Cohere model)
- **Test Dataset:** 26 vectors from Physical AI & Humanoid Robotics textbook (stored in Spec-1)
- **Backward Compatibility:** Must work with existing textbook_embeddings collection
- **No Agent Integration:** Do not build OpenAI SDK integration or agent functionality
- **No UI Components:** Focus on programmatic retrieval API only
- **No Advanced Retrieval:** No reranking, fusion, or complex retrieval strategies
- **No Benchmarking:** Minimal performance testing; focus on correctness

## Solution Approach

### High-Level Design

The validation layer will:

1. **Connect to Qdrant** - Reuse credentials and client configuration from Spec-1
2. **Generate Query Embeddings** - Use same Cohere service for embedding test queries
3. **Execute Similarity Search** - Query Qdrant using cosine similarity distance
4. **Validate Results** - Verify metadata integrity and content accessibility
5. **Report Findings** - Generate human-readable validation report with pass/fail results

### Technical Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  Data Retrieval System                       │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────────┐          ┌──────────────────┐         │
│  │  Query Service   │          │ Cohere Client    │         │
│  │  (entry point)   │──────────▶ (Embedder)       │         │
│  └──────────────────┘          └──────────────────┘         │
│         │                               │                    │
│         │                               ▼                    │
│         │                        ┌─────────────┐            │
│         │                        │  Embedding  │            │
│         │                        │  (1024-dim) │            │
│         │                        └─────────────┘            │
│         │                               │                    │
│         ▼                               ▼                    │
│  ┌──────────────────────────────────────────────┐           │
│  │     Qdrant Similarity Search Client          │           │
│  │  (vector_search + metadata retrieval)        │           │
│  └──────────────────────────────────────────────┘           │
│         │                                                    │
│         ▼                                                    │
│  ┌──────────────────────────────────────────────┐           │
│  │   Validation & Verification Layer            │           │
│  │  (metadata check, content verification)      │           │
│  └──────────────────────────────────────────────┘           │
│         │                                                    │
│         ▼                                                    │
│  ┌──────────────────────────────────────────────┐           │
│  │  Report Generation & Logging                 │           │
│  │  (validation results, statistics)            │           │
│  └──────────────────────────────────────────────┘           │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Query Input** - Developer provides query text (e.g., "ROS2 communication")
2. **Embedding Generation** - Query text converted to 1024-dim Cohere embedding
3. **Similarity Search** - Query embedding compared against stored textbook vectors using cosine similarity
4. **Result Ranking** - Results returned sorted by similarity score (highest first)
5. **Metadata Attachment** - Each result includes: similarity_score, source_url, page_title, chunk_index
6. **Validation** - System verifies metadata consistency and content accessibility
7. **Report Output** - Return results with validation status and any detected issues

## Detailed Design

### Component 1: Qdrant Query Service
- **Purpose:** Execute similarity search queries against stored vectors in Qdrant Cloud
- **Interfaces:**
  - Input: Query text (string), K value (int, default=5)
  - Output: List of (similarity_score, metadata) tuples
- **Dependencies:** Qdrant Cloud client, Cohere embedding service
- **Methods:**
  - `search_by_similarity(query_text: str, k: int = 5) -> List[Dict]`
  - `search_by_metadata_filter(url_filter: str = None) -> List[Dict]`
  - `get_collection_stats() -> Dict`

### Component 2: Embedding Generation Service
- **Purpose:** Generate query embeddings using same model as ingestion pipeline
- **Interfaces:**
  - Input: Query text (string or list of strings)
  - Output: 1024-dimensional embedding vector(s)
- **Dependencies:** Cohere API client (reuse from Spec-1)
- **Methods:**
  - `embed_query(query_text: str) -> List[float]`
  - `embed_batch(queries: List[str]) -> List[List[float]]`

### Component 3: Validation & Verification Engine
- **Purpose:** Verify data integrity of retrieved vectors and metadata
- **Interfaces:**
  - Input: Retrieved vectors with metadata
  - Output: Validation report with pass/fail status
- **Dependencies:** Qdrant client, HTTP client for URL verification
- **Validation Checks:**
  - Metadata completeness (URL, title, chunk index present)
  - Metadata consistency (values match expected types and formats)
  - Content accessibility (URLs still resolve)
  - Embedding consistency (re-embedding same text produces similar vector)
  - Retrieval determinism (repeated queries return same results)

### Component 4: Test Query and Validation Suite
- **Purpose:** Provide reproducible test cases with expected outcomes
- **Test Queries:** Predefined 8-10 queries covering:
  - Single keywords ("ROS2", "embedding", "simulation")
  - Multi-word phrases ("robot communication", "digital twin simulation")
  - Complex queries ("how to implement behavior planning")
  - Edge cases (special characters, numbers, very short/long queries)
- **Expected Outcomes:** For each query, document expected relevance patterns
  - Primary domain (which module should have highest relevance)
  - Expected top-3 relevant chunks
  - Expected similarity score range

### Component 5: Report Generation and Logging
- **Purpose:** Generate human-readable validation reports and structured logs
- **Report Sections:**
  - Collection statistics (total vectors, collection size)
  - Retrieval test results (queries executed, results returned)
  - Validation summary (pass/fail for each check)
  - Performance metrics (latency, throughput)
  - Sample results (show top-3 for each query)
  - Failure diagnostics (any issues detected)
- **Log Format:** Structured JSON logs with timestamp, operation, result, duration

## User Experience

### Learning Objectives
- Understand how vector similarity search works in practice
- Learn to validate data integrity in vector databases
- Gain confidence in RAG pipeline correctness before production use
- Understand metadata preservation and retrieval patterns

### Content Structure
- **Section 1: Retrieval Fundamentals** - How similarity search works, cosine distance, top-K results
- **Section 2: Query Validation** - Testing retrieval with known queries, expected outcomes
- **Section 3: Data Integrity Checks** - Metadata verification, content consistency, determinism
- **Section 4: Production Readiness** - Checklist for validating pipeline before deployment

## Implementation Requirements

### Technology Stack
- **Primary Framework:** Python 3.10+ (consistent with Spec-1)
- **Vector Database Client:** qdrant-client
- **Embedding Service:** Cohere API client
- **HTTP Client:** requests (for URL verification)
- **Configuration:** python-dotenv (reuse from Spec-1)
- **Logging:** Python logging with JSON formatter

### Code Standards
- Follow PEP 8 style guidelines
- Type hints on all function signatures
- Docstrings for all classes and public methods
- Error handling with specific exception types
- Structured logging with context (operation, query, results)
- No hardcoded credentials or configuration

### Testing Requirements
- **Unit Tests:** Test each service component independently
  - Embedding service returns correct dimensions
  - Query parsing handles edge cases
  - Validation checks detect metadata issues
- **Integration Tests:** End-to-end retrieval workflow
  - Query → embedding → search → validation → report
  - Verify with 5-10 predefined test queries
- **Validation Tests:** Verify correctness against specification
  - Retrieved vectors have correct metadata
  - Similarity scores are in valid range [0, 1]
  - Results are deterministic across runs

## Quality Assurance

### Technical Validation
- Execute 10 test queries and verify results are semantically relevant
- Compare retrieval results against manual ground truth (developer review)
- Verify embedding dimensions match ingested data (1024)
- Test with edge cases (empty queries, very long queries, special characters)
- Validate error handling with simulated Qdrant failures

### Validation Metrics
- Retrieval success rate: 100% of valid queries return results
- Metadata integrity: 100% of results have complete metadata
- Content accessibility: 100% of URLs still resolve
- Determinism: Repeated queries within 5 minutes return identical results
- Latency: Single query embedding + search < 500ms (95th percentile)

## Dependencies and Integrations

### Internal Dependencies
- **Spec-1 (Website Ingestion):** Must have completed ingestion pipeline
  - Requires: textbook_embeddings collection in Qdrant
  - Requires: 26 vectors with metadata (URL, page_title, chunk_index)
- **Configuration:** Reuse .env from Spec-1 (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)

### External Dependencies
- **Qdrant Cloud:** Same instance as Spec-1 (Free Tier)
- **Cohere API:** Same account and API key as Spec-1
- **Python Libraries:**
  - qdrant-client (≥2.7.0)
  - cohere (≥5.0.0)
  - requests (≥2.31.0)
  - python-dotenv (≥1.0.0)

## Risks and Mitigation

### Technical Risks

**Risk 1: Qdrant Connection Failures**
- **Impact:** Retrieval operations fail, blocking validation
- **Mitigation:** Implement retry logic with exponential backoff; timeout handling; clear error messages

**Risk 2: Embedding Inconsistency**
- **Impact:** Query embeddings don't match stored vectors; poor retrieval results
- **Mitigation:** Verify Cohere model version matches Spec-1; test with known queries; log embedding dimensions

**Risk 3: Metadata Corruption**
- **Impact:** Retrieved results have wrong source information
- **Mitigation:** Validate metadata structure; compare against ingested data; implement consistency checks

**Risk 4: API Rate Limiting**
- **Impact:** Batch validation queries hit Cohere rate limits
- **Mitigation:** Implement request throttling; use batch API when available; monitor usage

### Operational Risks

**Risk 1: Test Data Changes**
- **Impact:** Stored vectors modified between ingestion and retrieval; validation fails
- **Mitigation:** Document collection state; verify vector count matches expected (26); implement immutability checks

**Risk 2: Determinism Issues**
- **Impact:** Same query produces different results on different runs
- **Mitigation:** Document all randomness sources; fix random seeds for testing; verify consistency

## Success Criteria

### Quantitative Measures
- Retrieve 100% of stored vectors (26/26) without data loss
- Query latency: Single embedding + search < 500ms (95th percentile)
- Batch retrieval (10 vectors): < 5 seconds
- Metadata completeness: 100% of retrieved vectors have all metadata fields
- Test query relevance: 8 out of 10 test queries return ≥2 highly relevant results in top-5
- Determinism: 100% of repeated queries return identical top-5 results
- Availability: System handles 10 concurrent retrieval requests without errors

### Qualitative Measures
- Validation report clearly documents pipeline correctness
- Error messages are actionable (specific issue identified, suggested fix)
- Retrieved results are visibly relevant to query (manual inspection)
- Documentation enables developers to run validation independently
- Test queries cover diverse topics from textbook content

## Assumptions

- Cohere model version in Spec-1 (embed-english-v3.0) will remain unchanged
- Qdrant collection name will remain "textbook_embeddings" with 1024-dim vectors
- All 26 source URLs from Spec-1 remain accessible and unchanged
- Similarity score in range [0, 1] with 1.0 = perfect match (cosine distance)
- Test queries will be in English (matching ingested textbook language)
- Metadata structure matches Spec-1 ingestion (url, page_title, chunk_index fields)

## References and Citations

- **Spec-1:** RAG Website Ingestion & Vector Storage (specs/2-website-ingestion/spec.md)
- **Qdrant Documentation:** https://qdrant.tech/documentation/
- **Cohere Documentation:** https://docs.cohere.com/
- **Cosine Similarity:** https://en.wikipedia.org/wiki/Cosine_similarity
- **Vector Database Best Practices:** Qdrant Cloud documentation on retrieval and validation

---

*This specification defines the data retrieval and validation layer for the RAG system, enabling confident deployment of the ingestion pipeline and laying groundwork for chatbot integration.*
