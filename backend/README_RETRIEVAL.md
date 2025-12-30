# Retrieval Service - Data Retrieval & Pipeline Validation

## Overview

The Retrieval Service implements the data retrieval and validation layer for the RAG (Retrieval-Augmented Generation) system. It provides:

- **Vector Similarity Search:** Query stored embeddings using cosine similarity
- **Metadata Validation:** Verify data integrity of retrieved vectors
- **Content Verification:** Check accessibility and consistency of source content
- **Test Query Suite:** Deterministic validation with 8-10 predefined test queries
- **Comprehensive Reporting:** Generate validation reports with detailed statistics
- **CLI Interface:** Command-line tools for search, validation, and diagnostics

## Architecture

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

## Module Structure

```
backend/src/retrieval/
├── __init__.py                 # Public interfaces
├── config.py                   # Configuration management
├── qdrant_client.py            # Qdrant vector database wrapper
├── embedding_service.py        # Cohere embedding generation
├── validator.py                # Metadata/content validation
├── test_queries.py             # Test query suite (8-10 queries)
├── retrieval_service.py        # Main retrieval service
├── logging.py                  # Structured logging
├── report_generator.py         # Validation report generation
└── cli.py                      # Command-line interface
```

## Usage

### Python API

```python
from backend.src.retrieval.config import RetrievalConfig
from backend.src.retrieval.retrieval_service import RetrievalService

# Load configuration from environment
config = RetrievalConfig.load_from_env()

# Initialize retrieval service
service = RetrievalService(config)

# Execute similarity search
result = service.search("ROS2 communication middleware", k=5)
print(f"Found {len(result['results'])} results in {result['latency_ms']}ms")

for result in result['results']:
    print(f"  Score: {result['similarity_score']:.3f}")
    print(f"  URL: {result['metadata']['url']}")
    print(f"  Title: {result['metadata']['page_title']}")

# Validate retrieval results
validation = service.validate_retrieval("robot simulation")
print(f"Validation: {'PASS' if validation['validation']['is_valid'] else 'FAIL'}")

# Get collection statistics
stats = service.get_stats()
print(f"Collection: {stats['total_vectors']} vectors, {stats['vector_dimension']}-dimensional")
```

### Command-Line Interface

#### Run Full Validation
```bash
python -m backend.src.retrieval.cli run-validation
```
Executes all 8-10 test queries and generates comprehensive validation report.

#### Search Query
```bash
python -m backend.src.retrieval.cli search "ROS2 communication" -k 5
```
Execute similarity search and display top-K results.

#### Get Collection Info
```bash
python -m backend.src.retrieval.cli get-info
```
Display collection statistics, vector dimensions, and service health.

#### Test Query Suite
```bash
python -m backend.src.retrieval.cli test-query
python -m backend.src.retrieval.cli test-query -q "custom query"
```
Run predefined test queries or execute a custom query.

## Configuration

Environment variables (in `.env`):

```bash
# Qdrant Cloud Configuration
QDRANT_URL=https://your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=textbook_embeddings

# Cohere API Configuration
COHERE_API_KEY=your-cohere-api-key
COHERE_MODEL=embed-english-v3.0

# Retrieval Settings
RETRIEVAL_DEFAULT_K=5              # Default number of results
RETRIEVAL_MAX_K=100                # Maximum number of results
RETRIEVAL_TIMEOUT_SECONDS=30       # Request timeout

# Logging Configuration
LOG_LEVEL=INFO                     # DEBUG, INFO, WARNING, ERROR
LOG_FORMAT=json                    # json or text
```

## Test Query Suite

The retrieval service includes 8-10 deterministic test queries covering:

- **Keywords:** Single-word queries (ROS2, simulation, embedding)
- **Phrases:** Multi-word queries (robot communication, digital twin)
- **Complex:** Detailed queries (how to implement behavior planning)
- **Edge Cases:** Short, special characters, numbers

Each test query has documented expected outcomes:
- Expected primary domain
- Expected relevant modules
- Expected similarity score range
- Relevance expectation (high/medium/low)

## Validation Metrics

### Functional Validation
- **Vector Retrieval:** 26/26 vectors retrievable (100%)
- **Metadata Completeness:** 100% of results have all required fields
- **Metadata Consistency:** URL format, chunk_index validity
- **Content Accessibility:** URLs resolve and content is accessible
- **Ranking Correctness:** Results sorted by similarity score (descending)

### Performance Validation
- **Single Query Latency:** < 500ms (p95), < 1s (p99)
- **Batch Retrieval:** 10 vectors in < 5 seconds
- **Query Accuracy:** 80%+ top-1 relevance, 90%+ top-5 relevance

### Consistency Validation
- **Determinism:** Same query → identical results across runs
- **Data Integrity:** 100% of stored vectors retrievable without corruption
- **Metadata Consistency:** Metadata matches ingested data

## Output Files

### Validation Report
`backend/VALIDATION_RESULTS.txt` - Human-readable validation report with:
- Summary statistics (pass/fail rate, latency)
- Per-query results with validation status
- Issues and errors detected
- Sample results with relevance scores

### Performance Metrics
`backend/PERFORMANCE_METRICS.txt` - Performance benchmarking results:
- Single query latency (average, p95, p99)
- Batch retrieval latency for different batch sizes
- Comparison against NFR targets

### Logs
`backend/logs/retrieval.log` - Structured JSON logs with:
- Timestamp of all operations
- Query text and result count
- Latency and status
- Validation results and issues

## Troubleshooting

### "QDRANT_URL is not set" Error
**Solution:** Ensure `.env` file contains `QDRANT_URL=your-qdrant-cloud-url`

### "Connection failed" when querying Qdrant
**Solution:** Verify:
- Qdrant Cloud URL is reachable
- API key is valid
- Network connectivity is available
- Collection "textbook_embeddings" exists

### "Embedding dimension mismatch" Error
**Solution:** Verify:
- Cohere API key is valid
- Model is embed-english-v3.0 (1024-dimensional)
- Matches Spec-1 ingestion model

### Low relevance scores on test queries
**Solution:**
- Verify 26 vectors were successfully ingested (Spec-1)
- Check that metadata is correct (URL, title, chunk_index)
- Review validation report for specific issues
- Ensure test queries are semantically related to textbook content

## Performance Considerations

### Optimization Tips
- Use default k=5 for most queries (faster than k=20+)
- Batch queries when possible (fewer embedding API calls)
- Cache query embeddings if running multiple identical queries
- Use URL filtering to limit result scope when appropriate

### Scaling
- Service supports 100+ vectors without degradation
- Single machine can handle 10+ concurrent requests
- For higher concurrency, consider async refactoring (Phase 2)

## Next Steps

After successful Phase 1-3 validation:

1. **Phase 4:** Complete unit and integration tests
2. **Phase 5:** Execute production validation and benchmarking
3. **Phase 6:** Integrate retrieval service with chatbot (Spec-3)
4. **Phase 7:** Deploy to production environment

## References

- **Specification:** specs/3-data-retrieval/spec.md
- **Implementation Plan:** specs/3-data-retrieval/plan.md
- **Task Decomposition:** specs/3-data-retrieval/tasks.md
- **Qdrant Documentation:** https://qdrant.tech/documentation/
- **Cohere Documentation:** https://docs.cohere.com/

---

*Retrieval Service - Part of RAG System for Physical AI & Humanoid Robotics Educational Content*
