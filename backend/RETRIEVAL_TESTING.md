# Retrieval Service Testing Guide

## Overview

This guide explains the test query definitions, how to run tests, and how to interpret results.

## Test Query Suite (8-10 Queries)

### Query Definitions

#### Query 1: "ROS2" (Keyword - High Relevance)
- **Category:** keyword
- **Expected Domain:** robotics communication
- **Expected Modules:** module-1-ros2
- **Expected Keywords:** ROS2, middleware, DDS, publication, subscription
- **Expected Relevance:** High
- **Purpose:** Validate retrieval of ROS2 fundamentals content

#### Query 2: "simulation" (Keyword - High Relevance)
- **Category:** keyword
- **Expected Domain:** digital twin simulation
- **Expected Modules:** module-2-digital-twin
- **Expected Keywords:** Gazebo, simulation, physics, environment
- **Expected Relevance:** High
- **Purpose:** Validate retrieval of simulation content

#### Query 3: "robot communication middleware" (Phrase - High Relevance)
- **Category:** phrase
- **Expected Domain:** robotics
- **Expected Modules:** module-1-ros2
- **Expected Keywords:** ROS2, communication, middleware, publish, subscribe
- **Expected Relevance:** High
- **Purpose:** Validate phrase-based retrieval of communication concepts

#### Query 4: "digital twin virtual environment" (Phrase - High Relevance)
- **Category:** phrase
- **Expected Domain:** simulation
- **Expected Modules:** module-2-digital-twin
- **Expected Keywords:** Gazebo, digital twin, environment, simulation
- **Expected Relevance:** High
- **Purpose:** Validate phrase-based retrieval of simulation concepts

#### Query 5: "how to implement perception pipeline for humanoid robot" (Complex - Medium Relevance)
- **Category:** complex
- **Expected Domain:** perception and sensing
- **Expected Modules:** module-3-isaac
- **Expected Keywords:** perception, sensor, Isaac, vision, pipeline
- **Expected Relevance:** Medium
- **Purpose:** Validate complex multi-part query understanding

#### Query 6: "large language model for robot behavior planning" (Complex - Medium Relevance)
- **Category:** complex
- **Expected Domain:** AI and behavior generation
- **Expected Modules:** module-4-vla
- **Expected Keywords:** language model, behavior, planning, VLA
- **Expected Relevance:** Medium
- **Purpose:** Validate understanding of AI-based behavior planning

#### Query 7: "a" (Edge Case - Low Relevance)
- **Category:** edge_case
- **Expected Domain:** general
- **Expected Modules:** module-1-ros2, module-2-digital-twin
- **Expected Relevance:** Low
- **Purpose:** Validate handling of very short queries

#### Query 8: "ROS 2.0 @learning" (Edge Case - Medium Relevance)
- **Category:** edge_case
- **Expected Domain:** robotics
- **Expected Modules:** module-1-ros2
- **Expected Keywords:** ROS, learning, module
- **Expected Relevance:** Medium
- **Purpose:** Validate handling of special characters and version numbers

## Running Tests

### Unit Tests

Test the embedding service and validation logic:

```bash
# Run all unit tests
pytest backend/tests/retrieval/

# Run specific test file
pytest backend/tests/retrieval/test_embedding_service.py -v

# Run specific test
pytest backend/tests/retrieval/test_embedding_service.py::TestEmbeddingService::test_embed_query_returns_1024_dims -v
```

**Expected Results:**
- All embedding tests should pass (dimension verification, edge case handling)
- All validation tests should pass (metadata completeness, consistency checks)
- All retrieval tests should pass (ranking, scoring validation)

### Validation Tests

Test the query suite structure and coverage:

```bash
# Run validation tests
pytest backend/tests/validation/ -v

# Check test query suite consistency
pytest backend/tests/validation/test_retrieval_validation.py::TestQuerySuite -v

# Check module coverage
pytest backend/tests/validation/test_retrieval_validation.py::TestQuerySuite::test_test_query_suite_covers_all_modules -v
```

**Expected Results:**
- Test suite validation should pass (8+ queries, all categories covered)
- Module coverage should pass (queries covering all 4 modules)
- All query structure checks should pass

### Integration Tests

Test end-to-end retrieval workflow (requires Qdrant access):

```bash
# Run integration tests
pytest backend/tests/integration/ -v

# Run specific integration test
pytest backend/tests/integration/test_retrieval_workflow.py -v
```

**Expected Results:**
- End-to-end query → embedding → search → validation should succeed
- Deterministic retrieval (same query → same results) should pass
- Metadata integrity checks should pass
- Error handling for Qdrant failures should be graceful

## Full Validation Suite

### Running Full Validation

Execute all test queries with detailed reporting:

```bash
python -m backend.src.retrieval.cli run-validation
```

**Output:**
- Console: Human-readable validation report
- File: `backend/VALIDATION_RESULTS.txt` - Complete validation report
- Logs: `backend/logs/retrieval.log` - Detailed operation logs

### Interpreting Results

#### Pass Rate Thresholds
- **✅ PASS:** ≥ 80% of queries return relevant results
- **⚠️ WARNING:** 60-80% of queries return relevant results
- **❌ FAIL:** < 60% of queries return relevant results

#### Latency Benchmarks
- **Excellent:** Single query < 200ms
- **Good:** Single query 200-500ms
- **Acceptable:** Single query 500ms-1s
- **Poor:** Single query > 1s

#### Metadata Validation
- **✅ PASS:** 100% of results have complete metadata
- **⚠️ WARNING:** 95-100% metadata completeness
- **❌ FAIL:** < 95% metadata completeness

#### Content Verification
- **✅ PASS:** 100% of URLs accessible
- **⚠️ WARNING:** 90-100% URL accessibility
- **❌ FAIL:** < 90% URL accessibility

## Performance Benchmarking

### Single Query Benchmark

```bash
# Test single query performance
python -m backend.src.retrieval.cli search "ROS2" -k 5
```

**Metrics to Record:**
- Latency (ms): Time from query submission to results
- Result Count: Number of results returned
- Top Result Score: Similarity score of best match

**Expected Performance:**
- Latency: 200-500ms (p95 < 500ms, p99 < 1000ms)
- Result Count: 5 (for k=5)
- Top Result Score: > 0.5 for keyword queries

### Batch Benchmark

```bash
# Test batch query performance
python -c "
from backend.src.retrieval.retrieval_service import RetrievalService
from backend.src.retrieval.config import RetrievalConfig

config = RetrievalConfig.load_from_env()
service = RetrievalService(config)

queries = ['ROS2', 'simulation', 'perception', 'behavior planning', 'communication', 'embedding', 'sensor', 'physics', 'gazebo', 'humanoid']
results = service.batch_search(queries)
print(f'Batch of 10 queries: {sum(r[\"latency_ms\"] for r in results):.2f}ms total')
"
```

**Expected Performance:**
- 10 queries total latency: < 5 seconds
- Per-query average: < 500ms

## Debugging Failed Tests

### Metadata Issues
If metadata validation fails:
1. Check `VALIDATION_RESULTS.txt` for specific missing fields
2. Verify Spec-1 ingestion stored all metadata (url, page_title, chunk_index)
3. Ensure Qdrant collection has correct structure

### Low Relevance Scores
If similarity scores are low:
1. Verify query text matches textbook content domain
2. Check that Cohere model is embed-english-v3.0 (1024-dim)
3. Ensure textbook vectors were ingested correctly (Spec-1)
4. Review logs for embedding generation errors

### Connection Failures
If Qdrant connections fail:
1. Verify QDRANT_URL and QDRANT_API_KEY in .env
2. Test URL is reachable: `curl https://your-qdrant-url/health`
3. Check API key has proper permissions
4. Verify collection "textbook_embeddings" exists

### Timing Issues
If tests timeout:
1. Increase RETRIEVAL_TIMEOUT_SECONDS in .env (default: 30)
2. Check Qdrant Cloud availability (performance may vary)
3. Check Cohere API rate limits (free tier: 5 req/min)
4. Consider batch requests instead of sequential

## Expected Test Results Summary

| Test Category | Total | Expected Pass | Threshold |
|---|---|---|---|
| Unit Tests | 8 | 8/8 (100%) | ≥ 95% |
| Validation Tests | 10 | 10/10 (100%) | ≥ 95% |
| Integration Tests | 4 | 4/4 (100%) | ≥ 90% |
| Test Queries | 8 | 7/8 (87.5%) | ≥ 80% |
| **Total** | **30** | **29/30 (96.7%)** | **≥ 85%** |

## Continuous Testing

### CI/CD Integration

For GitHub Actions or similar:

```yaml
- name: Run Retrieval Tests
  run: |
    pytest backend/tests/retrieval/ -v --junit-xml=test-results.xml
    pytest backend/tests/validation/ -v --junit-xml=test-results.xml
    pytest backend/tests/integration/ -v --junit-xml=test-results.xml
    python -m backend.src.retrieval.cli run-validation > validation-report.txt
```

### Test Coverage

Ensure adequate code coverage:

```bash
# Generate coverage report
pytest backend/tests/ --cov=backend.src.retrieval --cov-report=html

# View coverage
open htmlcov/index.html
```

**Target Coverage:** ≥ 80% for core retrieval modules

## Test Maintenance

### Adding New Test Queries

To add more test queries:

1. Edit `backend/src/retrieval/test_queries.py`
2. Add new `TestQuery` instance to `TEST_QUERIES` list
3. Ensure all required fields are populated
4. Run validation: `pytest backend/tests/validation/ -v`
5. Execute full validation: `python -m backend.src.retrieval.cli run-validation`

### Updating Expected Outcomes

If test expectations change:

1. Review `backend/tests/validation/test_retrieval_validation.py`
2. Update module/keyword expectations
3. Re-run all tests to verify new baselines
4. Update this documentation

---

*Testing Guide for RAG Data Retrieval & Pipeline Validation Service*
