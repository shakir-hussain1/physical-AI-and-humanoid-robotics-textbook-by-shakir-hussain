# Quickstart: RAG Chatbot Backend

**Date**: 2025-12-16
**Status**: Phase 1 Complete

This guide walks you through local setup, indexing sample content, and making your first query.

---

## Prerequisites

- Python 3.11+
- Docker & Docker Compose
- Git
- Anthropic API key (Claude 3.5 Sonnet)
- Cohere API key (embeddings)

---

## 1. Local Setup with Docker Compose

### Clone and navigate

```bash
git clone <repo-url>
cd backend
```

### Environment Configuration

Copy the example environment file:

```bash
cp .env.example .env
```

Edit `.env` with your API keys:

```bash
# .env
ANTHROPIC_API_KEY=sk-ant-...
COHERE_API_KEY=cohere-...
DATABASE_URL=postgresql://postgres:postgres@postgres:5432/ragchatbot
QDRANT_URL=http://qdrant:6333
FASTAPI_ENV=development
LOG_LEVEL=INFO
```

### Start Services

```bash
docker-compose up -d
```

This starts:
- **FastAPI** (backend): http://localhost:8000
- **PostgreSQL** (audit logs): postgres://localhost:5432
- **Qdrant** (vector DB): http://localhost:6333

### Verify Services

```bash
# Check API health
curl http://localhost:8000/health

# Expected output:
# {"status": "healthy", "timestamp": "2025-12-16T...", "services": {...}}
```

---

## 2. Set Up Database Schema

Create tables in PostgreSQL:

```bash
# Connect to the database
docker-compose exec postgres psql -U postgres -d ragchatbot -f /docker-entrypoint-initdb.d/schema.sql
```

Or manually create using the DDL in `data-model.md`:

```bash
docker-compose exec postgres psql -U postgres -d ragchatbot << EOF
-- Copy full DDL from data-model.md and paste here
EOF
```

---

## 3. Install Python Dependencies (Local Development)

If you want to run the app locally (not in Docker):

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

---

## 4. Index Sample Book Content

The repo includes sample chapters in `sample_content/`:

```bash
ls sample_content/
# ch01_fundamentals.md
# ch02_control_systems.md
# ch03_sensors.md
```

### Upload via API

```bash
curl -X POST http://localhost:8000/content/ingest \
  -H "Authorization: Bearer test-api-key" \
  -F "file=@sample_content/ch01_fundamentals.md" \
  -F "chapter_id=ch01" \
  -F "metadata={\"author\": \"Shakir Hussain\", \"publication_date\": \"2025-01-01\"}"

# Expected: 202 Accepted
# {"ingest_id": "ingest-550e8400...", "status": "queued"}
```

Poll for completion:

```bash
curl http://localhost:8000/content/ingest/ingest-550e8400... \
  -H "Authorization: Bearer test-api-key"

# Expected: 200 OK when complete
# {"chunks_created": 24, "document_id": "..."}
```

Or use Python script:

```bash
python scripts/ingest_sample_content.py
```

---

## 5. Make Your First Query

### Via cURL

```bash
curl -X POST http://localhost:8000/chat/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer test-api-key" \
  -d '{"query": "What is hierarchical robot control?"}'
```

### Via Python

```python
import requests

response = requests.post(
    "http://localhost:8000/chat/query",
    headers={
        "Authorization": "Bearer test-api-key",
        "Content-Type": "application/json"
    },
    json={"query": "What is passive compliance in robotics?"}
)

print(response.json())
# Output:
# {
#   "answer_id": "550e8400-e29b-41d4-a716...",
#   "answer": "Passive compliance is... [Source: ...]",
#   "sources": [...],
#   "confidence": 0.92,
#   "confidence_level": "high",
#   "status": "success"
# }
```

### Via JavaScript (Docusaurus integration)

```javascript
async function askChatbot(query) {
  const response = await fetch('/api/chat/query', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${apiKey}`
    },
    body: JSON.stringify({ query })
  });
  return response.json();
}
```

---

## 6. Test Context-Restricted Mode

```bash
curl -X POST http://localhost:8000/chat/context-restricted \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer test-api-key" \
  -d '{
    "query": "What is proprioception in simple terms?",
    "selected_passage": "Proprioception is the ability of a robot to sense its own joint positions and velocities without external sensors. This is achieved through internal feedback loops that measure actuator commands and compare them with observed outcomes."
  }'
```

---

## 7. Run Tests

### Unit Tests

```bash
# Run all unit tests
pytest tests/unit/ -v

# Run with coverage
pytest tests/unit/ --cov=src --cov-report=html
# Open htmlcov/index.html to view coverage report
```

### Integration Tests

```bash
# Start services first
docker-compose up -d

# Run integration tests
pytest tests/integration/ -v
```

### Contract Tests (API)

```bash
# Verify API contracts match OpenAPI spec
pytest tests/contract/ -v
```

### All Tests

```bash
pytest tests/ -v --cov=src --cov-report=html
```

---

## 8. View Logs

### FastAPI Application Logs

```bash
docker-compose logs -f fastapi
```

### PostgreSQL Logs

```bash
docker-compose logs -f postgres
```

### Qdrant Logs

```bash
docker-compose logs -f qdrant
```

---

## 9. Fact-Checking Review Interface (Manual)

Query audit logs to sample answers for fact-checking:

```sql
-- Sample 5 random answers for review (higher confidence first)
SELECT
  a.answer_id,
  q.user_query_text,
  a.answer_text,
  a.confidence_score,
  al.timestamp
FROM answers a
JOIN queries q ON a.query_id = q.query_id
JOIN audit_logs al ON a.answer_id = al.answer_id
WHERE a.status = 'pending_review'
  AND a.generated_at > NOW() - INTERVAL '1 day'
ORDER BY a.confidence_score DESC
LIMIT 5;

-- Mark answer as approved after review
UPDATE answers
SET status = 'approved'
WHERE answer_id = 'some-uuid';

-- Or record formal grade
INSERT INTO fact_check_grades (
  answer_id, reviewer_id, accuracy_score,
  hallucination_detected, approved_for_production, reviewed_at
) VALUES (
  'some-uuid', 'shakir@example.com', 100,
  FALSE, TRUE, NOW()
);
```

---

## 10. Deployment Checklist

Before production deployment:

- [ ] Environment variables set (API keys, DB URL, Qdrant URL)
- [ ] PostgreSQL database initialized and tested
- [ ] Qdrant vector DB initialized and tested
- [ ] Sample content indexed successfully
- [ ] All tests passing (unit, integration, contract)
- [ ] Coverage >= 80%
- [ ] API endpoints responding correctly
- [ ] Health check endpoint /health returns "healthy"
- [ ] API key authentication working
- [ ] Rate limiting configured
- [ ] Logging configured (structured JSON)
- [ ] Monitoring/alerting set up (latency, hallucination rate, retrieval precision)
- [ ] Rollback procedure documented (hallucination >5%)
- [ ] Fact-checking review process established (20–30 samples/month)

---

## 11. Directory Structure

```
backend/
├── src/
│   ├── main.py              # FastAPI app entrypoint
│   ├── config.py            # Configuration
│   ├── models/              # Pydantic schemas + domain entities
│   ├── services/            # Business logic (retrieval, generation, etc.)
│   ├── api/                 # REST endpoints
│   ├── db/                  # Database clients + migrations
│   └── utils/               # Validation, constants, tracing
├── tests/
│   ├── unit/                # Unit tests
│   ├── integration/         # Integration tests
│   ├── contract/            # API contract tests
│   └── fixtures/            # Test data
├── sample_content/          # Sample chapters for testing
├── scripts/                 # Utility scripts
├── Dockerfile               # Production image
├── docker-compose.yml       # Local dev services
├── requirements.txt         # Python dependencies
├── .env.example             # Environment template
└── README.md                # Main documentation
```

---

## 12. Troubleshooting

### Qdrant Connection Error

```
Error: Failed to connect to Qdrant at http://qdrant:6333
```

**Fix**: Ensure Qdrant is running and accessible:

```bash
docker-compose ps  # Verify container is running
curl http://localhost:6333/health  # Check health
```

### PostgreSQL Connection Error

```
Error: psycopg2.OperationalError: could not connect to server
```

**Fix**: Check PostgreSQL is running and credentials match `.env`:

```bash
docker-compose exec postgres psql -U postgres -c "SELECT 1"
```

### API Key Authorization Failure

```
401 Unauthorized: Invalid API key
```

**Fix**: Ensure Authorization header is correct:

```bash
curl -H "Authorization: Bearer test-api-key" http://localhost:8000/health
```

### Query Timeout (504)

```
504 Gateway Timeout: Query processing timed out
```

**Fix**: Query was too complex or system overloaded. Retry with simpler query:

```bash
curl -X POST http://localhost:8000/chat/query \
  -H "Authorization: Bearer test-api-key" \
  -d '{"query": "What is control?"}'  # Simpler query
```

### Low Confidence Answer

```
"confidence": 0.52,
"confidence_level": "low"
```

**Note**: Low-confidence answers are still returned but flagged. Retry with different phrasing or check if content is indexed.

---

## Next Steps

1. **Indexing**: Add your full book content via `/content/ingest` endpoint
2. **Testing**: Verify retrieval and generation quality on real queries
3. **Monitoring**: Set up dashboards for latency, accuracy, hallucination rate
4. **Fact-Checking**: Establish monthly review cycle (20–30 samples)
5. **Deployment**: Deploy to production (Docker, Kubernetes, or VM)
6. **Integration**: Connect Docusaurus plugin to backend via API
7. **Monitoring & Alerts**: Configure thresholds for p95 latency (>3s), accuracy drop (>2%), hallucination (>5%)

---

**For full API documentation, see**: `contracts/openapi.yaml` (OpenAPI 3.0 spec)

**For data model details, see**: `data-model.md`

**For technical decisions, see**: `research.md`
