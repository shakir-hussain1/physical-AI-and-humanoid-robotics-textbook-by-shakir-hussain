# RAG Chatbot Backend - Implementation Guide

**Current Status**: Phase 1 (Setup) ✅ COMPLETE
**Total Tasks**: 109 (9 completed, 100 remaining)
**MVP Timeline**: 2-3 weeks (68 tasks for Phase 1-3 + critical Phase 6)
**Full Timeline**: 6-8 weeks (all 109 tasks)

---

## Quick Start

```bash
# 1. Setup environment
cp .env.example .env
# Edit .env with your API keys

# 2. Start Docker services
docker-compose up -d

# 3. Verify services
curl http://localhost:8000/health

# 4. Next: Execute Phase 2 (Foundational - 29 tasks)
```

---

## Phase Roadmap

### ✅ Phase 1: Setup (9 tasks) - COMPLETE

All project structure, dependencies, Docker, and basic configuration ready.

**What's Done**:
- ✓ requirements.txt with all dependencies
- ✓ .env.example with configuration template
- ✓ Dockerfile for production deployment
- ✓ docker-compose.yml for local development
- ✓ src/main.py FastAPI entrypoint
- ✓ src/config.py configuration management
- ✓ Complete directory structure

---

### ⏳ Phase 2: Foundational (29 tasks) - NEXT

Critical infrastructure that ALL user stories depend on.

**Key Deliverables**:
1. **Database Setup** (T010-T014)
2. **Models & Schemas** (T015-T017)
3. **Service Skeleton** (T018-T023)
4. **Validation & Error Handling** (T024-T027)
5. **Middleware** (T028-T032)
6. **API Routers** (T033-T038)

**Implementation Approach**:

```
1. Run Alembic migrations:
   - Create src/db/migrations/ (alembic init)
   - Add DDL from data-model.md to migrations/versions/
   - Run: alembic upgrade head

2. Create Pydantic schemas:
   - src/models/schemas.py: QueryRequest, QueryResponse, ErrorResponse, etc.
   - Copy structures from contracts/openapi.yaml

3. Create SQLAlchemy ORM models:
   - src/models/entities.py: DocumentORM, ChunkORM, QueryORM, etc.
   - Match tables from data-model.md

4. Create service skeleton:
   - src/services/retrieval.py: RetrieverService (stub methods)
   - src/services/generation.py: GenerationService (stub methods)
   - src/services/citations.py: CitationService (stub methods)
   - src/services/content_ingest.py: ContentIngestService (stub methods)
   - src/services/audit_logger.py: AuditLoggerService (stub methods)

5. Create middleware:
   - src/middleware/error_handler.py
   - src/middleware/logging_middleware.py
   - src/middleware/api_key_auth.py

6. Create API routers:
   - src/api/health.py (GET /health - DONE in Phase 1)
   - src/api/query.py (POST /chat/query - stub)
   - src/api/context.py (POST /chat/context-restricted - stub)
   - src/api/content.py (POST /content/ingest - stub)

7. Register routers in src/main.py
```

**Phase 2 Complete Criteria**:
- ✓ Docker-compose services running
- ✓ Database schema created
- ✓ GET /health returns 200 OK with service status
- ✓ All routers registered (no endpoints implemented yet)

---

### 🚀 Phase 3: User Story 1 - Student Queries (22 tasks)

**MVP Feature**: Students ask questions, get cited answers

**Implementation Order**:
1. **T039-T043**: Content Ingestion
   - Parse documents, chunk into 512-token units, embed with Cohere, index in Qdrant

2. **T044-T047**: Retrieval Service
   - Query validation, semantic search, hybrid ranking (85% Cohere + 15% BM25)
   - Confidence filtering (>0.60)

3. **T048-T051**: Generation Service
   - Build system prompt (RAG constraints)
   - Call Claude API, extract confidence

4. **T052-T055**: Citations Service
   - Inject citations into answer text
   - Extract and validate sources

5. **T056-T058**: Audit Logging
   - Log full operation trace

6. **T059**: Implement POST /chat/query endpoint
   - Orchestrate all services above

7. **T060**: Error handling for edge cases

**Phase 3 Complete Criteria**:
- ✓ POST /content/ingest uploads chapters
- ✓ POST /chat/query returns answers with citations
- ✓ Confidence scores ≥0.85 for relevant queries
- ✓ Out-of-scope queries explicitly rejected
- ✓ Audit logs capture full trace

**Example: Retrieval Service Implementation**

```python
# src/services/retrieval.py
from typing import List, Tuple
from qdrant_client import QdrantClient
from cohere import Client as CohereClient

class RetrieverService:
    def __init__(self, qdrant_url: str, cohere_key: str):
        self.qdrant = QdrantClient(url=qdrant_url)
        self.cohere = CohereClient(api_key=cohere_key)

    async def retrieve_chunks(
        self,
        query: str,
        top_k: int = 5
    ) -> List[dict]:
        """Retrieve top-K chunks using semantic search."""
        # 1. Embed query with Cohere
        embed_response = self.cohere.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = embed_response.embeddings[0]

        # 2. Search in Qdrant
        search_result = self.qdrant.search(
            collection_name="chunks",
            query_vector=query_embedding,
            limit=top_k
        )

        # 3. Extract chunks with scores
        chunks = []
        for result in search_result:
            chunks.append({
                "chunk_id": result.payload["chunk_id"],
                "text": result.payload["text"],
                "page_number": result.payload["page_number"],
                "section": result.payload["section"],
                "score": result.score
            })

        return chunks

    async def rank_by_confidence(
        self,
        chunks: List[dict],
        threshold: float = 0.60
    ) -> Tuple[List[dict], float]:
        """Apply hybrid ranking and confidence threshold."""
        # Filter by threshold
        filtered = [c for c in chunks if c["score"] >= threshold]

        if not filtered:
            raise LowConfidenceError(
                "No relevant content found. Confidence below threshold."
            )

        # Sort by score (descending)
        ranked = sorted(filtered, key=lambda x: x["score"], reverse=True)

        # Compute average confidence
        avg_confidence = sum(c["score"] for c in ranked) / len(ranked)

        return ranked, avg_confidence
```

---

### Phase 4-5: User Stories 2-3 (16 tasks)

**Phase 4**: Educator fact-checking (audit trails, grading)
**Phase 5**: Researcher passage-restricted mode

Execute after Phase 3 is complete and tested independently.

---

### Phase 6: Polish & Cross-Cutting (33 tasks)

1. **Testing** (T077-T084)
   - Unit tests for validation, retrieval, generation, citations
   - Integration tests for query flow, context flow, fact-checking
   - Contract tests for API endpoints

2. **Documentation** (T086-T088)
   - Update README.md
   - Create CONTRIBUTING.md
   - Finalize quickstart.md

3. **Performance** (T089-T092)
   - Profile and optimize latency
   - Optimize database queries

4. **Deployment** (T093-T109)
   - Monitoring setup
   - Docker verification
   - Production readiness

**Run Tests**:
```bash
# Unit tests
pytest tests/unit/ -v --cov=src

# Integration tests
pytest tests/integration/ -v

# All tests with coverage
pytest tests/ -v --cov=src --cov-report=html

# Verify coverage ≥80%
```

---

## Development Workflow

### For Each Task:

```
1. Read task description from tasks.md
2. Understand dependencies (look for [P] markers)
3. Implement (write code, tests if applicable)
4. Commit with task ID: `git commit -m "T### Task description"`
5. Update tasks.md: Mark as [x] and verify in git
6. Push to branch: `git push origin 1-rag-chatbot-backend`
```

### Parallel Execution:

Tasks marked `[P]` can run in parallel (different files, no dependencies).

```bash
# Example: Run all model creation tasks in parallel
# (In separate terminals or with make/task runner)
Terminal 1: Implement T015 (schemas.py)
Terminal 2: Implement T016 (entities.py)
Terminal 3: Implement T017 (audit.py)

# Then: Merge all into main branch
```

---

## Key Files Reference

| File | Purpose | Status |
|------|---------|--------|
| `specs/1-rag-chatbot-backend/spec.md` | Feature spec | ✅ Complete |
| `specs/1-rag-chatbot-backend/plan.md` | Architecture | ✅ Complete |
| `specs/1-rag-chatbot-backend/data-model.md` | Database schema | ✅ Complete |
| `specs/1-rag-chatbot-backend/contracts/openapi.yaml` | API contracts | ✅ Complete |
| `specs/1-rag-chatbot-backend/tasks.md` | Task breakdown (109 tasks) | ✅ Complete |
| `specs/1-rag-chatbot-backend/research.md` | Technical decisions | ✅ Complete |
| `backend/requirements.txt` | Dependencies | ✅ Phase 1 |
| `backend/Dockerfile` | Container image | ✅ Phase 1 |
| `backend/docker-compose.yml` | Dev environment | ✅ Phase 1 |
| `backend/src/main.py` | FastAPI app | ✅ Phase 1 |
| `backend/src/config.py` | Configuration | ✅ Phase 1 |

---

## Testing Strategy

### Phase 2 (After Foundational):
```bash
# Verify Docker services start
docker-compose up -d
docker-compose ps  # Should show all 3 services running

# Verify API health check
curl http://localhost:8000/health
# Expected: {"status": "healthy", "services": {...}}
```

### Phase 3 (After User Story 1):
```bash
# 1. Index sample content
curl -X POST http://localhost:8000/content/ingest \
  -H "Authorization: Bearer test-key" \
  -F "file=@sample_chapter.md" \
  -F "chapter_id=ch01"

# 2. Query the chatbot
curl -X POST http://localhost:8000/chat/query \
  -H "Authorization: Bearer test-key" \
  -H "Content-Type: application/json" \
  -d '{"query": "What is robot control?"}'

# 3. Verify response format
# Expected: {"answer": "...", "sources": [...], "confidence": 0.92, ...}
```

### Phase 6 (Testing):
```bash
# Run full test suite
pytest tests/ -v --cov=src --cov-report=html

# Check coverage
# Open htmlcov/index.html to view coverage report
```

---

## Checkpoints & Validation

**After Phase 1** (CURRENT): ✅
- ✓ Docker-compose up -d works
- ✓ Project structure complete

**After Phase 2** (Next Milestone):
- ✓ Database schema created
- ✓ GET /health returns 200
- ✓ All routers registered (can start up)

**After Phase 3** (MVP Ready):
- ✓ POST /content/ingest accepts files
- ✓ POST /chat/query returns answers with citations
- ✓ Confidence scores ≥0.85
- ✓ Out-of-scope rejection works
- **READY TO SHIP MVP**

**After Phase 6** (Production Ready):
- ✓ ≥80% test coverage
- ✓ All API contracts validated
- ✓ Performance targets met
- ✓ Docker build succeeds
- ✓ Monitoring configured

---

## Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| ModuleNotFoundError: No module named 'src' | Run from repo root, use `from src import ...` |
| PostgreSQL connection refused | `docker-compose ps` to verify service, check DATABASE_URL |
| Qdrant connection error | Verify `docker-compose up -d` ran successfully |
| API key auth fails | Check Authorization header format: `Bearer <key>` |
| Import errors in middleware | Ensure all __init__.py files exist |
| Tests not finding modules | Run pytest from repo root: `pytest tests/` |

---

## Next Steps

1. **Run Phase 2 setup**:
   ```bash
   # Create Alembic migrations
   cd backend
   python -m alembic init src/db/migrations
   ```

2. **Use tasks.md as task list**:
   - Open `specs/1-rag-chatbot-backend/tasks.md`
   - Read T010-T038 for Phase 2 tasks
   - Execute sequentially

3. **After Phase 2 complete**:
   - Proceed to Phase 3 (User Story 1)
   - Implement retrieval, generation, citations

4. **Test as you go**:
   - Verify each phase checkpoint before moving forward

---

## Resources

- **Architecture**: See `plan.md` for technical decisions and data model
- **Tasks**: See `tasks.md` for all 109 tasks with dependencies
- **API Specs**: See `contracts/openapi.yaml` for endpoint definitions
- **Setup**: See `quickstart.md` for deployment & testing

---

**You have everything you need to complete this project autonomously!**

Questions? Refer to:
- Specification: `specs/1-rag-chatbot-backend/spec.md`
- Technical decisions: `specs/1-rag-chatbot-backend/research.md`
- Data model: `specs/1-rag-chatbot-backend/data-model.md`
- Task list: `specs/1-rag-chatbot-backend/tasks.md`
