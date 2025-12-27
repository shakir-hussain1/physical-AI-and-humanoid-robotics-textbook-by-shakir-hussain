# RAG Spec-4: Backend–Frontend Integration via FastAPI - Implementation Plan

## Version
**Plan Version:** 1.0.0
**Created:** 2025-12-26
**Last Updated:** 2025-12-26

## Feature Overview
**Feature:** Backend–Frontend Integration via FastAPI
**Module Alignment:** HTTP API layer for RAG system
**Branch:** 5-fastapi-integration
**Dependencies:**
- Spec-3 (RAG Agent) - AgentOrchestrator for query processing
- Spec-2 (Retrieval Service) - RetrievalService for search
- FastAPI >= 0.100.0
- Pydantic >= 2.0.0
- Python 3.10+

## Constitutional Alignment
This plan aligns with the following constitutional principles:
- PRINCIPLE_1: Technical Excellence and Accuracy
- PRINCIPLE_4: Practical Application
- PRINCIPLE_6: Collaborative Intelligence
- PRINCIPLE_9: RAG Chatbot Integration and Knowledge Accessibility
- PRINCIPLE_10: Deployment and Platform Accessibility

## Scope and Requirements

### In Scope
- FastAPI application initialization with proper configuration
- 4 API endpoints (query, context-query, retrieve, health)
- Pydantic request/response schemas with validation
- CORS middleware for frontend access
- Error handling with appropriate HTTP status codes
- Logging and structured responses
- Integration with Spec-3 agent and Spec-2 retrieval
- Request/response validation
- Conversation history support
- Selected-text context feature

### Out of Scope
- User authentication or session management
- Rate limiting or request throttling
- Production hardening (HTTPS, load balancing)
- Database for storing conversations
- Analytics or usage tracking
- WebSocket support for real-time updates
- GraphQL API
- API documentation generation (manual docs sufficient)

### External Dependencies
- **Spec-3 (RAG Agent):** AgentOrchestrator class with query() method
- **Spec-2 (Retrieval Service):** RetrievalService class with search() method
- **FastAPI:** Modern async web framework
- **Pydantic:** Data validation and serialization
- **Python-dotenv:** Environment variable management
- **OpenAI API:** Via Spec-3 agent

## Key Decisions and Rationale

### Options Considered

**Option 1: Minimal API (Query endpoint only)**
- Description: Single endpoint for queries
- Trade-offs: Simple but missing functionality (retrieval-only, health checks)
- Status: Rejected - insufficient for full integration

**Option 2: Complete REST API with 4 Endpoints (Selected)**
- Description: Query, context-query, retrieve, and health endpoints
- Trade-offs: More comprehensive but reasonable scope
- Status: Selected - balances completeness with MVP timeline

**Option 3: Full-Featured API with Streaming/WebSockets**
- Description: Add streaming responses, WebSocket support
- Trade-offs: More complex, exceeds MVP scope
- Status: Deferred - future enhancement for Spec-5

### Selected Approach
**Decision:** Complete REST API with 4 endpoints, synchronous implementation

**Rationale:**
- Covers all user stories from specification
- Clear separation of concerns (query vs. search vs. health)
- Synchronous endpoints simpler for MVP (can upgrade to async)
- Stateless design enables future scaling
- Compatible with frontend frameworks (fetch/axios)

**Trade-offs:**
- Synchronous implementation (async upgrade in future)
- No request streaming (full response buffering)
- No persistent conversation storage
- No advanced caching

### Principles
- **Measurable:** Success criteria include latency targets (P95 < 6s) and uptime (99.5%)
- **Reversible:** Synchronous endpoints can be upgraded to async without breaking API
- **Smallest Viable Change:** Only implement what's required for frontend integration

## Implementation Strategy

### Phase 1: Foundation & Configuration (Days 1-2)

**Objective:** Set up FastAPI application structure and configuration

- [ ] **P1.1:** Create backend/src/api/ package structure
  - Create `__init__.py` with exports
  - Create `main.py` for FastAPI app initialization
  - Create `config.py` for application configuration
  - Purpose: Organize API code and load configuration from environment

- [ ] **P1.2:** Initialize FastAPI application
  - Create FastAPI() instance
  - Set up lifespan/startup/shutdown events
  - Configure base path and root endpoint
  - Add CORS middleware
  - Purpose: Ready for endpoints

- [ ] **P1.3:** Create configuration management
  - Load from environment variables (OpenAI key, agent config)
  - Validate all required settings on startup
  - Provide reasonable defaults
  - Purpose: Environment-based configuration without hardcoding

- [ ] **P1.4:** Initialize RAG agent and retrieval service
  - Import and instantiate AgentOrchestrator from Spec-3
  - Import and instantiate RetrievalService from Spec-2
  - Make available to endpoints via dependency injection
  - Purpose: Make backend services available to API endpoints

**Acceptance Criteria for Phase 1:**
- [ ] FastAPI app starts without errors
- [ ] Configuration loads from environment
- [ ] Agent and retrieval services initialized successfully
- [ ] CORS configured for frontend origin
- [ ] Root endpoint (GET /) returns welcome message

### Phase 2: Data Models & Validation (Days 3-4)

**Objective:** Define Pydantic schemas for request/response validation

- [ ] **P2.1:** Create request models in backend/src/api/models.py
  - QueryRequest: query (str), conversation_history (List[Message]), user_role (str)
  - MessageModel: role (str), content (str), timestamp (str)
  - RetrievalRequest: query (str), k (int)
  - ContextQueryRequest: selected_text (str), query (str), conversation_history (List)
  - Validation: non-empty strings, valid lengths, field types

- [ ] **P2.2:** Create response models in backend/src/api/models.py
  - SourceInfo: url, page_title, relevance_score, chunk_index
  - QueryResponse: answer, sources, confidence, metadata, timestamp
  - RetrievalResult: id, text, similarity_score, metadata
  - HealthResponse: status, timestamp, components (Dict[str, str])
  - ErrorResponse: error_code, message, request_id, details (optional)

- [ ] **P2.3:** Add validation logic
  - Query validation: non-empty, max 10K chars
  - History validation: array of valid Message objects
  - URL validation: properly formatted URLs in sources
  - Relevance scores: floats in [0, 1]
  - HTTP status codes: standard REST codes (200, 400, 502, 503, 504)

**Acceptance Criteria for Phase 2:**
- [ ] All Pydantic models created with validation
- [ ] Request validation rejects invalid input with 400 errors
- [ ] Response schemas match specification
- [ ] Type hints present on all fields
- [ ] Documentation strings on models

### Phase 3: Query Endpoints (Days 5-6)

**Objective:** Implement main query endpoints

- [ ] **P3.1:** Implement POST /api/query endpoint
  - Accept QueryRequest (query, history, role)
  - Call AgentOrchestrator.query() with parameters
  - Validate response before returning
  - Handle timeout (30s): return 504 Gateway Timeout
  - Handle agent errors: return 502 Bad Gateway
  - Return QueryResponse with 200 OK
  - File: backend/src/api/endpoints/query.py

- [ ] **P3.2:** Implement POST /api/query/with-context endpoint
  - Accept ContextQueryRequest (selected_text, query, history)
  - Prepend selected text to context before calling agent
  - Call AgentOrchestrator.query() with enhanced prompt
  - Return QueryResponse with improved grounding
  - Handle same errors as /api/query
  - File: backend/src/api/endpoints/query.py

- [ ] **P3.3:** Add request validation and error handling
  - Validate query non-empty and <= 10K chars
  - Validate history is array of Message objects
  - Validate selected_text non-empty (for context endpoint)
  - Return 400 Bad Request with field-specific errors
  - Log validation failures

- [ ] **P3.4:** Add response formatting and metadata
  - Calculate latency_ms (query processing time)
  - Include grounding status in metadata
  - Add follow-up suggestions from agent
  - Ensure timestamp in ISO 8601 format
  - Validate response before returning

**Acceptance Criteria for Phase 3:**
- [ ] POST /api/query accepts valid requests
- [ ] POST /api/query/with-context adds selected text context
- [ ] Invalid input returns 400 Bad Request
- [ ] Timeout returns 504 Gateway Timeout
- [ ] Successful queries return 200 OK with QueryResponse
- [ ] Response includes answer, sources, confidence, metadata
- [ ] Latency tracked and included in response

### Phase 4: Search & Health Endpoints (Days 7-8)

**Objective:** Implement retrieval and health check endpoints

- [ ] **P4.1:** Implement POST /api/retrieve endpoint
  - Accept RetrievalRequest (query, k)
  - Call RetrievalService.search(query, k=k)
  - Return raw results without LLM generation
  - Handle validation: query non-empty, k in [1, 20]
  - Return list of RetrievalResult objects with 200 OK
  - Handle retrieval errors: return 503 Service Unavailable
  - File: backend/src/api/endpoints/retrieve.py

- [ ] **P4.2:** Implement GET /api/health endpoint
  - Check agent is initialized
  - Check retrieval service is available
  - Check LLM API connectivity
  - Return HealthResponse with component status
  - Response time target: < 100ms
  - Return 200 OK regardless of component status
  - File: backend/src/api/endpoints/health.py

- [ ] **P4.3:** Add performance optimization to health check
  - Cache health results for 30 seconds
  - Don't recheck every request
  - Ensure < 100ms response time
  - Update cache on background timer

**Acceptance Criteria for Phase 4:**
- [ ] POST /api/retrieve returns raw search results
- [ ] GET /api/health returns component status
- [ ] Health check responds in < 100ms
- [ ] Retrieval endpoint validates input (k in range)
- [ ] Errors handled appropriately (503 for unavailable)

### Phase 5: Middleware & Error Handling (Days 9-10)

**Objective:** Implement cross-cutting concerns

- [ ] **P5.1:** Implement CORS middleware
  - Allow requests from frontend origin (from env var)
  - Allow credentials in cross-origin requests
  - Set appropriate headers (Content-Type, Authorization)
  - Max age: 86400 seconds (1 day)
  - File: backend/src/api/middleware.py

- [ ] **P5.2:** Implement error handling middleware
  - Catch all exceptions and return standard error response
  - Include request_id for debugging
  - Log errors with context (endpoint, method, status)
  - Return ErrorResponse with appropriate HTTP status
  - Distinguish validation errors (400) from server errors (500)
  - File: backend/src/api/errors.py

- [ ] **P5.3:** Implement request/response logging
  - Log all requests with: method, path, status, latency
  - Log all responses with: status_code, response_time_ms
  - Include request_id for tracing
  - Log errors with full context and stack trace
  - Use structured JSON logging

- [ ] **P5.4:** Add request context tracking
  - Generate unique request_id for each request
  - Include request_id in all logs
  - Return request_id in error responses for debugging
  - Track timing: total latency, agent latency, retrieval latency

**Acceptance Criteria for Phase 5:**
- [ ] CORS headers properly set
- [ ] All errors return standard error response format
- [ ] All requests/responses logged
- [ ] Request ID included in logs and error responses
- [ ] Latency tracking accurate

### Phase 6: Integration Testing (Days 11-12)

**Objective:** Test end-to-end flows and integration

- [ ] **P6.1:** Create integration tests
  - Test /api/query with valid request
  - Test /api/query/with-context with selected text
  - Test /api/retrieve with various k values
  - Test /api/health endpoint
  - Test error cases (invalid input, timeout, unavailable)
  - File: backend/tests/integration/test_api_endpoints.py

- [ ] **P6.2:** Test error scenarios
  - Invalid JSON in request body (400)
  - Missing required fields (400)
  - Query too long (400)
  - Timeout (504)
  - Retrieval unavailable (503)
  - LLM error (502)
  - Each error returns proper HTTP status

- [ ] **P6.3:** Test concurrent requests
  - 10+ concurrent queries
  - No memory leaks
  - Proper response ordering
  - No cross-contamination between requests

- [ ] **P6.4:** Test conversation history
  - Pass conversation history in requests
  - Verify agent receives history
  - Verify follow-up questions use context
  - Test history truncation if needed

**Acceptance Criteria for Phase 6:**
- [ ] All integration tests passing
- [ ] Error cases handled correctly
- [ ] 10+ concurrent requests work
- [ ] Conversation history properly handled
- [ ] Performance < 6s p95 latency

### Phase 7: Documentation & Deployment (Days 13-14)

**Objective:** Document API and prepare for deployment

- [ ] **P7.1:** Create API documentation
  - Document all 4 endpoints (request/response examples)
  - Document error codes and messages
  - Document configuration (environment variables)
  - Document running locally
  - File: backend/README_API.md

- [ ] **P7.2:** Create deployment guide
  - How to configure for production
  - Environment variables required
  - Running with gunicorn/uvicorn
  - Health check for load balancers
  - File: backend/DEPLOYMENT.md

- [ ] **P7.3:** Create startup/shutdown procedures
  - Graceful shutdown (complete pending requests)
  - Cleanup resources
  - Restart procedures
  - Monitoring integration

- [ ] **P7.4:** Performance validation
  - Run load tests with Apache JMeter or similar
  - Verify P95 latency < 6s
  - Verify health check < 100ms
  - Verify 10+ concurrent requests work
  - Document results

**Acceptance Criteria for Phase 7:**
- [ ] API documentation complete
- [ ] Deployment guide complete
- [ ] Performance targets met
- [ ] Graceful shutdown working
- [ ] Ready for production deployment

## Architectural Design

### Architecture Diagram

```
FastAPI Server
    ↓
[Request Validation] (Pydantic)
    ↓
[Routing]
├─ POST /api/query
│   ├─ Validate QueryRequest
│   ├─ Call AgentOrchestrator.query()
│   ├─ Format QueryResponse
│   └─ Return 200 OK
│
├─ POST /api/query/with-context
│   ├─ Prepend selected_text to context
│   ├─ Call AgentOrchestrator.query()
│   └─ Return 200 OK
│
├─ POST /api/retrieve
│   ├─ Validate RetrievalRequest
│   ├─ Call RetrievalService.search()
│   └─ Return 200 OK with results
│
└─ GET /api/health
    ├─ Check agent ready
    ├─ Check retrieval available
    └─ Return 200 OK with status
    ↓
[CORS Middleware]
    ↓
[Error Handling]
    ↓
JSON Response → Frontend
```

### File Structure

```
backend/src/api/
├── __init__.py              # Public exports
├── main.py                  # FastAPI app initialization
├── config.py                # Configuration management
├── models.py                # Pydantic request/response schemas
├── middleware.py            # CORS, logging middleware
├── errors.py                # Error handling
├── validators.py            # Input validation logic
└── endpoints/
    ├── __init__.py
    ├── query.py             # POST /api/query, /api/query/with-context
    ├── retrieve.py          # POST /api/retrieve
    └── health.py            # GET /api/health
```

### Data Models

**QueryRequest:**
```python
class QueryRequest(BaseModel):
    query: str  # non-empty, max 10K chars
    conversation_history: Optional[List[Message]] = None
    user_role: str = "student"  # one of: student, teacher, researcher
```

**QueryResponse:**
```python
class QueryResponse(BaseModel):
    answer: str
    sources: List[SourceInfo]  # top-3 sources
    confidence: str  # high, medium, low
    metadata: Dict  # {latency_ms, grounding, follow_ups}
    timestamp: str  # ISO 8601
```

**HealthResponse:**
```python
class HealthResponse(BaseModel):
    status: str  # healthy, degraded, error
    timestamp: str
    components: Dict[str, str]  # {agent: ready, retrieval: ready, llm: ready}
```

## Integration Points

### With Spec-3 (RAG Agent)
- **Uses:** AgentOrchestrator class and query() method
- **Passes:** query_text, conversation_history, user_role
- **Receives:** AgentResponse with answer, sources, confidence, metadata
- **Error handling:** Catches agent exceptions, returns 502 Bad Gateway

### With Spec-2 (Retrieval Service)
- **Uses:** RetrievalService class and search() method (via agent and direct)
- **Passes:** query text, k (number of results)
- **Receives:** List of chunks with metadata
- **Error handling:** Returns 503 Service Unavailable if unavailable

### With Frontend Application
- **Protocol:** HTTP/1.1 with JSON
- **CORS:** Enabled for frontend origin
- **Authentication:** None required (MVP)
- **Conversation:** Client maintains history, sends with each request

## Non-Functional Requirements

### Performance
- **Query endpoint P95 latency:** < 6 seconds (includes agent processing)
- **Retrieval endpoint latency:** < 1 second
- **Health check latency:** < 100 milliseconds
- **Concurrent users:** Support 10+ simultaneous requests

### Reliability
- **Uptime:** 99.5% availability
- **Graceful shutdown:** Complete pending requests before stopping
- **Error recovery:** Automatic retry on transient failures
- **Connection pooling:** Reuse connections to external services

### Scalability
- **Stateless design:** No in-memory state between requests
- **Horizontal scaling:** Ready for deployment behind load balancer (future)
- **Memory efficiency:** < 500MB per process

## Success Criteria

### Functional Success
- ✅ 4 endpoints implemented and working
- ✅ All requests properly validated
- ✅ Responses match documented schema
- ✅ Error handling works for all cases
- ✅ CORS configured for frontend

### Performance Success
- ✅ Query endpoint P95 < 6 seconds
- ✅ Health check < 100ms
- ✅ No memory leaks under load
- ✅ 10+ concurrent requests handled

### Integration Success
- ✅ Frontend can send queries to /api/query
- ✅ Frontend can use selected-text feature
- ✅ Frontend can check service health
- ✅ Frontend can search with /api/retrieve
- ✅ Conversation history properly maintained

## Risks and Mitigation

### Risk 1: Agent Timeout (Probability: High)
- **Impact:** User-facing 504 errors
- **Mitigation:**
  - 30-second timeout implemented
  - Clear error message to user
  - Monitor latency metrics
  - Optimize agent if needed

### Risk 2: Retrieval Unavailability (Probability: Medium)
- **Impact:** API returns 503 errors
- **Mitigation:**
  - Health check detects unavailability
  - Clear error message to frontend
  - Retry guidance in error response

### Risk 3: Invalid Agent Response (Probability: Low)
- **Impact:** API returns invalid JSON
- **Mitigation:**
  - Validate response schema
  - Log unexpected formats
  - Return 500 error with details

## Next Steps

After implementation:
1. Create `/sp.tasks` task decomposition for granular execution
2. Execute `/sp.implement` to build all components
3. Run integration tests and performance validation
4. Deploy to production environment

---

*RAG Spec-4: Backend–Frontend Integration via FastAPI*
*Implementation plan provides clear roadmap for API development*
