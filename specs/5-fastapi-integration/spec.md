# RAG Spec-4: Backend–Frontend Integration via FastAPI

## Version
**Spec Version:** 1.0.0
**Created:** 2025-12-26
**Last Updated:** 2025-12-26

## Feature Overview
**Feature:** Backend–Frontend Integration via FastAPI
**Module Alignment:** HTTP API layer for RAG system
**Target Audience:** Full-stack developers integrating the RAG backend with the book frontend

## Constitutional Alignment
This specification aligns with the following constitutional principles:
- PRINCIPLE_1: Technical Excellence and Accuracy
- PRINCIPLE_4: Practical Application
- PRINCIPLE_6: Collaborative Intelligence
- PRINCIPLE_9: RAG Chatbot Integration and Knowledge Accessibility
- PRINCIPLE_10: Deployment and Platform Accessibility

## Problem Statement

The RAG agent (Spec-3) and retrieval service (Spec-2) provide powerful backend functionality, but there is no HTTP API layer to expose these capabilities to frontend applications. Without FastAPI integration, frontend applications cannot:

1. **Send user queries** to the agent and receive answers
2. **Stream context-aware responses** with proper error handling
3. **Provide selected-text input** from the textbook to the agent
4. **Handle real-time interactions** with proper request/response validation
5. **Integrate with frontend frameworks** through standard REST endpoints

This specification defines the HTTP API layer that exposes the agent and retrieval functionality through FastAPI.

## Requirements

### Functional Requirements

**REQ-1: Query Endpoint**
- Endpoint: `POST /api/query`
- Input: User query text, optional conversation history, user role
- Processing: Forward to RAG agent, validate response
- Output: Structured JSON response with answer, sources, confidence
- Error handling: Return appropriate HTTP status codes with error messages

**REQ-2: Context Query Endpoint (for selected text)**
- Endpoint: `POST /api/query/with-context`
- Input: Selected text from document, user query, conversation history
- Processing: Prepend selected text to context, call agent
- Output: Response with enhanced grounding from selected text
- Use case: User highlights relevant text and asks follow-up question

**REQ-3: Health Check Endpoint**
- Endpoint: `GET /api/health`
- Output: Service status, component health (agent, retrieval, LLM)
- Purpose: Enable frontend to detect service availability
- Response format: JSON with status, timestamp, component details

**REQ-4: Retrieval-Only Endpoint**
- Endpoint: `POST /api/retrieve`
- Input: Query text, number of results (k)
- Output: Retrieved chunks with metadata (URLs, titles, relevance scores)
- Use case: Frontend displays search results before generating answer

**REQ-5: Request Validation**
- All endpoints validate input: non-empty strings, valid field types
- Return 400 Bad Request with error details for invalid input
- Validate conversation history format (array of message objects)
- Validate query length (> 0, < 10,000 chars)

**REQ-6: Response Schema**
- All query responses include: answer, sources, confidence, metadata
- Sources include: URL, page title, relevance score, chunk index
- Metadata includes: latency_ms, grounding status, follow-up suggestions
- Timestamp in ISO 8601 format

**REQ-7: Error Handling**
- Graceful handling of agent failures with user-friendly messages
- Timeout handling: 30 second request timeout
- Retrieval failures: Return 503 Service Unavailable with retry guidance
- LLM failures: Return 502 Bad Gateway with fallback message

**REQ-8: CORS Support**
- Enable cross-origin requests from frontend
- Allow requests from specified frontend origins
- Support credentials in cross-origin requests if needed

### Non-Functional Requirements

**NFR-1: Response Latency**
- 95th percentile response time: < 6 seconds (end-to-end)
- Retrieval endpoint: < 1 second
- Health check: < 100 milliseconds

**NFR-2: Reliability**
- Server uptime: 99.5% during operational hours
- Graceful shutdown: Complete pending requests before termination
- Error logging: All errors logged with request context

**NFR-3: Request/Response Format**
- JSON-based request/response bodies
- UTF-8 encoding for all text
- Consistent error response format

**NFR-4: Scalability**
- Support 10+ concurrent requests
- No performance degradation with conversation history
- Connection pooling for external services

**NFR-5: Configuration**
- All sensitive data from environment variables (.env)
- No hardcoded secrets (API keys, URLs, credentials)
- Configuration validation on startup

## Solution Approach

### Architecture Overview

```
Frontend Application
        ↓
   (HTTP Request)
        ↓
   FastAPI Server
        ├── Request Validation
        ├── Routing
        ├── Middleware (CORS, logging)
        ├── Response Formatting
        └── Error Handling
        ↓
   RAG Agent (Spec-3)
        └── Retrieval Service (Spec-2)
        ↓
   (JSON Response)
        ↓
Frontend Application
```

### API Endpoints

**1. POST /api/query**
```json
Request:
{
  "query": "What is ROS2?",
  "conversation_history": [
    {"role": "user", "content": "..."},
    {"role": "assistant", "content": "..."}
  ],
  "user_role": "student"
}

Response:
{
  "answer": "ROS2 is a middleware framework...",
  "sources": [
    {
      "url": "http://example.com/ros2",
      "page_title": "ROS2 Introduction",
      "relevance_score": 0.92,
      "chunk_index": 0
    }
  ],
  "confidence": "high",
  "metadata": {
    "latency_ms": 2453,
    "grounding": true,
    "follow_ups": ["Tell me about ROS2 topics", "How do services work?"]
  },
  "timestamp": "2025-12-26T12:34:56Z"
}
```

**2. POST /api/query/with-context**
```json
Request:
{
  "selected_text": "ROS2 is a middleware framework...",
  "query": "Can you explain this in simpler terms?",
  "conversation_history": []
}

Response:
{
  "answer": "Middleware means...",
  "sources": [...],
  "confidence": "high",
  "metadata": {...}
}
```

**3. GET /api/health**
```json
Response:
{
  "status": "healthy",
  "timestamp": "2025-12-26T12:34:56Z",
  "components": {
    "agent": "ready",
    "retrieval": "ready",
    "llm": "ready"
  }
}
```

**4. POST /api/retrieve**
```json
Request:
{
  "query": "What is ROS2?",
  "k": 5
}

Response:
{
  "results": [
    {
      "id": "chunk-123",
      "text": "ROS2 is a middleware framework...",
      "similarity_score": 0.92,
      "metadata": {
        "url": "http://example.com",
        "page_title": "ROS2 Intro"
      }
    }
  ],
  "count": 1,
  "latency_ms": 287
}
```

### Request/Response Validation

**Input Validation**
- Query text: required, non-empty string, max 10,000 characters
- Conversation history: optional, array of message objects
- User role: optional, one of ("student", "teacher", "researcher")
- Selected text: optional, non-empty string (for with-context endpoint)

**Response Validation**
- All responses include HTTP status code (200, 400, 502, 503)
- All error responses include error message and request ID for debugging
- Answer length validated: non-empty, reasonable length (< 5,000 chars)
- Source metadata validated: URL format, scores in [0-1]

### Error Handling Strategy

| Error Type | HTTP Status | Response Message |
|-----------|-------------|------------------|
| Invalid JSON | 400 Bad Request | "Invalid request format" |
| Missing required field | 400 Bad Request | "Missing required field: {field}" |
| Query too long | 400 Bad Request | "Query exceeds maximum length" |
| Agent timeout | 504 Gateway Timeout | "Request took too long, please try again" |
| Retrieval unavailable | 503 Service Unavailable | "Knowledge base temporarily unavailable" |
| LLM API error | 502 Bad Gateway | "Unable to generate response" |
| Internal server error | 500 Internal Server Error | "An unexpected error occurred" |

### CORS Configuration

- Allow frontend origin (configurable via environment variable)
- Allow credentials in cross-origin requests
- Allowed methods: GET, POST, OPTIONS
- Allowed headers: Content-Type, Authorization
- Max age: 86400 seconds (1 day)

## Component Design

### FastAPI Application Structure

```
backend/src/api/
├── __init__.py                  # API module exports
├── main.py                      # FastAPI app initialization
├── models.py                    # Request/response schemas
├── endpoints/
│   ├── __init__.py
│   ├── query.py                 # Query endpoint
│   ├── health.py                # Health check endpoint
│   └── retrieve.py              # Retrieval endpoint
├── middleware.py                # CORS, logging, error handling
├── validators.py                # Request validation logic
└── errors.py                    # Error response formatting
```

### Request/Response Models (Pydantic)

```python
class QueryRequest(BaseModel):
    query: str
    conversation_history: Optional[List[Dict]] = None
    user_role: str = "student"

class QueryResponse(BaseModel):
    answer: str
    sources: List[Dict]
    confidence: str
    metadata: Dict
    timestamp: str

class HealthResponse(BaseModel):
    status: str
    timestamp: str
    components: Dict[str, str]
```

## Integration Points

### With Spec-3 (RAG Agent)
- Uses: AgentOrchestrator.query() method
- Passes: query text, conversation history, user role
- Receives: AgentResponse object
- Error handling: Catches agent exceptions, returns HTTP error

### With Spec-2 (Retrieval Service)
- Uses: RetrievalService.search() method (via agent)
- Also directly used by /api/retrieve endpoint
- Expects: Returns list of chunks with metadata

### With Frontend
- Client sends HTTP requests to /api/* endpoints
- Expects: JSON request/response bodies
- Connection: HTTP/1.1 or HTTP/2
- Headers: Content-Type: application/json

## Success Criteria

### Functional Success
- ✅ All 4 endpoints implemented and working
- ✅ Requests properly validated with clear error messages
- ✅ Responses match documented schema
- ✅ Conversation history properly maintained
- ✅ Selected-text context properly integrated
- ✅ CORS configured for frontend access

### Performance Success
- ✅ Query endpoint P95 latency < 6 seconds
- ✅ Retrieval endpoint < 1 second
- ✅ Health check < 100ms
- ✅ No memory leaks under load
- ✅ Graceful handling of 10+ concurrent requests

### Reliability Success
- ✅ Server starts without errors
- ✅ All error cases handled gracefully
- ✅ Invalid input rejected with 400 errors
- ✅ Service unavailability reported correctly
- ✅ Request logging enables debugging

### Integration Success
- ✅ Frontend can send queries and receive answers
- ✅ Frontend can display sources and confidence levels
- ✅ Frontend can maintain conversation context
- ✅ Frontend can use selected-text feature
- ✅ Frontend can check service health

## Data Models

### Query Message
```python
{
  "role": "user" | "assistant",
  "content": "Text content",
  "timestamp": "ISO8601"
}
```

### Source Information
```python
{
  "url": "string",
  "page_title": "string",
  "relevance_score": float [0-1],
  "chunk_index": int
}
```

### Agent Response
```python
{
  "answer": "string",
  "sources": [SourceInfo],
  "confidence": "high" | "medium" | "low",
  "metadata": {
    "latency_ms": int,
    "grounding": boolean,
    "follow_ups": [string]
  },
  "timestamp": "ISO8601"
}
```

## Constraints

- **Language:** Python 3.10+
- **Framework:** FastAPI (modern, async-ready)
- **Agent:** Spec-3 RAG agent (OpenAI Agents SDK)
- **Retrieval:** Spec-2 RetrievalService
- **Request Format:** JSON only
- **Configuration:** Environment variables (.env)
- **No authentication:** Not in scope (noted in out-of-scope)
- **No rate limiting:** Not in scope (noted in out-of-scope)

## Assumptions

1. **Frontend sends properly formatted JSON** - API assumes valid JSON in requests
2. **Agent is always available** - FastAPI server assumes Spec-3 agent is initialized and ready
3. **Retrieval service is available** - Assumes Spec-2 RetrievalService is accessible
4. **OpenAI API credentials are configured** - Assumes OPENAI_API_KEY is set
5. **Single-server deployment** - No load balancing, clustering, or horizontal scaling
6. **No persistent session storage** - Conversation history sent by client each request
7. **Synchronous request handling** - FastAPI runs sync endpoints (can be upgraded to async later)
8. **CORS headers sufficient for security** - No additional authentication required for MVP

## Out of Scope

- ❌ User authentication or session management
- ❌ Rate limiting or request throttling
- ❌ Production hardening (https, load balancing, monitoring)
- ❌ Database for storing conversations
- ❌ Analytics or usage tracking
- ❌ API documentation generation (will document manually)
- ❌ Advanced caching strategies
- ❌ WebSocket support for real-time updates
- ❌ GraphQL API

## Risks

### Technical Risks

**Risk 1: Agent Timeout**
- **Probability:** High (LLM can be slow)
- **Impact:** User-facing 504 errors
- **Mitigation:**
  - Set 30-second request timeout
  - Implement graceful timeout message
  - Log timeout events for debugging
  - Monitor P95 latency

**Risk 2: Retrieval Unavailability**
- **Probability:** Medium (Qdrant service could go down)
- **Impact:** Agent cannot answer questions
- **Mitigation:**
  - Return 503 Service Unavailable
  - Guide users to try again
  - Monitor retrieval service health
  - Implement health check endpoint

**Risk 3: Invalid Agent Response**
- **Probability:** Low (agent should always return valid response)
- **Impact:** API returns invalid JSON
- **Mitigation:**
  - Validate agent response before returning
  - Log unexpected response format
  - Return 500 error with details
  - Add response schema validation

### Operational Risks

**Risk 4: Memory Leaks under Load**
- **Probability:** Low (FastAPI is well-tested)
- **Impact:** Server crash after extended use
- **Mitigation:**
  - Run load tests with 10+ concurrent requests
  - Monitor memory usage
  - Log connection state
  - Implement graceful restart

## Acceptance Scenarios

### Scenario 1: Basic Query (Happy Path)
```
Given: Frontend is connected
When: User asks "What is ROS2?"
Then:
  - API returns 200 OK
  - Response includes answer, sources, confidence
  - Latency < 6 seconds
```

### Scenario 2: Query with History
```
Given: User has previous conversation
When: User asks follow-up "Can you explain more?"
Then:
  - API includes previous conversation in request
  - Agent understands context
  - Response references previous answer
```

### Scenario 3: Selected Text Feature
```
Given: User has selected text from book
When: User asks "What does this mean?"
Then:
  - API sends selected text as context
  - Response is more targeted
  - Confidence is high
```

### Scenario 4: Out-of-Domain Query
```
Given: User asks non-textbook question
When: User asks "What is the weather?"
Then:
  - API returns appropriate error message
  - HTTP 200 OK (graceful decline)
  - Message guides user to textbook topics
```

### Scenario 5: Service Unavailable
```
Given: Retrieval service is down
When: User sends query
Then:
  - API returns 503 Service Unavailable
  - Error message is clear
  - User knows to try again later
```

### Scenario 6: Invalid Input
```
Given: Frontend sends malformed request
When: Request missing required field
Then:
  - API returns 400 Bad Request
  - Error message specifies missing field
  - No server error is logged
```

## Testing Strategy

### Unit Tests
- Endpoint request validation
- Response schema validation
- Error message formatting
- CORS header generation

### Integration Tests
- End-to-end query flow with agent
- Conversation history passing
- Selected-text context injection
- Health check accuracy

### Load Tests
- 10+ concurrent requests
- Memory usage under load
- Response time distribution
- No timeouts or errors

### Manual Tests
- Query endpoint with various queries
- Health check endpoint
- Invalid input handling
- Frontend integration (if frontend ready)

---

*RAG Spec-4: Backend–Frontend Integration via FastAPI*
*Ready for planning and implementation*
