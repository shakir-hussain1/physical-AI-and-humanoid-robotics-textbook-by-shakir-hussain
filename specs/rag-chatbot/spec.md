# RAG Chatbot System - Specification

## 1. Project Overview

**Project Name:** Physical AI and Humanoid Robotics RAG Chatbot
**Status:** COMPLETED (MVP)
**Tech Stack:** FastAPI (Backend) + React (Frontend) + Qdrant (Vector DB) + OpenAI (LLM)
**Branch:** `feat/chatbot-ui-and-fastapi-integration`

---

## 2. System Objectives

### Primary Goals:
1. Provide intelligent Q&A for Physical AI and Humanoid Robotics textbook content
2. Retrieve relevant information using semantic search (RAG)
3. Generate accurate answers grounded in knowledge base
4. Support multiple user roles (student, teacher, researcher)
5. Deliver professional, responsive chatbot UI
6. Track answer confidence and source attribution
7. Gracefully handle out-of-domain queries

### Success Criteria:
- ✅ Retrieve relevant documents from vector database
- ✅ Generate contextually accurate responses via LLM
- ✅ Validate answers are grounded in retrieved content
- ✅ Display source citations with relevance scores
- ✅ Report confidence levels (high/medium/low)
- ✅ Handle errors gracefully with user-friendly messages
- ✅ Support multi-turn conversations with history
- ✅ Professional, responsive web UI
- ✅ Sub-5 second average response time

---

## 3. Feature Specifications

### 3.1 RAG (Retrieval-Augmented Generation) Pipeline

**Purpose:** Combine document retrieval with LLM generation for grounded answers

**9-Step Workflow:**

1. **Intent Parsing**
   - Input: User query text
   - Output: Intent type, primary topic, scope
   - Identifies: factual, conceptual, how-to, clarification, out-of-scope queries
   - Topics: ROS, simulation, perception, planning, learning, control

2. **Early Domain Check**
   - Detects out-of-domain queries (weather, sports, jokes, etc.)
   - Returns guidance: "Ask about robotics topics..."

3. **Semantic Retrieval**
   - Query embeddings: Cohere `embed-english-v3.0` (1024-dim)
   - Database: Qdrant Cloud
   - Collection: `textbook_embeddings` (26 stored embeddings)
   - Returns: Top 5 chunks by cosine similarity
   - Fallback: Graceful degradation if retrieval fails

4. **Context Construction**
   - Format: Chunk text + metadata (URL, title, page)
   - Token budget: ~2000 tokens for context
   - Metadata format: "[Page: X] [URL: ...] Content..."

5. **Prompt Building**
   - System Prompt: Role-specific (student/teacher/researcher)
   - Constraint: "Answer only from provided content. No hallucination."
   - User Prompt: Question + context + history (last 5 messages)
   - Forces: Citation of sources

6. **LLM Response Generation**
   - Model: OpenAI GPT-4o-mini
   - Temperature: 0.7 (configurable)
   - Max tokens: 500 per response
   - Retry: Exponential backoff (1s, 2s, 4s) up to 3 attempts
   - Timeout: 30 seconds per request

7. **Grounding Validation**
   - Score answer against context
   - Checks: Word overlap, name/date validation, factual consistency
   - Detects: Hallucinations, out-of-context claims
   - Output: (is_grounded: bool, score: float, reason: str)

8. **Confidence Computation**
   - Formula: grounding_score × retrieval_quality × context_relevance
   - Levels: HIGH (>0.8), MEDIUM (0.5-0.8), LOW (<0.5)
   - Factors: Retrieved chunk count, similarity scores, grounding validation

9. **Response Formatting**
   - Answer: Main response text
   - Sources: Top 3 with URL, title, relevance_score, chunk_index
   - Metadata: grounding_score, follow_up_questions, latency_ms

---

### 3.2 Backend API

#### Endpoints

**POST /api/query**
- Purpose: Main chatbot query endpoint
- Input:
  ```json
  {
    "query": "What is ROS2?",
    "conversation_history": [
      {"role": "user", "content": "..."},
      {"role": "assistant", "content": "..."}
    ],
    "user_role": "student"
  }
  ```
- Output:
  ```json
  {
    "answer": "ROS2 is...",
    "sources": [
      {
        "url": "https://...",
        "page_title": "Introduction to ROS2",
        "relevance_score": 0.92,
        "chunk_index": 0
      }
    ],
    "confidence": "high",
    "metadata": {
      "latency_ms": 2453,
      "grounding": true,
      "grounding_score": 0.87,
      "follow_ups": ["What is ROS1?", "..."],
      "request_id": "abc123"
    }
  }
  ```
- Validation: Query length (max 10k), history format, user role (student/teacher/researcher)
- Errors: 400, 502, 503, 504

**POST /api/query/with-context**
- Purpose: Context-aware query enhancement
- Input: selected_text + query + history
- Processing: Prepends selected text for stronger grounding
- Output: Same as /api/query with context_enhanced flag

**POST /api/retrieve**
- Purpose: Raw vector search without LLM generation
- Input: query, k (1-20 results)
- Output: List of RetrievalResult objects with similarity scores
- Use case: Debugging, testing retrieval quality

**GET /api/health**
- Purpose: Component health status
- Output: {status: ok/degraded, components: {agent, retrieval, api}}

---

#### Request Validation

| Field | Type | Constraints | Error |
|-------|------|-------------|-------|
| query | string | 1-10k chars | QueryTooLongError (400) |
| conversation_history | array | Valid messages, ≤20 items | InvalidHistoryError (400) |
| user_role | string | student/teacher/researcher | ValidationError (400) |
| selected_text | string | ≤5k chars | ValidationError (400) |
| k | integer | 1-20 | InvalidKError (400) |

---

#### Error Handling

| Error | Status | Cause | Message |
|-------|--------|-------|---------|
| ValidationError | 400 | Invalid input | "Query exceeds maximum length..." |
| OutOfDomainError | 400 | Out of scope | "I can only answer about robotics..." |
| LLMError | 502 | GPT-4 failure | "Unable to generate response..." |
| RetrievalError | 503 | Vector DB failure | "Unable to retrieve documents..." |
| ServiceUnavailableError | 503 | Agent not ready | "Service temporarily unavailable..." |
| RequestTimeoutError | 504 | >30s response | "Request exceeded timeout..." |

---

### 3.3 Frontend Chatbot Widget

#### Component: ChatbotWidget

**Location:** `frontend/src/components/ChatbotWidget.jsx`

**Features:**

1. **Persistent Toggle Button**
   - 68px circular button, bottom-right corner
   - Gradient: purple (667eea → 764ba2)
   - Icon: 💬 emoji
   - Animations: Hover scale, click feedback
   - Always visible, non-intrusive

2. **Chat Window**
   - 420px × 680px (responsive)
   - Collapsible with smooth animations
   - Gradient background
   - Header: Bot icon + Title + Close button

3. **Message Display**
   - User messages: Right-aligned, gradient purple, white text
   - Bot messages: Left-aligned, light gray, dark text
   - Timestamps on each message
   - Auto-scroll to latest message
   - Fade-in animation on arrival

4. **Source Attribution**
   - Display under bot message
   - Format: "[📖] Source Title (relevance: 92%)"
   - Clickable links to source URL
   - Truncated if too many sources (show top 3)

5. **Confidence Display**
   - Badge with confidence level
   - HIGH: Green (#059669)
   - MEDIUM: Orange (#d97706)
   - LOW: Red (#dc2626)

6. **Latency Indicator**
   - Format: "⏱️ 2453ms"
   - Displayed in message metadata
   - Performance tracking

7. **Input Area**
   - Multi-line textarea
   - Send button with gradient
   - Enter-to-send (Shift+Enter for newline)
   - Loading state: Button disabled, spinner visible
   - Placeholder: "Ask about robotics..."

8. **Error Handling**
   - Error messages with red background
   - Help text: "Make sure backend is running..."
   - Connection error recovery

9. **Dark Mode Support**
   - Detects `prefers-color-scheme: dark`
   - Alternative color scheme
   - Maintains readability in both modes

10. **Responsive Design**
    - Mobile: 100vw - 32px width
    - Tablet: Adjusted padding, font sizes
    - Desktop: Full 420px width
    - Touch-friendly: 44px+ tap targets

---

### 3.4 Vector Database (Qdrant)

**Cloud Instance:** Qdrant Cloud (eu-west3-0, GCP)

**Collection:** `textbook_embeddings`
- Dimension: 1024 (Cohere embeddings)
- Distance metric: Cosine similarity
- Size: 26 vectors (textbook chunks)
- Payload: {url, page_title, chunk_text, chunk_index}

**Similarity Search:**
- Input: Query embedding (1024-dim)
- Process: Cosine similarity calculation
- Output: Top 5 results with scores [0, 1]
- Score interpretation: >0.8 = highly relevant, 0.5-0.8 = relevant, <0.5 = marginal

---

### 3.5 Embedding Service (Cohere)

**Model:** `embed-english-v3.0`
- Dimension: 1024
- Input truncation: 2048 chars max
- Batch capability: Process up to 100 queries
- Retry logic: Max 3 attempts with backoff

**Usage:**
- Single query: `embed_query_text(text)` → embedding
- Batch: `embed_batch_queries(texts)` → embeddings list

---

### 3.6 LLM Integration (OpenAI)

**Model:** GPT-4o-mini
- Temperature: 0.7 (balanced creativity/consistency)
- Max tokens: 500 per response
- Retry: Exponential backoff (1s, 2s, 4s)
- Max retries: 3 attempts
- Timeout: 30 seconds
- Error handling: Graceful fallback to error message

**Prompting Strategy:**
- System: Role-specific constraints + ground-truth instruction
- User: Context-first, then question
- Output: Structured response with source citations

---

### 3.7 Configuration Management

**Environment Variables:**

```bash
# OpenAI
OPENAI_API_KEY=sk-proj-...
OPENAI_MODEL=gpt-4o-mini

# Qdrant
QDRANT_URL=https://...eu-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGc...
QDRANT_COLLECTION=textbook_embeddings

# Cohere
COHERE_API_KEY=NNFpXQnExa...

# FastAPI
FASTAPI_HOST=0.0.0.0
FASTAPI_PORT=8000
LOG_LEVEL=INFO
```

**Server Config:**
- Host: 0.0.0.0 (all interfaces)
- Port: 8000
- CORS: Enabled for localhost:3000, localhost:8080
- Reload: Enabled in development
- Logging: JSON format with request IDs

---

## 4. Non-Functional Requirements

### 4.1 Performance

- **Average Response Time:** <5 seconds (retrieval + LLM)
- **Retrieval Latency:** <500ms (vector search)
- **LLM Latency:** 1-4 seconds (including retries)
- **Frontend Response:** <100ms (API request handling)
- **Concurrent Users:** Support 50+ simultaneous queries
- **Throughput:** 10+ queries/second

### 4.2 Reliability

- **Availability:** 99.5% uptime target
- **Error Recovery:** Graceful degradation on failures
- **Retry Logic:** 3 attempts with exponential backoff
- **Fallback Responses:** LLM-only if retrieval fails
- **Health Checks:** /api/health endpoint
- **Monitoring:** Latency tracking, hallucination detection

### 4.3 Scalability

- **Vector DB:** Cloud-based (Qdrant Cloud)
- **LLM:** API-based (OpenAI)
- **Backend:** Async/await with ASGI
- **Frontend:** Client-side rendering
- **Caching:** Future: Query result caching, embedding cache

### 4.4 Security

- **API Keys:** Environment variables only
- **HTTPS:** Required for production
- **CORS:** Restricted to known origins
- **Input Validation:** All user inputs sanitized
- **Rate Limiting:** Future enhancement
- **Authentication:** JWT support ready (not active yet)

### 4.5 Maintainability

- **Code Organization:** Modular, layered architecture
- **Documentation:** Inline comments + spec docs
- **Testing:** Unit + integration tests in place
- **Logging:** Structured JSON logging
- **Error Messages:** User-friendly with technical details

---

## 5. Architecture Constraints

### Technology Choices:

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| Backend | FastAPI | Async, fast, modern Python framework |
| Frontend | React | Component-based, responsive |
| Vector DB | Qdrant | Cloud-ready, high-performance |
| Embeddings | Cohere | Strong multilingual support |
| LLM | OpenAI GPT-4 | SOTA reasoning, reliable API |
| Orchestration | Custom Agent | Full control, transparency |

### Design Patterns:

- **Pipeline Pattern:** 9-step RAG workflow
- **Service Layer:** Separation of concerns
- **Dependency Injection:** Configuration-driven setup
- **Error Handling:** Custom exception hierarchy
- **Validation:** Multi-layer input validation
- **Monitoring:** Centralized logging/metrics

---

## 6. Data Models

### Request Models:
- `QueryRequest`: query + history + user_role
- `ContextQueryRequest`: selected_text + query + history
- `RetrievalRequest`: query + k
- `MessageModel`: role + content

### Response Models:
- `QueryResponse`: answer + sources + confidence + metadata
- `SourceInfo`: url + page_title + relevance_score + chunk_index
- `RetrievalResponse`: results + count + latency_ms
- `HealthResponse`: status + components
- `ErrorResponse`: error + message + request_id

### Internal Models:
- `Intent`: query_type + primary_topic + scope
- `Query`: text + history + user_role
- `AgentResponse`: answer + sources + confidence + metadata
- `RetrievedChunk`: text + metadata + similarity_score

---

## 7. Known Limitations

1. **Knowledge Base Size:** Limited to 26 textbook chunks
   - Mitigation: Expand with more textbook content

2. **Conversation Memory:** Limited to last 5 messages
   - Mitigation: Future database persistence

3. **Language:** English only
   - Mitigation: Cohere supports multilingual, can be enabled

4. **Rate Limiting:** Not implemented
   - Mitigation: Add Redis-based rate limiting

5. **Authentication:** Not required yet
   - Mitigation: JWT tokens ready if needed

6. **Caching:** No response caching
   - Mitigation: Add Redis cache for repeated queries

---

## 8. Future Enhancements

### Phase 2:
- [ ] Database persistence for conversations
- [ ] User accounts and conversation history
- [ ] Advanced search filters (by topic, date, etc.)
- [ ] Multi-language support
- [ ] Response caching for popular queries
- [ ] Analytics dashboard

### Phase 3:
- [ ] Fine-tuned LLM for domain-specific performance
- [ ] Advanced citation generation
- [ ] Fact-checking module
- [ ] Knowledge base automatic updates
- [ ] Batch processing for bulk queries

### Phase 4:
- [ ] Mobile app (iOS/Android)
- [ ] Webhook integrations
- [ ] Custom model fine-tuning API
- [ ] Enterprise features (SSO, audit logs)

---

## 9. Success Metrics

### Functional Metrics:
- Retrieval accuracy: >80% relevant documents in top 5
- Answer grounding: >90% answers grounded in context
- Out-of-domain detection: >95% correct classification
- Source attribution: 100% (every answer has sources)

### Performance Metrics:
- Average response time: <5 seconds
- P95 latency: <8 seconds
- API availability: >99.5%
- Error rate: <1%

### User Metrics:
- UI responsiveness: <100ms
- Load time: <2 seconds
- Error message clarity: User survey >4/5
- Confidence accuracy: >85% match with actual quality

---

## 10. Acceptance Criteria

✅ **Backend API:**
- All endpoints respond correctly
- Validation catches invalid inputs
- Error messages are helpful
- Health check returns component status

✅ **RAG Pipeline:**
- Retrieves relevant documents
- Generates grounded answers
- Computes confidence correctly
- Formats sources accurately

✅ **Frontend UI:**
- Chatbot widget displays properly
- Messages send and receive
- Sources are clickable
- Dark mode works
- Responsive on mobile

✅ **Configuration:**
- Loads from environment variables
- Validates required vars
- Provides helpful error messages

✅ **Documentation:**
- This spec file
- plan.md (architecture)
- tasks.md (implementation)
- requirements.md (dependencies)
- Inline code comments

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-27 | Initial completion: RAG chatbot MVP |

---

## Review & Sign-off

**Status:** COMPLETED
**Last Updated:** 2025-12-27
**Review Date:** -
**Approved By:** -
