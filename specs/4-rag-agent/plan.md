# RAG Spec-3: Agent Construction with Retrieval Integration - Implementation Plan

## Version
**Plan Version:** 1.0.0
**Created:** 2025-12-26
**Last Updated:** 2025-12-26

## Feature Overview
**Feature:** Agent Construction with Retrieval Integration
**Module Alignment:** Backend AI Agent for RAG Chatbot System
**Dependencies:**
- Spec-2 (Data Retrieval & Pipeline Validation) - RetrievalService for query results
- Spec-1 (Website Ingestion) - textbook_embeddings collection in Qdrant Cloud
- OpenAI Agents SDK (latest version with function calling support)
- Python 3.10+ environment with openai, pydantic, and retrieval modules

## Constitutional Alignment
This plan aligns with the following constitutional principles:
- PRINCIPLE_1: Technical Excellence and Accuracy
- PRINCIPLE_4: Practical Application
- PRINCIPLE_6: Collaborative Intelligence
- PRINCIPLE_9: RAG Chatbot Integration and Knowledge Accessibility
- PRINCIPLE_10: Deployment and Platform Accessibility

## Scope and Requirements

### In Scope
- Implement OpenAI Agents SDK-based agent with retrieval integration
- Connect agent to RetrievalService from Spec-2 for grounded responses
- Implement grounding validation to prevent hallucinations
- Support conversation history for multi-turn interactions
- Provide source attribution and confidence indicators
- Implement error handling and graceful fallbacks
- Response formatting with markdown support
- Performance monitoring and latency tracking
- Comprehensive test suite validating agent behavior
- Integration-ready architecture compatible with future FastAPI layer

### Out of Scope
- FastAPI HTTP server or REST endpoints (Spec-4)
- Frontend UI or web components
- User authentication or session management
- Advanced reasoning tools or multi-agent orchestration
- Persistent conversation storage
- Custom LLM fine-tuning or prompt optimization
- Advanced retrieval strategies (reranking, fusion, cross-encoding)

### External Dependencies
- **OpenAI API:** GPT-4 model for response generation
- **Qdrant Cloud:** textbook_embeddings collection with 26 vectors
- **RetrievalService:** Spec-2 service for similarity search
- **Python Ecosystem:** openai>=1.0.0, pydantic, python-dotenv

## Key Decisions and Rationale

### Options Considered

**Option 1: Custom Agent Loop**
- Description: Build custom agent loop with state management and tool execution
- Trade-offs: Full control but complex; error-prone; duplicates OpenAI Agents SDK functionality

**Option 2: OpenAI Agents SDK (Selected)**
- Description: Leverage OpenAI Agents SDK for agent orchestration with custom tools
- Trade-offs: Less control but proven reliability; reduces development scope; maintains framework standards

**Option 3: LangChain Integration**
- Description: Use LangChain's agent abstractions with OpenAI backend
- Trade-offs: Additional dependency; less direct control; overly complex for scope

### Selected Approach
**Decision:** OpenAI Agents SDK (Option 2) with synchronous retrieval integration
**Rationale:**
- OpenAI Agents SDK provides proven, battle-tested agent orchestration
- Allows focus on retrieval integration rather than agent mechanics
- Compatible with GPT-4 and future model upgrades
- Clear API for tool integration (retrieval as a tool)
- Enables stateless query processing (no persistent agent state)

**Trade-offs:**
- Synchronous implementation (async integration deferred to Spec-4)
- 10-message conversation history limit (balance between context richness and token cost)
- No persistent memory across sessions (stateless agent)
- No multi-turn reasoning with intermediate steps (single-turn answers)

### Principles
- **Measurable:** Success criteria include specific latency targets (< 5s p95) and accuracy thresholds (> 95% grounding, < 5% hallucination)
- **Reversible:** Agent architecture designed for future FastAPI integration without refactoring
- **Smallest Viable Change:** Only implement what's needed for context-grounded question answering with transparency

## Implementation Strategy

### Phase 1: Foundation & Agent Setup (Days 1-2)

**Objective:** Build core agent infrastructure and retrieval integration

- [ ] **P1.1:** Initialize agent module structure
  - Create `backend/src/agent/` package with public interfaces
  - Create `backend/src/agent/config.py` for agent configuration (model, max_history, system prompts)
  - Create `backend/src/agent/types.py` for data models (Query, Response, SourceInfo)
  - Create `backend/src/agent/__init__.py` with exports

- [ ] **P1.2:** Implement Agent Orchestrator
  - Create `backend/src/agent/orchestrator.py` with AgentOrchestrator class
  - Initialize OpenAI Agents SDK with GPT-4 model
  - Implement `query(query_text: str, conversation_history: List[Dict] = None) -> AgentResponse`
  - Implement conversation history management (truncate to 10 messages if needed)
  - Integrate retrieval as tool via `tools` parameter in agent creation
  - Handle agent execution and result parsing

- [ ] **P1.3:** Implement Intent Parser
  - Create `backend/src/agent/intent_parser.py` with IntentParser class
  - `parse_intent(query: str) -> Intent` returning topic, query_type, scope
  - Detect out-of-domain queries early (e.g., "What is the weather?")
  - Classify query types: factual_question, conceptual_question, how_to, clarification, out_of_scope
  - Extract temporal/scope context if present (e.g., "Chapter 2" scope restriction)
  - Cache results for repeated queries

- [ ] **P1.4:** Implement Context Constructor
  - Create `backend/src/agent/context_constructor.py` with ContextConstructor class
  - `construct_context(retrieved_chunks: List[Dict], query: str) -> str`
  - Format retrieved chunks into coherent context string
  - Include source metadata (URLs, titles, relevance scores) in context
  - Implement context truncation if combined length > token budget (e.g., 2000 tokens)
  - Preserve chunk ordering by relevance score

### Phase 2: Response Generation & Grounding (Days 3-4)

**Objective:** Implement LLM integration with grounding validation

- [ ] **P2.1:** Implement Prompt Builder
  - Create `backend/src/agent/prompt_builder.py` with PromptBuilder class
  - `build_system_prompt(role: str = "assistant") -> str`
    - Define system prompts for different roles (student, teacher, researcher)
    - Include explicit constraint: "Answer ONLY using provided context, do not hallucinate"
    - Include example responses showing grounded answers with citations
  - `build_user_prompt(query: str, context: str, history: List[Dict] = None) -> str`
    - Combine query, context, and conversation history into single prompt
    - Include retrieved sources and relevance scores
    - Include instruction: "If context does not contain answer, say 'I cannot find this information'"

- [ ] **P2.2:** Implement LLM Interface
  - Create `backend/src/agent/llm_interface.py` with LLMInterface class
  - `generate_response(system_prompt: str, user_prompt: str, temperature: float = 0.7) -> str`
  - Call OpenAI GPT-4 API with system + user prompts
  - Implement exponential backoff for API failures (1s, 2s, 4s, max 3 retries)
  - Parse response and extract answer text
  - Track latency and include in response metadata
  - Handle token limit gracefully (truncate if needed)

- [ ] **P2.3:** Implement Grounding Validator
  - Create `backend/src/agent/grounding_validator.py` with GroundingValidator class
  - `validate_grounding(answer: str, context: str) -> Tuple[bool, float, str]`
    - Check that answer references only information from context
    - Use semantic similarity to detect out-of-context claims
    - Return: (is_grounded, confidence_score [0-1], validation_reason)
  - `detect_hallucination(answer: str, context: str) -> bool`
    - Flag obvious hallucinations (dates, names not in context)
    - Use fuzzy matching for entity detection
  - Track hallucination metrics for monitoring

- [ ] **P2.4:** Implement Response Formatter
  - Create `backend/src/agent/response_formatter.py` with ResponseFormatter class
  - `format_response(answer: str, sources: List[Dict], confidence: str, metadata: Dict) -> AgentResponse`
  - Structure response with: answer, sources (top-3), confidence level, metadata
  - Include reasoning ("Based on [sources X, Y, Z]")
  - Generate suggested follow-up topics based on related chunks
  - Support markdown formatting for readability
  - Truncate long answers (> 500 words) with continuation indicator

### Phase 3: Integration & Error Handling (Days 5-6)

**Objective:** Integrate all components and implement resilience

- [ ] **P3.1:** Implement Retrieval Tool Integration
  - Create `backend/src/agent/retrieval_tool.py` with RetrievalTool class
  - Wrap RetrievalService.search() as OpenAI tool
  - Tool signature: `search_knowledge_base(query: str, k: int = 5) -> List[Dict]`
  - Register tool with agent for automatic invocation
  - Handle retrieval failures with graceful fallback

- [ ] **P3.2:** Implement Error Handling & Fallbacks
  - Create `backend/src/agent/error_handler.py` with ErrorHandler class
  - `handle_retrieval_error(error: Exception) -> AgentResponse`
    - Retry logic: exponential backoff (1s, 2s, 4s)
    - Fallback: "Unable to access knowledge base. Please try again."
  - `handle_llm_error(error: Exception) -> AgentResponse`
    - Return cached response if available
    - Fallback: "Unable to generate response. Please try again."
  - `handle_out_of_domain(query: str) -> AgentResponse`
    - Detect out-of-domain early (via IntentParser)
    - Return: "I can only answer questions about [textbook content]. Please ask about..."
  - Log all errors for debugging

- [ ] **P3.3:** Implement Monitoring & Logging
  - Create `backend/src/agent/monitoring.py` with AgentMonitor class
  - `track_operation(operation: str, start_time: float, result: Dict) -> None`
    - Log latency, result quality, confidence indicators
    - Track retrieval quality (avg relevance score)
    - Track hallucination rate (flagged responses / total)
  - Implement structured JSON logging for machine readability
  - Emit metrics: query_latency_ms, retrieval_quality, confidence_distribution

- [ ] **P3.4:** Implement Configuration Management
  - Create `backend/src/agent/agent_config.py` with AgentConfig class
  - Load from environment: OPENAI_API_KEY, OPENAI_MODEL, AGENT_ROLE, MAX_HISTORY
  - Implement validation and defaults
  - Support configuration updates without restart
  - Document all configuration options

### Phase 4: Testing & Validation (Days 7-8)

**Objective:** Comprehensive testing and integration validation

- [ ] **P4.1:** Create Unit Tests
  - Create `backend/tests/agent/test_intent_parser.py`
    - Test query classification (factual, conceptual, how_to, out_of_scope)
    - Test topic extraction from various query formats
    - Test edge cases (empty, very long, special characters)
  - Create `backend/tests/agent/test_context_constructor.py`
    - Test chunk formatting and ordering
    - Test context truncation behavior
    - Test metadata preservation
  - Create `backend/tests/agent/test_prompt_builder.py`
    - Test system prompt generation for different roles
    - Test user prompt assembly with context
    - Test history formatting
  - Create `backend/tests/agent/test_grounding_validator.py`
    - Test grounding validation logic
    - Test hallucination detection
    - Test edge cases (context-based answers, entity matching)

- [ ] **P4.2:** Create Integration Tests
  - Create `backend/tests/integration/test_agent_workflow.py`
    - Test end-to-end: query → retrieval → generation → validation → response
    - Test conversation history handling
    - Test multiple query types (factual, how_to, follow-up)
  - Create `backend/tests/integration/test_agent_with_retrieval.py`
    - Test agent with actual RetrievalService
    - Verify grounded responses using book content
    - Test error handling (missing context, retrieval failures)

- [ ] **P4.3:** Create Agent Behavior Tests
  - Create `backend/tests/agent/test_agent_behavior.py`
    - Test set of 8-10 book-related questions
    - Verify answers are grounded in textbook content
    - Verify source citations are accurate
    - Verify hallucination detection works
  - Create test queries covering:
    - Single topics: "What is ROS2?", "Explain digital twins"
    - Multi-part: "How do you implement perception in humanoid robots?"
    - Follow-ups: "Can you explain that in more detail?"
    - Out-of-scope: "What is the weather?", "Tell me a joke"

- [ ] **P4.4:** Create Documentation
  - Create `backend/README_AGENT.md`
    - Overview and architecture
    - Module structure and responsibilities
    - Usage examples (direct query, conversation)
    - Configuration options
    - Performance characteristics
  - Create `backend/AGENT_TESTING.md`
    - Test query definitions with expected outcomes
    - Running tests and interpreting results
    - Performance benchmarking procedures
    - Debugging guide for common issues

### Phase 5: Production Validation (Days 9-10)

**Objective:** Execute full validation and performance benchmarking

- [ ] **P5.1:** Execute Unit & Integration Tests
  - Run: `pytest backend/tests/agent/ backend/tests/integration/test_agent*`
  - Target: 100% pass rate on all test suites
  - Measure: Code coverage ≥ 80% on agent modules
  - Verify: No flaky tests (deterministic results)

- [ ] **P5.2:** Execute Agent Behavior Validation
  - Run full book-related test query suite
  - For each query: verify grounding, source accuracy, confidence
  - Generate validation report with results
  - Check: 100% of grounded answers, < 5% hallucination rate
  - Track: P95 latency < 5 seconds for all queries

- [ ] **P5.3:** Performance Benchmarking
  - Measure: End-to-end latency (query → answer)
  - Measure: Per-component latency (retrieval, LLM generation, validation)
  - Measure: Memory usage under load
  - Compare: Against NFR targets (P95 < 5s, avg < 3s)
  - Document results in AGENT_PERFORMANCE_METRICS.txt

- [ ] **P5.4:** Documentation & Readiness
  - Ensure all docstrings and type hints present
  - Create quick-start guide for integration
  - Document API contracts for downstream (Spec-4)
  - Verify production readiness checklist

## Architectural Decisions

### Architecture Diagram

```
User Query
    ↓
[Agent Orchestrator] ← orchestrates execution
    ├─→ [Intent Parser] → detect query type, extract scope
    ├─→ [Retrieval Tool] → search knowledge base
    │       ↓
    │   [RetrievalService] (from Spec-2)
    │       ↓
    │   [Qdrant Cloud] (textbook_embeddings)
    │
    ├─→ [Context Constructor] → format retrieved chunks
    ├─→ [Prompt Builder] → build system + user prompts
    ├─→ [LLM Interface] → call GPT-4 API
    ├─→ [Grounding Validator] → check hallucinations
    ├─→ [Response Formatter] → structure with sources
    └─→ [Monitoring] → track metrics
        ↓
    Agent Response (answer + sources + confidence)
```

### Component Interfaces

**AgentOrchestrator**
```python
class AgentOrchestrator:
    def query(
        query_text: str,
        conversation_history: Optional[List[Message]] = None,
        user_role: str = "student"
    ) -> AgentResponse

class AgentResponse:
    answer: str
    sources: List[SourceInfo]  # top-3 with relevance scores
    confidence: Literal["high", "medium", "low"]
    metadata: Dict  # latency_ms, retrieval_quality, follow_ups
    timestamp: str  # ISO format
```

**RetrievalTool**
```python
def search_knowledge_base(query: str, k: int = 5) -> List[RetrievedChunk]:
    # Called by OpenAI agent as a tool
    # Returns: [{"id": ..., "similarity_score": ..., "metadata": {...}}]
```

### Data Models

**Query (Input)**
```python
@dataclass
class Query:
    text: str
    conversation_history: Optional[List[Message]] = None
    user_role: str = "student"
    context_scope: Optional[str] = None  # e.g., "Chapter 2"
```

**SourceInfo (Output Component)**
```python
@dataclass
class SourceInfo:
    url: str
    page_title: str
    relevance_score: float  # [0, 1]
    chunk_index: int
```

**AgentResponse (Output)**
```python
@dataclass
class AgentResponse:
    answer: str
    sources: List[SourceInfo]
    confidence: Literal["high", "medium", "low"]
    metadata: Dict  # {latency_ms, retrieval_quality, follow_ups}
    timestamp: str
```

## Integration Points

### With Spec-2 (Data Retrieval)
- **Uses:** RetrievalService.search(query, k=5) → List[Dict] with id, similarity_score, metadata
- **Dependency:** RetrievalService must be importable from `backend.src.retrieval`
- **Interface:** Synchronous, expects < 500ms latency per call

### With Spec-1 (Website Ingestion)
- **Indirect:** Uses vectors from Spec-1 via Spec-2
- **Requirement:** 26 vectors in textbook_embeddings collection with complete metadata

### With Future Specs (FastAPI, Frontend)
- **Provides:** AgentResponse JSON structure (serializable to REST response)
- **Enables:** Stateless query processing (no session state)
- **Interface:** Query → Response (compatible with HTTP request/response pattern)

## Non-Functional Requirements

### Performance
- **Response Latency:** P95 < 5 seconds end-to-end (retrieval + LLM generation)
- **Retrieval Latency:** < 500ms (delegated to Spec-2)
- **LLM Latency:** < 4.5s for typical queries (GPT-4 standard latency)

### Quality
- **Hallucination Rate:** < 5% (grounding validation flags suspicious answers)
- **Grounding Rate:** > 95% of answers grounded in retrieved content
- **Source Accuracy:** 100% - sources must match actual retrieved chunks

### Reliability
- **Uptime:** 99.5% availability (dependent on OpenAI API)
- **Retry Logic:** Exponential backoff (1s, 2s, 4s) for transient failures
- **Graceful Degradation:** Clear error messages when service unavailable

### Scalability
- **Concurrent Users:** Support 10+ concurrent queries
- **Query Throughput:** Handle 100K queries/day (avg 1.2 req/s)
- **Memory:** < 500MB base + < 100MB per active conversation

## Success Criteria

### Functional Validation
- ✅ Agent successfully initializes with OpenAI Agents SDK
- ✅ Agent integrates RetrievalService for knowledge queries
- ✅ Responses grounded in retrieved textbook content (> 95%)
- ✅ Source attribution includes URLs and relevance scores
- ✅ Conversation history improves follow-up understanding
- ✅ Error handling works for retrieval and LLM failures
- ✅ Out-of-scope queries handled gracefully

### Performance Validation
- ✅ P95 latency < 5 seconds for book-related queries
- ✅ Hallucination rate < 5% (grounding validation effective)
- ✅ All 8-10 test queries answered correctly with proper sources
- ✅ Confidence indicators accurate (high confidence = high relevance)

### Code Quality
- ✅ All modules have docstrings and type hints
- ✅ Unit test coverage ≥ 80%
- ✅ Integration tests for end-to-end workflows
- ✅ Zero hardcoded secrets (all from .env)

### Documentation
- ✅ README_AGENT.md with architecture and usage
- ✅ AGENT_TESTING.md with test queries and procedures
- ✅ AGENT_PERFORMANCE_METRICS.txt with benchmarking results

## Risk Analysis

### Technical Risks

**Risk 1: Hallucination in Agent Responses**
- **Impact:** Loss of user trust; incorrect information propagated
- **Mitigation:**
  - Implement grounding validator to flag out-of-context answers
  - Use system prompts with explicit constraints
  - Track hallucination metrics; alert if > 5%
  - Provide clear "I cannot find this information" fallback

**Risk 2: Retrieval Failures or Slow Queries**
- **Impact:** Poor user experience; timeouts; cascading failures
- **Mitigation:**
  - Exponential backoff retry logic
  - Clear error messages to user
  - Graceful fallback to general response
  - Monitor Spec-2 retrieval latency

**Risk 3: Token Limit Exceeded in Long Conversations**
- **Impact:** Truncated responses; loss of context
- **Mitigation:**
  - Limit conversation history to 10 messages
  - Implement context summarization if needed
  - Monitor token usage per conversation
  - Clear truncation strategy in prompt

### Operational Risks

**Risk 4: OpenAI API Rate Limits or Service Outages**
- **Impact:** Service degradation during peak usage
- **Mitigation:**
  - Implement request rate limiting
  - Cache responses for identical queries
  - Clear error messages during outages
  - Monitor API availability

**Risk 5: Configuration Drift or Invalid Secrets**
- **Impact:** Service fails at runtime with cryptic errors
- **Mitigation:**
  - Validate configuration on startup
  - Clear error messages for missing/invalid secrets
  - Document configuration options thoroughly

## Assumptions

- **OpenAI Agents SDK API** remains stable and compatible with GPT-4
- **RetrievalService from Spec-2** is available and performant (< 500ms)
- **Qdrant Cloud** textbook_embeddings collection has 26 vectors with complete metadata
- **User queries** are in English (model language)
- **Conversation history** is provided by calling code (agent is stateless)

## Constraints

- **Framework:** OpenAI Agents SDK (latest version)
- **LLM:** GPT-4 model via OpenAI API
- **Language:** Python 3.10+
- **Retrieval Source:** Synchronous RetrievalService from Spec-2
- **Knowledge Base:** Qdrant Cloud textbook_embeddings collection
- **Conversation State:** Stateless (caller provides history, agent doesn't store)
- **No Advanced Tools:** Single retrieval tool only (no code execution, web search, etc.)
- **No Persistent Memory:** Agent doesn't learn from interactions

## Next Steps

After implementation and Phase 5 validation:
1. Create `/sp.tasks` task decomposition for granular execution tracking
2. Execute `/sp.implement` to build all 5 phases
3. Create `/sp.adr` for significant architectural decisions (if needed)
4. Begin Spec-4 planning for FastAPI integration and REST endpoints

---

*This implementation plan provides a clear, phased approach to building a production-grade RAG agent with grounding validation, transparent source attribution, and seamless integration with existing retrieval infrastructure.*
