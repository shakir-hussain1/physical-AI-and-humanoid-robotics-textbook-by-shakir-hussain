# RAG Spec-3: Agent Construction with Retrieval Integration

## Version
**Spec Version:** 1.0.0
**Created:** 2025-12-26
**Last Updated:** 2025-12-26

## Feature Overview
**Feature:** Agent Construction with Retrieval Integration
**Module Alignment:** Backend AI Agent for RAG Chatbot System
**Target Audience:** AI engineers implementing intelligent query handling over book content

## Constitutional Alignment
This specification aligns with the following constitutional principles:
- PRINCIPLE_1: Technical Excellence and Accuracy
- PRINCIPLE_4: Practical Application
- PRINCIPLE_9: RAG Chatbot Integration and Knowledge Accessibility
- PRINCIPLE_10: Deployment and Platform Accessibility

## Problem Statement

The RAG system has successfully implemented:
1. **Spec-1:** Website ingestion and embedding generation (26 vectors in Qdrant)
2. **Spec-2:** Data retrieval and pipeline validation (similarity search with metadata)

However, without an intelligent agent layer, the system cannot:

1. **Understand Complex Queries** - Process nuanced, multi-part questions over textbook content
2. **Ground Responses in Evidence** - Ensure answers are based ONLY on retrieved book content
3. **Handle Query Context** - Support follow-up questions and conversation flows
4. **Provide Explainable Answers** - Include source citations and retrieval confidence
5. **Scale to Production** - Support integration with FastAPI and future UI components

This specification addresses the intelligent agent layer, enabling context-aware question answering grounded in the textbook while maintaining clear separation from retrieval logistics.

## Requirements

### Functional Requirements

**REQ-1: Agent Initialization and Configuration**
- Agent must initialize with access to RetrievalService (from Spec-2)
- Must load system prompts defining behavior and constraints
- Must validate configuration before accepting queries
- Must support configuration updates without reinitializing

**REQ-2: Query Understanding and Intent Recognition**
- Agent must parse user queries to identify:
  - Primary topic (e.g., "ROS2 communication", "simulation")
  - Query type (question, command, clarification)
  - Temporal/scope context if present
- Must handle various query formats (natural language, abbreviated, multi-sentence)
- Must reject or clarify ambiguous queries appropriately

**REQ-3: Retrieval-Augmented Response Generation**
- Given a user query, agent must:
  1. Retrieve relevant chunks from Qdrant via RetrievalService.search(k=5)
  2. Construct a context string from retrieved chunks
  3. Pass context + query to LLM for answer generation
  4. Ensure LLM responses reference only retrieved content
- Must validate that answers are grounded in retrieved content (no hallucination)
- Must track which chunks were used for each answer

**REQ-4: Context-Aware Conversation**
- Agent must maintain conversation history (up to 10 recent messages)
- Must use history to resolve pronouns and references in follow-up questions
- Must detect when context is insufficient for answering
- Must support role-based system prompts (teacher, student, researcher)

**REQ-5: Source Attribution and Transparency**
- Each response must include:
  - List of source chunks with relevance scores (top-3)
  - URLs and page titles of retrieved content
  - Confidence indicators (e.g., "highly confident" if avg relevance > 0.8)
- Must track retrieval latency and include in response metadata
- Must provide reasoning for answer ("Based on [chunks X, Y, Z]")

**REQ-6: Error Handling and Fallbacks**
- Must gracefully handle Qdrant retrieval failures:
  - Retry with exponential backoff (1s → 2s → 4s, max 3 attempts)
  - Return clear error message: "Unable to access knowledge base"
  - Log failures for debugging
- Must handle LLM API failures:
  - Provide cached response if available
  - Return human-readable error: "Unable to generate response, please try again"
- Must reject queries outside knowledge domain:
  - Example: "What is the weather?" → "I can only answer questions about the textbook content"

**REQ-7: Response Formatting and Presentation**
- Responses must be structured with:
  - Main answer (plain text, < 500 words)
  - Source citations (URLs, relevance scores)
  - Confidence level (high/medium/low)
  - Suggested follow-up topics (based on related chunks)
- Must support markdown formatting for readability
- Must truncate long answers with "..." and offer continuation

**REQ-8: Agent Performance Monitoring**
- Must track and log:
  - Query processing latency (retrieval + LLM)
  - Retrieval result quality (avg relevance score)
  - Answer coherence (whether grounded in context)
  - User satisfaction indicators (if available)
- Must identify patterns of failures or low-confidence answers
- Must support sampling for quality audits (10% of queries logged in detail)

### Non-Functional Requirements

**NFR-1: Response Latency**
- Query processing (retrieval + LLM generation): < 5 seconds p95
- Retrieval latency (Spec-2): < 500ms (already met)
- LLM API latency: < 4 seconds p95 (from OpenAI)
- Total end-to-end: < 5 seconds p95

**NFR-2: Answer Quality**
- Hallucination rate: < 5% (answers claiming unretrieved information)
- Grounding rate: > 95% (answers directly support by retrieved content)
- Source accuracy: 100% (all cited sources are correct)
- User satisfaction: > 80% (for grounded answers)

**NFR-3: Context Awareness**
- Conversation history support: 10 recent messages
- Pronoun resolution: > 85% accuracy on test set
- Follow-up question handling: > 90% success rate

**NFR-4: Reliability and Availability**
- Agent uptime: 99.5%+ during operational hours
- Graceful degradation on Qdrant failure (fallback messages)
- Error recovery: Automatic retry with exponential backoff
- Log persistence: All errors and failures logged for debugging

**NFR-5: Scalability**
- Support 10+ concurrent user queries
- Support up to 100,000 queries per day
- No performance degradation with conversation history up to 10 messages
- Memory usage < 500MB per concurrent user

### Constraints

- **Language:** Python 3.10+
- **Agent Framework:** OpenAI Agents SDK (or compatible)
- **Retrieval Integration:** Synchronous calls to RetrievalService from Spec-2
- **LLM Model:** OpenAI GPT-4 (or compatible, configurable)
- **Knowledge Source:** Qdrant Cloud textbook_embeddings collection (26 vectors from Spec-1)
- **Response Style:** Educational, clear, evidence-based
- **No Advanced Tools:** No web search, calculation, or specialized tools (retrieval only)
- **No Persistent State:** Stateless agent (conversation history passed per request)
- **No Authentication:** Single-user context (authentication handled upstream)

## Solution Approach

### High-Level Design

The agent layer orchestrates three main components:

```
User Query
    ↓
[Agent Intent Parser] → Understand query context
    ↓
[RetrievalService] → Fetch relevant chunks from Qdrant
    ↓
[Context Constructor] → Build context string from chunks
    ↓
[LLM Prompt Builder] → Format context + query for LLM
    ↓
[OpenAI API] → Generate grounded response
    ↓
[Response Formatter] → Add citations and metadata
    ↓
Structured Response (answer + sources + confidence)
```

### Technical Architecture

```
┌─────────────────────────────────────────────────────────┐
│              RAG Agent System                            │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────────┐                                   │
│  │  Query Input     │                                   │
│  │  (User Question) │                                   │
│  └──────────────────┘                                   │
│         │                                                │
│         ▼                                                │
│  ┌──────────────────────────────────────┐              │
│  │  Intent Parser & Query Analyzer      │              │
│  │  - Topic extraction                  │              │
│  │  - Query type detection              │              │
│  │  - Context preservation              │              │
│  └──────────────────────────────────────┘              │
│         │                                                │
│         ▼                                                │
│  ┌──────────────────────────────────────┐              │
│  │  RetrievalService Interface          │              │
│  │  (Spec-2 Integration)                │              │
│  │  - Search similar(query, k=5)        │              │
│  │  - Get collection stats              │              │
│  └──────────────────────────────────────┘              │
│         │                                                │
│         ▼                                                │
│  ┌──────────────────────────────────────┐              │
│  │  Context Constructor                 │              │
│  │  - Build context from chunks         │              │
│  │  - Calculate relevance scores        │              │
│  │  - Extract source metadata           │              │
│  └──────────────────────────────────────┘              │
│         │                                                │
│         ▼                                                │
│  ┌──────────────────────────────────────┐              │
│  │  LLM Prompt Builder                  │              │
│  │  - System prompt (constraints)       │              │
│  │  - Retrieved context                 │              │
│  │  - User query                        │              │
│  │  - Conversation history              │              │
│  └──────────────────────────────────────┘              │
│         │                                                │
│         ▼                                                │
│  ┌──────────────────────────────────────┐              │
│  │  OpenAI API Call (GPT-4)             │              │
│  │  - Generate response                 │              │
│  │  - Validate grounding                │              │
│  └──────────────────────────────────────┘              │
│         │                                                │
│         ▼                                                │
│  ┌──────────────────────────────────────┐              │
│  │  Response Formatter                  │              │
│  │  - Add citations                     │              │
│  │  - Calculate confidence              │              │
│  │  - Suggest follow-ups                │              │
│  │  - Attach metadata                   │              │
│  └──────────────────────────────────────┘              │
│         │                                                │
│         ▼                                                │
│  Structured Response JSON                              │
│  {                                                      │
│    "answer": "...",                                   │
│    "sources": [...],                                  │
│    "confidence": "high",                              │
│    "latency_ms": 2345,                                │
│    "follow_ups": [...]                                │
│  }                                                     │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Data Flow

1. **User provides query** → Query + optional conversation history
2. **Intent parsing** → Extract topic, query type, context
3. **Similarity search** → Call RetrievalService.search(query, k=5)
4. **Chunk retrieval** → Receive 5 most relevant chunks with metadata
5. **Context construction** → Build context string with chunks and metadata
6. **Prompt formatting** → Combine system prompt + context + query
7. **LLM generation** → Call OpenAI GPT-4 with formatted prompt
8. **Grounding validation** → Verify response references retrieved chunks
9. **Response formatting** → Add sources, confidence, and metadata
10. **Return to user** → Structured response with sources and confidence

## Detailed Design

### Component 1: Agent Orchestrator
- **Purpose:** Main entry point for query processing
- **Interfaces:**
  - Input: Query text (string), optional conversation_history (List[Message]), optional role (string)
  - Output: AgentResponse with answer, sources, confidence, metadata
- **Methods:**
  - `process_query(query: str, history: List[Message] = None, role: str = "student") -> AgentResponse`
  - `update_system_prompt(prompt: str) -> bool`
  - `get_stats() -> Dict`

### Component 2: Intent Parser
- **Purpose:** Understand query context and intent
- **Methods:**
  - `extract_topic(query: str) -> str` - Primary topic
  - `detect_query_type(query: str) -> QueryType` - question/command/clarification
  - `resolve_pronouns(query: str, history: List[Message]) -> str` - Resolve references
  - `validate_query(query: str) -> Tuple[bool, str]` - Check query validity

### Component 3: Context Constructor
- **Purpose:** Build context from retrieved chunks
- **Methods:**
  - `construct_context(chunks: List[Dict]) -> str` - Format chunks as context
  - `calculate_relevance(chunk: Dict) -> float` - Relevance score
  - `extract_metadata(chunks: List[Dict]) -> List[SourceInfo]` - Extract sources

### Component 4: Prompt Builder
- **Purpose:** Format prompts for LLM
- **Methods:**
  - `build_system_prompt(role: str) -> str` - Role-based system prompt
  - `build_user_prompt(query: str, context: str, history: List[Message]) -> str` - User prompt
  - `validate_prompt_length(prompt: str) -> bool` - Ensure within token limits

### Component 5: LLM Interface
- **Purpose:** Communicate with OpenAI API
- **Methods:**
  - `generate_response(system_prompt: str, user_prompt: str) -> str` - Get LLM response
  - `validate_grounding(response: str, context: str) -> Tuple[bool, float]` - Check grounding

### Component 6: Response Formatter
- **Purpose:** Format and structure agent output
- **Methods:**
  - `format_response(answer: str, sources: List[SourceInfo], context: str) -> AgentResponse`
  - `calculate_confidence(relevance_scores: List[float]) -> str` - high/medium/low
  - `suggest_follow_ups(chunks: List[Dict]) -> List[str]` - Related topics

## User Experience

### Learning Objectives
- Understand how RAG agents ground responses in evidence
- Learn to ask effective questions about textbook content
- Recognize confidence levels and source attribution
- Understand when agent lacks relevant knowledge

### Conversation Patterns

**Pattern 1: Direct Question**
```
User: "What is ROS2 middleware?"
Agent: "ROS2 uses DDS-based middleware for... [sources listed]"
```

**Pattern 2: Follow-up Question**
```
User: "Tell me about ROS2."
Agent: "ROS2 is a robotics middleware framework..."
User: "How does communication work?"
Agent: "[Uses conversation history to resolve 'communication' as ROS2-specific]"
```

**Pattern 3: Out-of-Domain Query**
```
User: "What is the weather?"
Agent: "I can only answer questions about Physical AI and Humanoid Robotics textbook content."
```

**Pattern 4: Low Confidence Answer**
```
User: "Advanced quantum computing in robotics?"
Agent: "I found limited information on this topic in the textbook. [Low relevance chunks]"
```

## Implementation Requirements

### Technology Stack
- **Framework:** OpenAI Agents SDK (or compatible)
- **LLM:** OpenAI GPT-4 API (gpt-4-turbo or latest)
- **Retrieval:** RetrievalService (Spec-2)
- **Vector DB:** Qdrant Cloud (via Spec-2)
- **Language:** Python 3.10+
- **Configuration:** python-dotenv

### Code Standards
- Follow PEP 8 style guidelines
- Type hints on all function signatures
- Docstrings for all classes and public methods
- Error handling with specific exception types
- Structured logging with context
- No hardcoded credentials or LLM parameters

### Testing Requirements
- **Unit Tests:** Intent parsing, prompt building, response formatting
- **Integration Tests:** End-to-end query → retrieval → LLM → response
- **Validation Tests:** Grounding validation, hallucination detection
- **Performance Tests:** Latency measurements against targets

## Quality Assurance

### Technical Validation
- Execute 20+ test queries covering all modules and topics
- Verify responses are grounded in retrieved content (manual review)
- Compare LLM latency against < 4 second target
- Validate source attribution accuracy
- Test conversation history and pronoun resolution
- Test error handling with simulated failures

### Validation Metrics
- Hallucination rate: < 5% (manual review of 100 sample responses)
- Grounding rate: > 95% (answers directly supported by context)
- Source accuracy: 100% (all cited URLs and titles correct)
- Response latency: p95 < 5 seconds
- Conversation continuity: > 90% on follow-up questions

## Dependencies and Integrations

### Internal Dependencies
- **Spec-2 (Data Retrieval):** RetrievalService class with search() method
  - Requires: textbook_embeddings collection in Qdrant
  - Interface: search(query: str, k: int) → List[Dict] with similarity scores
- **Configuration:** .env with OPENAI_API_KEY, OPENAI_MODEL, Qdrant credentials (from Spec-1/2)

### External Dependencies
- **OpenAI API:** GPT-4 model for LLM generation
  - Rate limit: 90,000 tokens/minute (paid tier)
  - Cost: ~$0.03 per 1K tokens (input), ~$0.06 per 1K tokens (output)
- **Qdrant Cloud:** Already configured in Spec-1/2
- **Python Libraries:**
  - openai (≥1.3.0)
  - python-dotenv (≥1.0.0)
  - (Spec-2 retrieval dependencies already installed)

## Risks and Mitigation

### Technical Risks

**Risk 1: LLM Hallucination (High Impact)**
- **Impact:** Agent provides plausible-sounding answers not supported by retrieved content
- **Mitigation:**
  - Implement grounding validation (check response references context)
  - Use system prompt with explicit constraint: "Only use retrieved information"
  - Track hallucination rate and escalate if > 5%

**Risk 2: Retrieval Failures (High Impact)**
- **Impact:** Unable to retrieve relevant chunks; agent generates low-quality answers
- **Mitigation:**
  - Implement retry logic (exponential backoff)
  - Provide fallback message: "Unable to find relevant content"
  - Log failures for debugging and monitoring

**Risk 3: LLM API Failures (Medium Impact)**
- **Impact:** OpenAI API unavailable or rate-limited
- **Mitigation:**
  - Implement exponential backoff and retry logic
  - Cache recent responses for fallback
  - Monitor API health and quota

**Risk 4: Context Token Overflow (Medium Impact)**
- **Impact:** Prompt exceeds LLM token limit; response generation fails
- **Mitigation:**
  - Validate prompt length before sending
  - Implement chunk truncation if necessary
  - Monitor token usage

### Operational Risks

**Risk 1: Out-of-Domain Questions (Low Impact)**
- **Impact:** Agent answers questions not about textbook (e.g., "What's the weather?")
- **Mitigation:**
  - System prompt explicitly constrains scope
  - Low relevance scores trigger "not found" response
  - Monitor for out-of-domain queries

**Risk 2: Conversation History Misuse (Low Impact)**
- **Impact:** Agent uses old conversation context inappropriately
- **Mitigation:**
  - Clear conversation reset between unrelated queries
  - Limit history to 10 recent messages max
  - Log conversation context for auditing

## Success Criteria

### Quantitative Measures
- Process queries in < 5 seconds p95 (retrieval + LLM)
- Hallucination rate < 5% (100 sample answers, manual review)
- Grounding rate > 95% (answers supported by retrieved content)
- Source accuracy 100% (all cited chunks correct)
- Conversation continuity > 90% (follow-up question success)
- Support 10+ concurrent user queries without degradation

### Qualitative Measures
- Agent responses are clear, educational, evidence-based
- Source attribution is transparent and useful
- Error messages are helpful and non-technical
- Agent appropriately defers on topics outside knowledge base
- Users perceive answers as trustworthy

## Assumptions

- OpenAI GPT-4 API will remain available and responsive (< 4s p95 latency)
- RetrievalService from Spec-2 functions correctly and maintains consistency
- User queries will be primarily about textbook content (not current events, math problems, etc.)
- Conversation history will be passed with each request (stateless agent)
- Top-5 retrieved chunks are sufficient for answer generation
- Relevance scores from Qdrant accurately reflect semantic similarity

## References and Citations

- **Spec-1:** RAG Website Ingestion & Vector Storage (specs/2-website-ingestion/spec.md)
- **Spec-2:** Data Retrieval & Pipeline Validation (specs/3-data-retrieval/spec.md)
- **OpenAI Documentation:** https://platform.openai.com/docs/
- **RAG Best Practices:** https://python.langchain.com/docs/use_cases/question_answering/
- **Prompt Engineering:** https://platform.openai.com/docs/guides/gpt-best-practices

---

*This specification defines the intelligent agent layer for the RAG system, enabling grounded, evidence-based question answering over textbook content.*
