# RAG Agent: Retrieval-Augmented Question Answering

## Overview

The RAG Agent is an intelligent question-answering system that retrieves relevant textbook content and generates grounded responses using OpenAI's GPT-4 model. It integrates with the Spec-2 RetrievalService to provide context-aware, transparent answers about Physical AI and Humanoid Robotics.

**Key Features:**
- ✅ Query understanding with intent classification
- ✅ Retrieval-augmented response generation
- ✅ Grounding validation to prevent hallucinations
- ✅ Source attribution with relevance scores
- ✅ Conversation history support
- ✅ Error handling and graceful fallbacks
- ✅ Performance monitoring and logging

## Architecture

```
User Query
    ↓
[Agent Orchestrator] - orchestrates entire workflow
    ├→ [Intent Parser] - classify query type, detect scope
    ├→ [Retrieval Tool] - search knowledge base
    │    ↓
    │  [RetrievalService] (from Spec-2)
    │    ↓
    │  [Qdrant Cloud] - textbook_embeddings collection
    │
    ├→ [Context Constructor] - format retrieved chunks
    ├→ [Prompt Builder] - build system + user prompts
    ├→ [LLM Interface] - call GPT-4 API
    ├→ [Grounding Validator] - check for hallucinations
    ├→ [Response Formatter] - structure output
    └→ [Monitoring] - track metrics
        ↓
    [AgentResponse] - answer + sources + confidence
```

## Module Structure

```
backend/src/agent/
├── __init__.py                 # Public interfaces
├── types.py                    # Data models (Query, Message, SourceInfo, AgentResponse)
├── config.py                   # Configuration management
├── orchestrator.py             # Main AgentOrchestrator class
├── intent_parser.py            # Query classification and intent extraction
├── context_constructor.py      # Retrieve and format context
├── retrieval_tool.py           # Retrieval integration
├── prompt_builder.py           # Prompt generation
├── llm_interface.py            # GPT-4 API interface
├── grounding_validator.py      # Hallucination detection
├── error_handler.py            # Error handling and fallbacks
├── response_formatter.py       # Output formatting
└── monitoring.py               # Performance tracking
```

## Installation

### Prerequisites
- Python 3.10+
- OpenAI API key
- Spec-2 RetrievalService accessible

### Setup

1. Install dependencies:
```bash
pip install openai pydantic python-dotenv
```

2. Configure environment (.env):
```bash
OPENAI_API_KEY=your-api-key
OPENAI_MODEL=gpt-4
AGENT_ROLE=assistant
AGENT_MAX_HISTORY=10
AGENT_TEMPERATURE=0.7
AGENT_RETRIEVAL_K=5
```

3. Import agent module:
```python
from backend.src.agent import AgentOrchestrator, AgentConfig

# Initialize
config = AgentConfig.load_from_env()
agent = AgentOrchestrator(config)
```

## Usage

### Basic Query

```python
from backend.src.agent import AgentOrchestrator

agent = AgentOrchestrator()
response = agent.query("What is ROS2?")

print(f"Answer: {response.answer}")
print(f"Confidence: {response.confidence}")
print(f"Sources: {response.sources}")
```

### With Conversation History

```python
from backend.src.agent import Message

history = [
    Message(role="user", content="What is ROS2?"),
    Message(role="assistant", content="ROS2 is a middleware framework..."),
]

response = agent.query(
    "Can you explain that in more detail?",
    conversation_history=history,
    user_role="student"
)
```

### Response Structure

```python
AgentResponse(
    answer="ROS2 is a middleware framework for robotics...",
    sources=[
        SourceInfo(
            url="http://example.com/ros2",
            page_title="ROS2 Introduction",
            relevance_score=0.92,
            chunk_index=0
        ),
        ...
    ],
    confidence="high",
    metadata={
        "reasoning": "Based on Chunk 0, Chunk 2",
        "latency_ms": 2453,
        "follow_up_suggestions": [...]
    }
)
```

## Configuration

### AgentConfig Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| OPENAI_API_KEY | Required | OpenAI API key |
| OPENAI_MODEL | gpt-4 | Model to use |
| AGENT_ROLE | assistant | System role |
| AGENT_MAX_HISTORY | 10 | Conversation history limit |
| AGENT_MAX_CONTEXT_TOKENS | 2000 | Max context size |
| AGENT_RETRIEVAL_K | 5 | Number of results to retrieve |
| AGENT_TEMPERATURE | 0.7 | LLM temperature |
| AGENT_CONFIDENCE_THRESHOLD | 0.5 | Grounding confidence threshold |

## Query Classification

The agent classifies queries into:
- **factual**: Direct information questions ("What is ROS2?")
- **conceptual**: Explanation questions ("Why is simulation important?")
- **how_to**: Implementation questions ("How do you implement perception?")
- **clarification**: Follow-up questions ("Can you explain more?")
- **out_of_scope**: Non-textbook questions ("What is the weather?")

## Grounding Validation

The agent validates responses to prevent hallucinations:
- Word overlap analysis
- Entity matching against context
- Fabrication pattern detection
- Confidence scoring [0-1]

**Target:** > 95% grounding rate, < 5% hallucination rate

## Performance Monitoring

The agent tracks:
- Query latency (p95 < 5 seconds)
- Grounding rate (> 95%)
- Hallucination rate (< 5%)
- Confidence distribution
- Retrieval quality metrics

Access metrics:
```python
metrics = agent.monitor.get_metrics()
print(f"P95 Latency: {metrics['p95_latency_ms']}ms")
print(f"Hallucination Rate: {metrics['hallucination_rate']:.2%}")
```

## Error Handling

The agent handles:
- **Retrieval failures**: Exponential backoff (1s, 2s, 4s)
- **LLM API errors**: Clear error messages
- **Out-of-domain queries**: Helpful guidance
- **Invalid inputs**: Graceful degradation

## Testing

### Run all tests
```bash
pytest backend/tests/agent/
pytest backend/tests/integration/test_agent_workflow.py
pytest backend/tests/agent/test_agent_behavior.py
```

### Test coverage
- Unit tests: Intent parsing, grounding validation, response formatting
- Integration tests: End-to-end workflows
- Behavior tests: 14+ book-related queries

Target coverage: ≥ 80%

## Integration with FastAPI (Spec-4)

The agent is designed for stateless REST integration:

```python
# Future Spec-4 endpoint
@app.post("/api/query")
def query_endpoint(query_request: QueryRequest):
    response = agent.query(
        query_text=query_request.text,
        conversation_history=query_request.history,
        user_role=query_request.role
    )
    return response.to_dict()
```

## Troubleshooting

### Issue: "Unable to access knowledge base"
- Check RetrievalService is running
- Verify Qdrant Cloud credentials
- Check network connectivity

### Issue: Low confidence/high hallucination rate
- Review grounding threshold
- Check context quality from Spec-2
- Adjust temperature for less creative responses

### Issue: Slow response times
- Monitor retrieval latency (target < 500ms)
- Check LLM API response times
- Consider caching for repeated queries

## Performance Benchmarks

**Single Query (p95):**
- Retrieval: < 500ms
- LLM generation: < 4s
- Total: < 5s

**Quality Metrics:**
- Grounding rate: > 95%
- Hallucination rate: < 5%
- Intent classification: 90%+

## Future Enhancements

- Conversation summarization for long histories
- Response caching for identical queries
- Multi-turn reasoning with intermediate steps
- Advanced retrieval strategies (reranking, fusion)
- Custom fine-tuning for textbook domain

## References

- [RAG Specification (Spec-3)](../specs/4-rag-agent/spec.md)
- [Implementation Plan](../specs/4-rag-agent/plan.md)
- [Data Retrieval (Spec-2)](../specs/3-data-retrieval/README_RETRIEVAL.md)
- [OpenAI Agents SDK](https://openai.com/developers)

---

**Status:** Production Ready (Phase 11 Complete)
