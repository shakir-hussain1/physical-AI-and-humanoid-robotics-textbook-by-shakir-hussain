# RAG Agent Testing Guide

## Overview

This guide covers testing procedures, test query definitions, and debugging strategies for the RAG Agent.

## Test Queries

### Category 1: Core Textbook Topics

**Query 1: ROS2 Fundamentals**
```
Query: "What is ROS2?"
Expected Content:
  - Definition of ROS2 as middleware framework
  - Key concepts (nodes, topics, services)
  - Communication patterns
Expected Confidence: high
Relevance Target: > 0.85
```

**Query 2: Digital Twin Concept**
```
Query: "Explain the digital twin concept"
Expected Content:
  - Virtual representation of physical systems
  - Simulation and synchronization
  - Applications in robotics
Expected Confidence: high
Relevance Target: > 0.82
```

**Query 3: Perception Systems**
```
Query: "How do perception systems work in humanoid robots?"
Expected Content:
  - Sensor types (camera, lidar, depth)
  - Processing pipeline
  - Integration with control
Expected Confidence: high
Relevance Target: > 0.80
```

**Query 4: Motion Planning**
```
Query: "What are the key components of motion planning?"
Expected Content:
  - Path planning algorithms
  - Collision avoidance
  - Trajectory generation
Expected Confidence: medium
Relevance Target: > 0.78
```

### Category 2: Implementation How-Tos

**Query 5: ROS2 Development**
```
Query: "How do you implement a ROS2 node?"
Expected Content:
  - Node creation syntax
  - Publisher/subscriber patterns
  - Service implementation
Expected Confidence: high
Relevance Target: > 0.88
```

**Query 6: Behavior Planning**
```
Query: "How do you implement behavior planning for humanoid robots?"
Expected Content:
  - Decision trees or state machines
  - Behavior hierarchy
  - Execution framework
Expected Confidence: medium
Relevance Target: > 0.75
```

### Category 3: Conceptual Understanding

**Query 7: AI in Robotics**
```
Query: "Why is artificial intelligence important for robotics?"
Expected Content:
  - Autonomy requirements
  - Decision making
  - Adaptation and learning
Expected Confidence: medium
Relevance Target: > 0.78
```

**Query 8: Integration Challenges**
```
Query: "What are the main challenges in integrating AI with robotics?"
Expected Content:
  - Real-time constraints
  - Safety and reliability
  - Computational resources
Expected Confidence: medium
Relevance Target: > 0.75
```

### Category 4: Edge Cases

**Query 9: Ambiguous Question**
```
Query: "Tell me about robots"
Expected: Lowest specificity
Expected Confidence: low
Relevance Target: > 0.60
```

**Query 10: Out-of-Domain**
```
Query: "What is the weather today?"
Expected: Out-of-scope detection
Expected Response: Guidance to textbook topics
Expected Confidence: high (clear error case)
```

## Running Tests

### Unit Tests

Test intent parser module:
```bash
pytest backend/tests/agent/test_intent_parser.py -v
```

Expected Results:
- Intent classification: 100% accuracy on test cases
- Topic extraction: Correct domain detection
- Out-of-domain detection: Accurate on keyword patterns
- Caching: Deterministic results

Test grounding validator:
```bash
pytest backend/tests/agent/test_grounding_validator.py -v
```

Expected Results:
- Grounding validation: Confidence [0, 1]
- Hallucination detection: Flags obvious fabrications
- Reason generation: Meaningful explanations

### Integration Tests

Test end-to-end workflow:
```bash
pytest backend/tests/integration/test_agent_workflow.py -v
```

Expected Results:
- Component initialization: All modules created
- Query processing: Returns valid AgentResponse
- History handling: Conversation context preserved
- Error handling: Graceful fallbacks work

### Behavior Tests

Test agent responses on book queries:
```bash
pytest backend/tests/agent/test_agent_behavior.py -v
```

Expected Results:
- 14+ book-related test queries pass
- Responses are non-empty strings
- Confidence levels properly set
- Sources included when available
- Out-of-domain queries handled correctly

### Full Test Suite

Run all tests:
```bash
pytest backend/tests/ -v --tb=short
```

Coverage Report:
```bash
pytest backend/tests/ --cov=backend.src.agent --cov-report=html
```

Target Coverage: ≥ 80%

## Manual Testing

### Test Query Execution

1. Create test script:
```python
from backend.src.agent import AgentOrchestrator
import json

agent = AgentOrchestrator()

test_queries = [
    "What is ROS2?",
    "How do you implement motion planning?",
    "What is the weather?",
]

for query in test_queries:
    response = agent.query(query)
    print(f"Query: {query}")
    print(f"Answer: {response.answer}")
    print(f"Confidence: {response.confidence}")
    print(f"Sources: {len(response.sources)}")
    print("---")
```

2. Execute:
```bash
python test_queries.py
```

3. Validate:
- Answers are non-empty
- Confidence is high/medium/low
- Sources include URLs and titles
- Out-of-domain queries are handled

### Performance Testing

1. Measure latency:
```python
import time
from backend.src.agent import AgentOrchestrator

agent = AgentOrchestrator()
queries = ["What is ROS2?"] * 10

latencies = []
for query in queries:
    start = time.time()
    response = agent.query(query)
    latency_ms = (time.time() - start) * 1000
    latencies.append(latency_ms)

import statistics
print(f"Mean: {statistics.mean(latencies):.1f}ms")
print(f"P95: {sorted(latencies)[int(len(latencies)*0.95)]:.1f}ms")
```

2. Expected Results:
- Single query: < 3 seconds (avg)
- P95: < 5 seconds
- No timeout errors

### Conversation Testing

1. Test history handling:
```python
from backend.src.agent import Message, AgentOrchestrator

agent = AgentOrchestrator()

# First query
response1 = agent.query("What is ROS2?")

# Follow-up with history
history = [
    Message(role="user", content="What is ROS2?"),
    Message(role="assistant", content=response1.answer)
]

response2 = agent.query("Can you explain more about topics?", conversation_history=history)
```

2. Validate:
- Agent understands context
- Follow-up uses conversation history
- Pronoun resolution works

## Debugging

### Enable Logging

```python
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("backend.src.agent")

# Now all agent operations will log
```

### Common Issues

**Issue: Low confidence/high hallucination**
- Check context quality from RetrievalService
- Verify grounding validator threshold
- Review LLM temperature setting

**Issue: Slow responses**
- Profile latency breakdown
- Check RetrievalService performance
- Monitor OpenAI API latency

**Issue: Incorrect intent classification**
- Review IntentParser keywords
- Add domain-specific patterns
- Update out-of-domain detection

### Component Debugging

1. Test IntentParser:
```python
from backend.src.agent import IntentParser

parser = IntentParser()
intent = parser.parse_intent("What is ROS2?")
print(f"Type: {intent.query_type}")
print(f"Topic: {intent.primary_topic}")
print(f"Confidence: {intent.confidence}")
```

2. Test Grounding:
```python
from backend.src.agent import GroundingValidator

validator = GroundingValidator()
answer = "ROS2 is middleware"
context = "ROS2 is a middleware framework"
is_grounded, conf, reason = validator.validate_grounding(answer, context)
print(f"Grounded: {is_grounded}, Confidence: {conf:.2f}, Reason: {reason}")
```

3. Test Response Formatting:
```python
from backend.src.agent import ResponseFormatter

formatter = ResponseFormatter()
response = formatter.format_response(
    answer="Test answer",
    sources=[{"url": "http://example.com", "page_title": "Test", "relevance_score": 0.9, "chunk_index": 0}],
    confidence="high"
)
print(response.to_dict())
```

## Performance Benchmarks

### Latency Metrics

| Operation | Target | Current |
|-----------|--------|---------|
| Intent parsing | < 100ms | TBD |
| Retrieval | < 500ms | TBD |
| Prompt building | < 50ms | TBD |
| LLM generation | < 4s | TBD |
| Grounding validation | < 100ms | TBD |
| Total (p95) | < 5s | TBD |

### Quality Metrics

| Metric | Target | Current |
|--------|--------|---------|
| Grounding rate | > 95% | TBD |
| Hallucination rate | < 5% | TBD |
| Intent accuracy | > 90% | TBD |
| Source accuracy | 100% | TBD |

## Continuous Integration

### GitHub Actions Workflow

```yaml
name: Test RAG Agent
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-python@v2
        with:
          python-version: 3.10
      - run: pip install -e .
      - run: pytest backend/tests/agent/ backend/tests/integration/
      - run: pytest --cov=backend.src.agent
```

## Test Checklist

Before releasing, verify:

- [ ] All unit tests pass (intent, grounding, formatter)
- [ ] All integration tests pass (workflow, components)
- [ ] All behavior tests pass (8-10 book queries)
- [ ] Code coverage ≥ 80%
- [ ] Performance: P95 latency < 5s
- [ ] Performance: Hallucination rate < 5%
- [ ] Documentation complete
- [ ] No hardcoded secrets in tests
- [ ] All error cases handled
- [ ] Logging works correctly

---

**Last Updated:** 2025-12-26
**Status:** Ready for Testing
