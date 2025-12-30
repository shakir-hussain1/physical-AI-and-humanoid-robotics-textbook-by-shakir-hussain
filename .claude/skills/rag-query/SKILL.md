---
name: rag-query
description: Query the RAG system for Physical AI and humanoid robotics knowledge from the textbook. Use when answering questions about Physical AI, humanoid robots, ROS, simulation, perception, control systems, or textbook content. Also use when the user asks to search the knowledge base or mentions RAG queries.
allowed-tools: Bash, Read
model: sonnet
---

# RAG Query Skill

Access the Physical AI and Humanoid Robotics textbook knowledge base through semantic search.

## Quick Start

Ask questions about:
- **Physical AI Concepts:** Digital twins, AI-driven systems, intelligent agents
- **Humanoid Robots:** Architectures, kinematics, dynamics, control
- **ROS & Middleware:** Robot Operating System, middleware design
- **Simulation:** Physics engines, virtual environments, testing
- **Perception:** Sensors, computer vision, state estimation
- **Learning:** Reinforcement learning, imitation learning, adaptation
- **Control Systems:** Inverse kinematics, dynamics, balance, motion planning

## Available Resources

**Textbook Coverage:**
- 26 chapters fully indexed
- 1024-dimensional semantic embeddings (Cohere)
- Qdrant vector database (cloud-hosted)
- Grounding validation for accuracy
- Confidence scoring (high/medium/low)

**Response Format:**
Each query returns:
- **Answer:** Grounded in textbook content
- **Sources:** 3+ references with relevance scores (0-1.0)
- **Confidence:** high (>0.8), medium (0.5-0.8), low (<0.5)
- **Grounding:** Validation score (>0.5 = well-grounded)

## How It Works

1. **Formulation:** Question is analyzed for key concepts
2. **Retrieval:** Semantic search finds 5 most relevant chunks
3. **Generation:** LLM synthesizes answer from retrieved content
4. **Validation:** Answer is checked against source material
5. **Attribution:** Sources ranked by relevance with scores

## Example Queries

### Factual Questions
> "What is ROS2?" → Returns definition with source chapters
> "When was digital twins first introduced?" → Specific facts from textbook
> "What are the main components of a humanoid robot?" → Component list with details

### Conceptual Questions
> "Explain inverse kinematics" → Concept definition + mathematical background + examples
> "How do humanoid robots maintain balance?" → Multi-part explanation with physics
> "What is a digital twin?" → Definition + use cases + architecture

### How-To Questions
> "How do you calculate inverse kinematics?" → Step-by-step algorithm
> "What's the process for humanoid robot motion planning?" → Detailed workflow
> "How are simulations used for testing?" → Methodology and examples

### Comparative Questions
> "What's the difference between ROS1 and ROS2?" → Comparison table + key differences
> "How do humanoid robots differ from traditional robots?" → Feature comparison

## Understanding Confidence Levels

**HIGH (>0.8):** Answer is well-grounded in textbook
- ✅ Use directly without caveat
- ✅ Cite sources (high relevance)

**MEDIUM (0.5-0.8):** Answer has reasonable foundation
- ⚠️ Mention "according to the textbook..."
- ⚠️ Acknowledge some uncertainty
- ⚠️ Suggest further reading

**LOW (<0.5):** Answer may lack sufficient grounding
- ❌ Don't rely solely on this answer
- ❌ Suggest user consult primary sources
- ❌ Recommend additional research

## Source Attribution

Sources are ranked by relevance score:

```
0.9-1.0   | Highly relevant, direct match
0.7-0.8   | Relevant, closely related
0.5-0.7   | Somewhat relevant
<0.5      | Marginal, supporting context
```

**Always cite sources with relevance scores:**
> According to the textbook (Chapter 5, relevance 0.92), inverse kinematics is...

## Tips for Better Results

1. **Be Specific:** "Humanoid robot balance control" vs "humanoid robots"
2. **Use Keywords:** Include technical terms the textbook uses
3. **Multi-step:** For complex topics, ask follow-up questions
4. **Context:** Reference previous questions for clarity
5. **Verification:** Check sources for accuracy

## For Developers

**API Endpoint:**
```
POST http://localhost:8000/api/query
```

**Request:**
```json
{
  "query": "What is ROS2?",
  "user_role": "student",
  "conversation_history": []
}
```

**Response:**
```json
{
  "answer": "ROS2 is a modern robotics middleware...",
  "sources": [
    {
      "url": "https://...",
      "page_title": "ROS Middleware",
      "relevance_score": 0.92
    }
  ],
  "confidence": "high",
  "metadata": {
    "latency_ms": 2453,
    "grounding_score": 0.87
  }
}
```

See [QUERY_REFERENCE.md](QUERY_REFERENCE.md) for detailed API documentation.
See [SCHEMA.md](SCHEMA.md) for textbook structure and collection details.
See [EXAMPLES.md](EXAMPLES.md) for more usage examples.

---

**Status:** Ready to use - Just ask your question!
