---
name: rag-search
description: Search the RAG knowledge base for Physical AI and robotics information
invokes: rag-query
usage: "/rag-search What is ROS2?"
returns: Grounded answers with sources and confidence
---

# /rag-search

Search the RAG (Retrieval-Augmented Generation) knowledge base for answers about Physical AI, Humanoid Robotics, and related topics.

## Usage

```
/rag-search [your question or query]
```

## Examples

### Factual Query
```
/rag-search What is ROS2?
```

Returns semantic search results from RAG system with:
- Direct answer from knowledge base
- Source citations with URLs
- Confidence score (0-100%)

### Conceptual Query
```
/rag-search How do humanoid robots maintain balance?
```

### How-To Query
```
/rag-search How do I implement inverse kinematics in Python?
```

### Comparative Query
```
/rag-search What's the difference between DH parameters and Euler angles?
```

## Features

- **Semantic Search:** Understands context, not just keywords
- **Source Attribution:** Know where information comes from
- **Confidence Scoring:** Understand reliability of answer
- **Multi-step Retrieval:** Handles complex questions
- **Integration:** Works with Qdrant vector database

## What Gets Searched

The RAG system searches across:
- Physical AI textbook content
- Robotics technical documentation
- ROS2 guides and tutorials
- Kinematics and dynamics resources
- Hardware specification sheets
- Research papers and references

## How It Works

1. **Query Processing:** Your question is analyzed
2. **Semantic Search:** Similar content found in vector database
3. **Retrieval:** Relevant documents retrieved with context
4. **Generation:** LLM generates answer grounded in retrieved content
5. **Attribution:** Sources cited with confidence scores

## Tips for Better Results

- **Be specific:** "ROS2 actionlib" instead of "ROS"
- **Use domain terms:** "inverse kinematics" not "how to move arms"
- **Ask multi-step:** Break complex questions into parts
- **Mention context:** "For humanoid robot" vs general question

## Response Format

```
Confidence: 92%

Answer:
[Generated answer grounded in retrieved content]

Sources:
1. Title: "ROS2 Architecture Overview"
   URL: https://...
   Relevance: 0.95

2. Title: "Async Communication in ROS2"
   URL: https://...
   Relevance: 0.88
```

## See Also

- [RAG Skill Documentation](./.claude/skills/rag-query/)
- [Knowledge Base Contents](./knowledge/)
- [Example Queries](./examples/rag-queries.md)
