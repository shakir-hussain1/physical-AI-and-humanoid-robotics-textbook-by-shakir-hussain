# RAG System Schema Reference

## Database Information

**Platform:** Qdrant Cloud
**Region:** EU-West-3 (GCP)
**Collection Name:** `textbook_embeddings`
**Vector Dimension:** 1024 (Cohere embed-english-v3.0)
**Distance Metric:** Cosine Similarity
**Total Embeddings:** 26+ chapters indexed

## Payload Structure

Each vector in Qdrant has the following metadata:

```json
{
  "vector": [float, float, ...],  // 1024-dimensional embedding
  "payload": {
    "chunk_text": "string",        // Full chapter/section text
    "page_title": "string",        // Chapter/section title
    "url": "string",               // Link to textbook section
    "chunk_index": 0,              // Position in chapter
    "topic": "string"              // Primary topic (ROS, Control, etc.)
  }
}
```

## Query Response Format

```json
{
  "answer": "string",              // Generated answer
  "sources": [
    {
      "url": "string",             // Link to source
      "page_title": "string",      // Chapter title
      "relevance_score": 0.92,     // Similarity score (0-1)
      "chunk_index": 0             // Position in chapter
    }
  ],
  "confidence": "high|medium|low",
  "metadata": {
    "latency_ms": 2453,
    "grounding": true,
    "grounding_score": 0.87
  }
}
```

## Indexing Coverage

### Topics Indexed
1. **Robotics Fundamentals**
   - Physical AI concepts
   - Humanoid robot design
   - Robot architecture

2. **Control Systems**
   - Kinematics (forward & inverse)
   - Dynamics and motion
   - Balance control
   - Walking patterns

3. **Perception**
   - Sensors and sensing
   - Computer vision
   - State estimation
   - Sensor fusion

4. **Planning & Navigation**
   - Motion planning algorithms
   - Path planning
   - Trajectory generation
   - Obstacle avoidance

5. **Learning Systems**
   - Reinforcement learning
   - Imitation learning
   - Adaptation
   - Training methodologies

6. **Simulation**
   - Physics engines
   - Digital twins
   - Virtual testing
   - Real-to-sim transfer

7. **ROS & Middleware**
   - ROS1 vs ROS2
   - Node architecture
   - Message passing
   - Service design

8. **Real-World Deployment**
   - Hardware integration
   - Safety systems
   - Power management
   - Field operations

## Search Parameters

**Similarity Threshold (for results):**
```
> 0.8 = High relevance (primary results)
0.5-0.8 = Medium relevance (supporting)
< 0.5 = Low relevance (context only)
```

**Top-K Results:**
```
Default: Return top 5 results
Can increase to 10 for comprehensive searches
```

## Grounding Score Interpretation

**Score Range:** 0-1.0

```
0.9-1.0   = Excellent grounding (answer fully supported)
0.7-0.8   = Good grounding (answer mostly supported)
0.5-0.7   = Fair grounding (partial support)
< 0.5     = Poor grounding (needs external sources)
```

## Rate Limits

**API Limits:**
- Queries per second: 10+
- Concurrent users: 50+
- Query timeout: 30 seconds
- Max query length: 10,000 characters

## Connection Details

**Endpoint:** https://b6fff44a-f670-4ad3-8d17-b8b3cf4709d3.europe-west3-0.gcp.cloud.qdrant.io

**Authentication:** API Key-based
- Key in environment: `QDRANT_API_KEY`
- Header: `api-key: {key}`

**Embedding Generation:**
- Service: Cohere
- Model: embed-english-v3.0
- Dimension: 1024
- Authentication: Cohere API key

**LLM Generation:**
- Service: OpenAI
- Model: gpt-4o-mini
- Temperature: 0.7
- Max tokens: 500

## Maintenance Notes

**Last Updated:** 2025-12-28
**Total Chapters:** 26
**Total Embeddings:** 26+ (chapter-level indexing)
**Update Frequency:** As new textbook content added
**Backup Status:** Cloud-hosted (automatic backups)

For updates to this schema, see project documentation.
