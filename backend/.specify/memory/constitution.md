# Physical AI RAG Chatbot Backend Constitution

## Core Principles

### I. Knowledge Source Verification (Non-Negotiable)
Every answer MUST be verifiable against stored sources. The chatbot answers only questions grounded in:
- Book content (chapters, sections, indexed passages)
- User-selected text passages (context-restricted mode)
- Retrieved academic chunks from vector database with explicit citations

Any response referencing the book requires a direct source link or passage ID. Never answer outside the verified knowledge base.

### II. Zero Hallucination Guarantee
All outputs undergo strict evaluation to prevent fabricated facts, invented citations, or unsupported claims.
- Retrieval must precede generation: only generate when relevant chunks found
- Confidence thresholds: reject low-confidence matches or ambiguous queries
- Citation requirement: every factual claim linked to source chunk ID, page/section
- Out-of-scope detection: explicitly flag queries unanswerable from knowledge base

### III. Academic Integrity & Citation Standards
RAG backend functions as academic research tool with rigorous citation practices:
- Every answer includes full citation: [Source ID / Page / Section]
- Answer format: derived claim + source evidence + confidence level
- No paraphrasing without explicit source linkage
- Failed retrievals reported clearly: "This question is not covered in current book content"

### IV. Test-First Answer Validation (Non-Negotiable)
Before deployment, all answer paths undergo fact-checking review:
- Unit tests verify retrieval accuracy (queries matched to correct chunks)
- Integration tests confirm end-to-end answer correctness against book facts
- Manual review: sample answers evaluated by domain expert (book author/editor)
- Grading criteria: factual accuracy, citation completeness, zero hallucinations

### V. FastAPI + Vector DB Architecture
Backend uses FastAPI (Python) with vector database (ERIA) for semantic retrieval.
- All endpoints require source verification before response
- Response schema: `{"answer": str, "sources": [{"id": str, "excerpt": str, "page": int}], "confidence": float}`
- Query timeout: 5 seconds (fail-safe to no-answer rather than timeout)
- Structured logging of all retrieval + generation steps for auditability

### VI. Context-Restricted Answers (User Passage Selection)
When users highlight or select a passage, answers derived exclusively from that context.
- Passage selection mode bypasses full vector search; uses only selected text
- Answers explicitly bounded: "Based on your selected passage..."
- Never expand beyond selected text unless user explicitly requests broader knowledge

## Security & Academic Integrity

### Data Handling
- Book content is authoritative source of truth
- No external API calls for facts without explicit caching + validation
- Access logs maintained for all queries and answer generations
- User queries logged for improvement (anonymized, no PII retention)

### Fact-Checking Review Gate
Before production deployment:
1. Sample 20% of answer-question pairs randomly
2. Domain expert validates factual accuracy against book
3. Require ≥95% accuracy score; failing answers blocked from deployment
4. Document all corrections as ADRs (architectural decision records)

## Observability & Quality Assurance

### Logging & Tracing
- Every query logged: timestamp, user query, retrieval query, chunks retrieved, generation latency
- Structured JSON logs for parsing and analysis
- Trace ID propagated across all services for debugging
- Failed retrievals flagged and categorized (out-of-scope vs. malformed query)

### Metrics & Monitoring
- Track: answer accuracy (manual sample review), retrieval precision, latency p95, hallucination rate
- Alert on hallucination detection (answers without valid sources or confidence < 0.7)
- Monthly accuracy audit: sample answers reviewed by domain expert

### Testing Gates (REQUIRED)
- Unit tests: ≥80% coverage for retrieval + embedding logic
- Integration tests: full query-to-answer flow with fact verification
- Regression tests: ensure updates don't degrade answer quality
- Grading: all PRs require passing fact-checker + manual review before merge

## Development Workflow

### Specification & Planning
- All features MUST include acceptance criteria tied to zero-hallucination requirement
- Plans MUST articulate source verification strategy and confidence threshold decisions
- Tasks MUST include fact-checking subtasks (manual review, grading scripts)

### Code Review Standards
- Every answer-generation change requires manual review by domain expert
- PRs blocked if grading scripts show hallucination or unverified claims
- Confidence threshold changes require ADR + stakeholder sign-off

### Deployment & Rollback
- Canary deployments: new models tested on 10% of traffic first
- Monitoring for hallucination rate increase within 24 hours post-deploy
- Instant rollback trigger: hallucination rate > 5% or accuracy drop > 2%
- Deployment approval: required from book author/editor (academic authority)

## Governance

Constitution supersedes all other practices and style preferences. This document governs all specifications, plans, tasks, and code for the RAG chatbot backend.

**Amendment Process:**
1. Identify architecturally significant decision or principle conflict
2. Document rationale, alternatives, tradeoffs in an ADR
3. Obtain approval from technical lead + academic authority (book author)
4. Update constitution and propagate changes to all dependent specs/plans/tasks
5. Version bump (MAJOR/MINOR/PATCH) and log in Sync Impact Report

**Compliance Review:**
- All PRs verify alignment with core principles (I–VI)
- Monthly constitution audit: spot-check code/tests against principles
- ADR reviews in /sp.adr workflow; architectural decisions cascade to constitution updates

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
