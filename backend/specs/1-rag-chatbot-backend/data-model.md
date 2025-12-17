# Data Model & Database Schema: RAG Chatbot Backend

**Date**: 2025-12-16
**Feature**: 1-rag-chatbot-backend
**Status**: Phase 1 Complete

---

## Entity Relationship Diagram

```
Document (1) ──→ (M) Chunk
   ↓
   └─ Metadata (title, chapter_id, page_range, etc.)

Query (1) ──→ (1) Answer
   ↓            ↓
   ├─ source_mode ── Citation (1) ──→ (M) Chunk
   ├─ selected_passage_id
   └─ timestamp

Answer (1) ──→ (M) Citation
   ├─ answer_text
   ├─ source_chunks[]
   ├─ confidence_score
   └─ timestamp

AuditLog (M) ──→ (1) Query
   ├─ captures full operation trace
   └─ timestamp

FactCheckGrade (1) ──→ (1) Answer
   ├─ accuracy_score
   ├─ hallucination_detected
   └─ reviewed_at
```

---

## Entity Definitions

### 1. Document (Book Chapter/Section)

**Purpose**: Store indexed book content metadata and relationships to chunks.

**Fields**:

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| document_id | UUID | Primary Key | Unique identifier |
| chapter_id | VARCHAR(50) | NOT NULL, UNIQUE | e.g., "ch01", "ch02_section3" |
| section_id | VARCHAR(100) | nullable | e.g., "sec_control_architectures" |
| title | VARCHAR(255) | NOT NULL | Chapter/section title |
| subtitle | VARCHAR(255) | nullable | Optional subtitle |
| text | TEXT | NOT NULL | Full chapter/section content (pre-chunking) |
| page_range | VARCHAR(20) | nullable | e.g., "42-67" |
| start_page | INT | nullable | First page number |
| end_page | INT | nullable | Last page number |
| metadata | JSONB | nullable | `{"author": "...", "publication_date": "2025-01-01", "tags": [...]}` |
| created_at | TIMESTAMPTZ | DEFAULT NOW() | Ingest timestamp |
| updated_at | TIMESTAMPTZ | DEFAULT NOW() | Last modification |
| status | VARCHAR(20) | DEFAULT 'active' | "active" \| "archived" \| "draft" |

**Indexes**:
```sql
CREATE UNIQUE INDEX idx_chapter_id ON documents(chapter_id);
CREATE INDEX idx_status ON documents(status);
CREATE INDEX idx_created_at ON documents(created_at DESC);
```

**Relationships**:
- (1:M) → Chunk: One document generates multiple chunks

**Validation Rules**:
- chapter_id: alphanumeric + underscore only; length 3–50
- title: length 1–255
- page_range: format "N-M" where N ≤ M; optional
- metadata: valid JSON object

---

### 2. Chunk (Atomic Searchable Unit)

**Purpose**: Represent 512-token chunks of book content, each independently searchable and embeddable.

**Fields**:

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| chunk_id | UUID | Primary Key | Unique identifier |
| document_id | UUID | Foreign Key | References Document.document_id |
| chunk_index | INT | NOT NULL | Sequential index within document (0, 1, 2, ...) |
| text | TEXT | NOT NULL | Chunk content (≤512 tokens enforced on ingest) |
| token_count | INT | NOT NULL | Actual token count (for validation) |
| start_page | INT | nullable | Page where chunk begins |
| end_page | INT | nullable | Page where chunk ends |
| start_sentence | INT | nullable | Sentence boundary start index |
| end_sentence | INT | nullable | Sentence boundary end index |
| embedding_vector | vector(768) | NOT NULL | Cohere Embed vector (768-dim) |
| embedding_model | VARCHAR(50) | DEFAULT 'cohere-embed' | Model used for embedding |
| embedding_confidence | FLOAT | NOT NULL | Quality score of embedding (0.0–1.0) |
| created_at | TIMESTAMPTZ | DEFAULT NOW() | Embedding timestamp |
| status | VARCHAR(20) | DEFAULT 'active' | "active" \| "deprecated" \| "superseded" |

**Indexes**:
```sql
CREATE INDEX idx_document_id ON chunks(document_id);
CREATE INDEX idx_chunk_index ON chunks(document_id, chunk_index);
CREATE INDEX idx_status ON chunks(status);
-- Vector index (Qdrant) separate; managed via Qdrant SDK
```

**Relationships**:
- (M:1) → Document: Many chunks reference one document
- (1:M) → Citation: One chunk cited in multiple answers
- (1:M) → FactCheckGrade: Chunk can be reviewed in multiple grades

**Validation Rules**:
- text: length 256–512 tokens (min to max)
- token_count: must match actual tokenization of text
- embedding_vector: 768 dimensions (Cohere standard)
- embedding_confidence: float [0.0–1.0]

**Storage Notes**:
- Text stored in PostgreSQL (for audit + metadata context)
- Vector stored in Qdrant (for semantic search) + PostgreSQL (for backup)
- Denormalization intentional: fast queries without Qdrant round-trip

---

### 3. Query (User Question + Metadata)

**Purpose**: Track user queries submitted to the chatbot; enables auditing and multi-turn support (future).

**Fields**:

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| query_id | UUID | Primary Key | Unique identifier |
| user_query_text | TEXT | NOT NULL, Length 1–1000 | Raw user input |
| source_mode | VARCHAR(20) | DEFAULT 'full-book' | "full-book" \| "context-restricted" |
| selected_passage_id | UUID | nullable, FK | References Chunk (if context-restricted mode) |
| selected_passage_text | TEXT | nullable | Copy of selected passage (for audit) |
| conversation_id | UUID | nullable | For future multi-turn support |
| client_api_key_hash | VARCHAR(64) | NOT NULL | SHA-256 hash of API key (no plaintext) |
| submitted_at | TIMESTAMPTZ | DEFAULT NOW() | Submission timestamp |
| response_latency_ms | INT | nullable | Time to generate answer (populated after response) |
| status | VARCHAR(20) | DEFAULT 'submitted' | "submitted" \| "processing" \| "answered" \| "error" \| "timeout" |

**Indexes**:
```sql
CREATE INDEX idx_conversation_id ON queries(conversation_id);
CREATE INDEX idx_submitted_at ON queries(submitted_at DESC);
CREATE INDEX idx_status ON queries(status);
CREATE INDEX idx_client_key ON queries(client_api_key_hash);
```

**Relationships**:
- (1:1) → Answer: One query results in one answer (or error)
- (1:M) → AuditLog: Query referenced in audit logs
- (M:1) → Chunk (selected_passage_id): If context-restricted mode

**Validation Rules**:
- user_query_text: length 1–1000 characters; UTF-8 valid
- source_mode: enum ["full-book", "context-restricted"]
- selected_passage_id: required if source_mode = "context-restricted"
- client_api_key_hash: SHA-256 hash (not stored plaintext for security)

---

### 4. Answer (Generated Response + Provenance)

**Purpose**: Store LLM-generated answers with full provenance, citations, and confidence.

**Fields**:

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| answer_id | UUID | Primary Key | Unique identifier |
| query_id | UUID | Foreign Key, NOT NULL | References Query.query_id |
| answer_text | TEXT | NOT NULL | Generated response (with inline citations) |
| answer_summary | VARCHAR(500) | nullable | Brief summary for logging |
| source_chunk_ids | UUID[] | NOT NULL | Array of chunk_ids used as sources |
| confidence_score | FLOAT | NOT NULL | Combined confidence (0.0–1.0) |
| retrieval_confidence | FLOAT | NOT NULL | Chunk retrieval quality (0.6–1.0) |
| generation_confidence | FLOAT | NOT NULL | Claude's self-assessed confidence |
| citations | JSONB | NOT NULL | Structured citations: `[{"chunk_id": "...", "excerpt": "...", "page": 42, "section": "..."}]` |
| llm_model | VARCHAR(50) | DEFAULT 'claude-3.5-sonnet' | Model used |
| llm_temperature | FLOAT | DEFAULT 0.2 | Temperature used (low for academic) |
| generated_at | TIMESTAMPTZ | DEFAULT NOW() | Generation timestamp |
| status | VARCHAR(20) | DEFAULT 'pending_review' | "pending_review" \| "approved" \| "rejected" \| "flagged" |
| audit_trail_id | UUID | Foreign Key | References AuditLog.log_id |

**Indexes**:
```sql
CREATE INDEX idx_query_id ON answers(query_id);
CREATE INDEX idx_status ON answers(status);
CREATE INDEX idx_generated_at ON answers(generated_at DESC);
CREATE INDEX idx_chunk_ids ON answers USING GIN(source_chunk_ids);
```

**Relationships**:
- (1:1) → Query: Answers reference single query
- (M:1) → Chunk (via source_chunk_ids): Multiple chunks referenced
- (1:M) → Citation: Multiple citation records per answer
- (1:1) → FactCheckGrade: Answer reviewed by domain expert
- (1:1) → AuditLog: Full trace in audit log

**Validation Rules**:
- confidence_score: float [0.0–1.0]
- retrieval_confidence: float [0.6–1.0] (minimum 0.60 enforced in service)
- generation_confidence: float [0.0–1.0]
- source_chunk_ids: non-empty array; all IDs must exist in Chunk table
- citations: valid JSON array with citation objects
- status: enum ["pending_review", "approved", "rejected", "flagged"]

**Computed Fields** (derived in service layer):
- confidence_level (string): "high" (>0.85), "medium" (0.70–0.85), "low" (<0.70)

---

### 5. Citation (Reference to Source Chunk)

**Purpose**: Detailed reference information for each cited chunk; enables citation validation and tracing.

**Fields**:

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| citation_id | UUID | Primary Key | Unique identifier |
| answer_id | UUID | Foreign Key, NOT NULL | References Answer.answer_id |
| chunk_id | UUID | Foreign Key, NOT NULL | References Chunk.chunk_id |
| excerpt | TEXT | NOT NULL | 30–50 word quote from chunk |
| page_number | INT | NOT NULL | Page where excerpt appears |
| section_heading | VARCHAR(255) | NOT NULL | Section name (for navigation) |
| sentence_indices | INT[] | nullable | Sentence positions within chunk |
| source_confidence | FLOAT | NOT NULL | Cohere similarity score for this chunk (0.0–1.0) |
| is_primary | BOOLEAN | DEFAULT FALSE | True if primary source for claim; false if supporting |
| created_at | TIMESTAMPTZ | DEFAULT NOW() | Citation creation timestamp |

**Indexes**:
```sql
CREATE INDEX idx_answer_id ON citations(answer_id);
CREATE INDEX idx_chunk_id ON citations(chunk_id);
CREATE INDEX idx_page_number ON citations(page_number);
```

**Relationships**:
- (M:1) → Answer: Multiple citations per answer
- (M:1) → Chunk: Multiple citations can reference same chunk

**Validation Rules**:
- excerpt: length 30–500 characters
- page_number: positive integer
- section_heading: length 1–255
- source_confidence: float [0.0–1.0]
- is_primary: boolean

---

### 6. AuditLog (Complete Operation Trace)

**Purpose**: Denormalized audit trail capturing full context of each query-answer operation for compliance, debugging, and fact-checking.

**Fields**:

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| log_id | UUID | Primary Key | Unique identifier |
| query_id | UUID | Foreign Key, NOT NULL | References Query.query_id |
| answer_id | UUID | Foreign Key, nullable | References Answer.answer_id (null if error) |
| user_query_text | TEXT | NOT NULL | Copy of query text (for audit independence) |
| source_mode | VARCHAR(20) | NOT NULL | "full-book" \| "context-restricted" |
| retrieval_query | TEXT | nullable | Processed query sent to Qdrant |
| chunks_retrieved_count | INT | nullable | Number of chunks returned by Qdrant |
| chunks_returned | JSONB | nullable | Top-5 chunks + scores: `[{"id": "...", "score": 0.92, "text": "..."}]` |
| retrieval_latency_ms | INT | nullable | Time to retrieve chunks |
| llm_prompt | TEXT | nullable | Full system + user prompt sent to Claude |
| llm_response | TEXT | nullable | Raw Claude response (before formatting) |
| answer_text | TEXT | nullable | Final formatted answer |
| confidence_score | FLOAT | nullable | Answer confidence score |
| citations | JSONB | nullable | Structured citations (copy from Answer) |
| success | BOOLEAN | NOT NULL | True if answered; false if error/timeout |
| error_code | VARCHAR(50) | nullable | "TIMEOUT" \| "LOW_CONFIDENCE" \| "NO_MATCH" \| "INVALID_QUERY" |
| error_message | TEXT | nullable | Human-readable error |
| response_time_ms | INT | NOT NULL | Total query-to-response time |
| timestamp | TIMESTAMPTZ | DEFAULT NOW() | Operation timestamp |
| client_api_key_hash | VARCHAR(64) | NOT NULL | Which client submitted query |

**Indexes**:
```sql
CREATE INDEX idx_query_id ON audit_logs(query_id);
CREATE INDEX idx_answer_id ON audit_logs(answer_id);
CREATE INDEX idx_timestamp ON audit_logs(timestamp DESC);
CREATE INDEX idx_success ON audit_logs(success);
CREATE INDEX idx_error_code ON audit_logs(error_code);
-- Partitioned by timestamp for performance (monthly partitions)
```

**Relationships**:
- (M:1) → Query: Multiple logs for same query (retries, if applicable)
- (M:1) → Answer: Multiple logs per answer (if multi-step generation)

**Retention Policy**:
- Hot (PostgreSQL): 90 days
- Warm (PostgreSQL): 365 days
- Cold (S3/Archive): >365 days (compressed, queryable)

**Validation Rules**:
- success: boolean
- error_code: nullable, enum or null if success=true
- response_time_ms: positive integer
- chunks_retrieved_count: 0–1000

---

### 7. FactCheckGrade (Domain Expert Review)

**Purpose**: Record manual fact-checking review by domain expert; enables compliance audits and accuracy tracking.

**Fields**:

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| grade_id | UUID | Primary Key | Unique identifier |
| answer_id | UUID | Foreign Key, NOT NULL | References Answer.answer_id (1:1 relationship) |
| reviewer_id | VARCHAR(100) | NOT NULL | Domain expert ID (book author/editor) |
| accuracy_score | INT | NOT NULL | 0–100 percentage |
| hallucination_detected | BOOLEAN | NOT NULL | True if answer contains fabricated facts |
| factual_errors_count | INT | DEFAULT 0 | Number of incorrect claims |
| citation_errors_count | INT | DEFAULT 0 | Number of incorrect/missing citations |
| comments | TEXT | nullable | Reviewer notes and corrections |
| corrections | JSONB | nullable | Structured corrections: `[{"claim": "...", "correction": "...", "source": "..."}]` |
| approved_for_production | BOOLEAN | NOT NULL | True if answer passes review; false if needs revision |
| reviewed_at | TIMESTAMPTZ | DEFAULT NOW() | Review completion timestamp |
| review_duration_minutes | INT | nullable | Time spent reviewing |

**Indexes**:
```sql
CREATE UNIQUE INDEX idx_answer_id_grade ON fact_check_grades(answer_id);
CREATE INDEX idx_reviewer_id ON fact_check_grades(reviewer_id);
CREATE INDEX idx_approved ON fact_check_grades(approved_for_production);
CREATE INDEX idx_reviewed_at ON fact_check_grades(reviewed_at DESC);
```

**Relationships**:
- (1:1) → Answer: One grade per answer (when reviewed)
- (M:1) → Reviewer: Multiple reviews by same expert

**Validation Rules**:
- accuracy_score: integer [0–100]
- hallucination_detected: boolean
- factual_errors_count: non-negative integer
- citation_errors_count: non-negative integer
- approved_for_production: boolean (must be true if accuracy_score ≥95 per constitutional requirement)
- reviewer_id: non-empty string

**Approval Logic**:
```python
if accuracy_score >= 95 and hallucination_detected == False:
    approved_for_production = True
else:
    approved_for_production = False  # Requires revision
```

---

## Database Schema (SQL DDL)

```sql
-- Create extensions
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";
CREATE EXTENSION IF NOT EXISTS "vector";

-- Document table
CREATE TABLE documents (
  document_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chapter_id VARCHAR(50) NOT NULL UNIQUE,
  section_id VARCHAR(100),
  title VARCHAR(255) NOT NULL,
  subtitle VARCHAR(255),
  text TEXT NOT NULL,
  page_range VARCHAR(20),
  start_page INT,
  end_page INT,
  metadata JSONB,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW(),
  status VARCHAR(20) DEFAULT 'active'
);

CREATE UNIQUE INDEX idx_document_chapter_id ON documents(chapter_id);
CREATE INDEX idx_document_status ON documents(status);
CREATE INDEX idx_document_created_at ON documents(created_at DESC);

-- Chunk table
CREATE TABLE chunks (
  chunk_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  document_id UUID NOT NULL REFERENCES documents(document_id),
  chunk_index INT NOT NULL,
  text TEXT NOT NULL,
  token_count INT NOT NULL,
  start_page INT,
  end_page INT,
  start_sentence INT,
  end_sentence INT,
  embedding_vector vector(768) NOT NULL,
  embedding_model VARCHAR(50) DEFAULT 'cohere-embed',
  embedding_confidence FLOAT NOT NULL,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  status VARCHAR(20) DEFAULT 'active'
);

CREATE INDEX idx_chunk_document_id ON chunks(document_id);
CREATE INDEX idx_chunk_index ON chunks(document_id, chunk_index);
CREATE INDEX idx_chunk_status ON chunks(status);

-- Query table
CREATE TABLE queries (
  query_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_query_text TEXT NOT NULL,
  source_mode VARCHAR(20) DEFAULT 'full-book',
  selected_passage_id UUID REFERENCES chunks(chunk_id),
  selected_passage_text TEXT,
  conversation_id UUID,
  client_api_key_hash VARCHAR(64) NOT NULL,
  submitted_at TIMESTAMPTZ DEFAULT NOW(),
  response_latency_ms INT,
  status VARCHAR(20) DEFAULT 'submitted'
);

CREATE INDEX idx_query_conversation_id ON queries(conversation_id);
CREATE INDEX idx_query_submitted_at ON queries(submitted_at DESC);
CREATE INDEX idx_query_status ON queries(status);

-- Answer table
CREATE TABLE answers (
  answer_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  query_id UUID NOT NULL UNIQUE REFERENCES queries(query_id),
  answer_text TEXT NOT NULL,
  answer_summary VARCHAR(500),
  source_chunk_ids UUID[] NOT NULL,
  confidence_score FLOAT NOT NULL,
  retrieval_confidence FLOAT NOT NULL,
  generation_confidence FLOAT NOT NULL,
  citations JSONB NOT NULL,
  llm_model VARCHAR(50) DEFAULT 'claude-3.5-sonnet',
  llm_temperature FLOAT DEFAULT 0.2,
  generated_at TIMESTAMPTZ DEFAULT NOW(),
  status VARCHAR(20) DEFAULT 'pending_review',
  audit_trail_id UUID
);

CREATE INDEX idx_answer_query_id ON answers(query_id);
CREATE INDEX idx_answer_status ON answers(status);
CREATE INDEX idx_answer_generated_at ON answers(generated_at DESC);

-- Citation table
CREATE TABLE citations (
  citation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  answer_id UUID NOT NULL REFERENCES answers(answer_id),
  chunk_id UUID NOT NULL REFERENCES chunks(chunk_id),
  excerpt TEXT NOT NULL,
  page_number INT NOT NULL,
  section_heading VARCHAR(255) NOT NULL,
  sentence_indices INT[],
  source_confidence FLOAT NOT NULL,
  is_primary BOOLEAN DEFAULT FALSE,
  created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_citation_answer_id ON citations(answer_id);
CREATE INDEX idx_citation_chunk_id ON citations(chunk_id);

-- AuditLog table (partitioned by timestamp)
CREATE TABLE audit_logs (
  log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  query_id UUID NOT NULL REFERENCES queries(query_id),
  answer_id UUID REFERENCES answers(answer_id),
  user_query_text TEXT NOT NULL,
  source_mode VARCHAR(20) NOT NULL,
  retrieval_query TEXT,
  chunks_retrieved_count INT,
  chunks_returned JSONB,
  retrieval_latency_ms INT,
  llm_prompt TEXT,
  llm_response TEXT,
  answer_text TEXT,
  confidence_score FLOAT,
  citations JSONB,
  success BOOLEAN NOT NULL,
  error_code VARCHAR(50),
  error_message TEXT,
  response_time_ms INT NOT NULL,
  timestamp TIMESTAMPTZ DEFAULT NOW(),
  client_api_key_hash VARCHAR(64) NOT NULL
) PARTITION BY RANGE (timestamp);

-- Monthly partitions for audit_logs
CREATE TABLE audit_logs_2025_12 PARTITION OF audit_logs
  FOR VALUES FROM ('2025-12-01') TO ('2026-01-01');

CREATE INDEX idx_audit_query_id ON audit_logs(query_id);
CREATE INDEX idx_audit_timestamp ON audit_logs(timestamp DESC);

-- FactCheckGrade table
CREATE TABLE fact_check_grades (
  grade_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  answer_id UUID NOT NULL UNIQUE REFERENCES answers(answer_id),
  reviewer_id VARCHAR(100) NOT NULL,
  accuracy_score INT NOT NULL,
  hallucination_detected BOOLEAN NOT NULL,
  factual_errors_count INT DEFAULT 0,
  citation_errors_count INT DEFAULT 0,
  comments TEXT,
  corrections JSONB,
  approved_for_production BOOLEAN NOT NULL,
  reviewed_at TIMESTAMPTZ DEFAULT NOW(),
  review_duration_minutes INT
);

CREATE INDEX idx_factcheck_answer_id ON fact_check_grades(answer_id);
CREATE INDEX idx_factcheck_reviewer_id ON fact_check_grades(reviewer_id);
CREATE INDEX idx_factcheck_approved ON fact_check_grades(approved_for_production);
```

---

## State Machines

### Query Lifecycle

```
submitted → processing → answered
                    ↓
                   error (timeout | invalid_query | low_confidence)
```

### Answer Lifecycle

```
pending_review → approved (production-ready)
       ↓
     flagged (hallucination detected)
       ↓
     rejected (accuracy <95%)
```

---

## Data Flow Example

```
1. User submits query via API
   → Query record created (status: "submitted")

2. Retrieval service searches Qdrant
   → Cohere Embed similarity scores returned
   → Top-5 chunks ranked by hybrid score

3. Generation service calls Claude
   → System prompt + chunks + query sent to Claude
   → Claude returns answer + confidence estimate

4. Citation service injects citations
   → Extract chunks referenced in answer
   → Create Citation records with page numbers + excerpts

5. Answer record created (status: "pending_review")
   → Store in PostgreSQL
   → AuditLog record created with full trace

6. Domain expert samples answer for review
   → FactCheckGrade record created
   → If accuracy ≥95%, approved_for_production = true
   → Else, flagged for revision

7. Query lifecycle: submitted → processing → answered
   Answer lifecycle: pending_review → approved
```

---

## Performance Considerations

| Table | Estimated Size | Query Pattern | Index Strategy |
|-------|----------------|---------------|-----------------|
| documents | Small (1–10 rows) | Infrequent | Primary key lookup |
| chunks | Large (10k–100k rows) | Frequent (vector search in Qdrant) | B-tree on document_id, chunk_index |
| queries | Medium (1k–10k rows/month) | Temporal range queries | B-tree on timestamp |
| answers | Medium (100–1k rows/month) | Query lookup, status filtering | B-tree on query_id, status |
| citations | Medium (500–5k rows/month) | Answer lookup | B-tree on answer_id |
| audit_logs | Very Large (10k–100k rows/month) | Temporal range, success filtering | Partitioned by month; B-tree on timestamp, success |
| fact_check_grades | Small (20–30 rows/month) | Approval filtering | B-tree on approved_for_production |

**Optimization Notes**:
- Chunk vectors: Qdrant (specialized vector DB) handles embeddings; PostgreSQL stores text backup only
- AuditLog partitioning: Monthly partitions enable fast archival + retention management
- Denormalized audit_logs: Accept duplication for query speed (audit independence)

---

**Phase 1 Status**: ✅ **DATA MODEL COMPLETE**

Ready for API contract design (`contracts/`) and quickstart documentation.
