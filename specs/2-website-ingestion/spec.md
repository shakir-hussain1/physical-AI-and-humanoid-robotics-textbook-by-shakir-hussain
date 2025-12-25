# Website URL Ingestion & Vector Storage Specification

## Version
**Spec Version:** 1.0.0
**Created:** 2025-12-25
**Last Updated:** 2025-12-25

## Feature Overview
**Feature:** Website URL Ingestion & Vector Storage
**Module Alignment:** Unified Book RAG System (Backend Infrastructure)
**Target Audience:** Developers and AI engineers maintaining the unified book RAG system

## Constitutional Alignment
This specification aligns with the following principles:
- Technical Excellence and Accuracy
- Practical Application
- Code Quality and Testing
- Documentation and Knowledge Transfer

## Problem Statement
The unified book RAG system requires reliable ingestion of content from Docusaurus-deployed websites, generation of high-quality embeddings, and persistent storage in a vector database. Currently, there is no standardized pipeline for crawling website URLs, chunking content while preserving semantic meaning, embedding with Cohere, and storing in Qdrant Cloud. This feature establishes the foundational ingestion layer for the RAG retrieval system.

## Requirements

### Functional Requirements
- **REQ-1:** System must crawl and extract text content from deployed Docusaurus website URLs without requiring authentication
- **REQ-2:** System must parse extracted content into semantic chunks that preserve context (max 1024 tokens per chunk, with overlap for continuity)
- **REQ-3:** System must generate embeddings for each chunk using Cohere's latest stable embedding model
- **REQ-4:** System must store embeddings, chunk metadata (URL, page title, chunk index), and original text in Qdrant Cloud with proper vector IDs
- **REQ-5:** System must track ingestion status (success/failure) with clear error messages for failed URLs or chunks
- **REQ-6:** System must verify data persistence by querying stored vectors and validating retrieved chunk IDs and metadata
- **REQ-7:** System must support batch ingestion of multiple URLs in a single operation
- **REQ-8:** System must be configurable via environment variables (API keys, URLs, Qdrant endpoints)

### Non-Functional Requirements
- **NFR-1:** Ingestion pipeline must complete processing a typical book chapter (15-20 KB) within 30 seconds
- **NFR-2:** Embedding API calls must be retried up to 3 times on transient failures with exponential backoff
- **NFR-3:** Vector storage operations must succeed with 99% reliability within Free Tier Qdrant quota limits
- **NFR-4:** Logging must capture all major operations (URL fetched, chunks created, embeddings generated, vectors stored) at INFO level with errors at ERROR level

### Constraints
- Language: Python
- Backend: FastAPI-compatible services
- Embeddings: Cohere (latest stable embedding model)
- Vector DB: Qdrant Cloud (Free Tier)
- Chunking strategy must preserve semantic context and avoid breaking mid-sentence
- All API keys and sensitive URLs must be environment-based (no hardcoding)
- No authentication required for target websites (Docusaurus public documentation)

## Solution Approach

### High-Level Design
The ingestion pipeline follows a three-stage process:
1. **Content Fetching:** Crawl Docusaurus URLs and extract plain text using BeautifulSoup
2. **Chunking & Embedding:** Split text into semantic chunks and generate embeddings using Cohere API
3. **Vector Storage:** Persist embeddings and metadata in Qdrant Cloud with verification queries

### Technical Architecture
```
URL Input
  ↓
[Web Crawler] (BeautifulSoup/requests)
  ↓
Raw HTML/Text
  ↓
[Text Extractor] (MD parsing)
  ↓
Extracted Text
  ↓
[Semantic Chunker] (token-aware, sentence boundaries)
  ↓
Text Chunks + Metadata
  ↓
[Cohere Embedding Service] (batch API calls)
  ↓
Embeddings (1024-dimensional vectors)
  ↓
[Qdrant Storage] (vector persistence + metadata indexing)
  ↓
Verification Queries (ID checks, metadata validation)
```

### Data Flow
1. User provides URL list and target collection name
2. System fetches and parses each URL into markdown
3. Text is chunked with preserved semantic boundaries (sentences, paragraphs)
4. Chunks are sent in batches to Cohere embedding API
5. Embeddings + metadata are stored in Qdrant with unique IDs
6. System logs success/failure for each URL and chunk
7. Verification queries confirm vector storage integrity

## Detailed Design

### Component 1: URL Crawler & Text Extractor
- **Purpose:** Fetch content from Docusaurus URLs and extract clean text
- **Interfaces:**
  - Input: URL string
  - Output: Extracted text, page title, source URL
- **Dependencies:** requests, BeautifulSoup4, logging module

### Component 2: Semantic Chunker
- **Purpose:** Split text into overlapping chunks that preserve context and sentence boundaries
- **Interfaces:**
  - Input: Text string, max_chunk_tokens (default 1024)
  - Output: List of chunks with metadata (chunk_index, char_range)
- **Dependencies:** tiktoken for token counting

### Component 3: Cohere Embedding Service
- **Purpose:** Generate embeddings for text chunks using Cohere API
- **Interfaces:**
  - Input: List of text chunks, embedding model name
  - Output: List of embedding vectors (float arrays)
- **Dependencies:** cohere Python SDK, environment variable for API key

### Component 4: Qdrant Vector Storage
- **Purpose:** Persist embeddings and metadata in Qdrant Cloud
- **Interfaces:**
  - Input: Embeddings, chunk metadata, collection name
  - Output: Stored vector IDs, confirmation status
- **Dependencies:** qdrant-client, environment variables for API endpoint and API key

### Component 5: Logging & Error Handling
- **Purpose:** Track ingestion progress and capture errors
- **Interfaces:**
  - Input: Operation name, status, error details
  - Output: Log entries at appropriate levels (INFO, ERROR)
- **Dependencies:** Python logging module

## User Experience

### Learning Objectives
- Understand how website content is ingested and converted into vector embeddings
- Learn semantic chunking strategies for preserving context in long documents
- Understand vector database persistence and query verification patterns
- Be able to monitor and troubleshoot ingestion pipelines

### Content Structure
- **Section 1:** Overview of the RAG ingestion pipeline and its role in retrieval
- **Section 2:** Web content extraction and parsing from Docusaurus sites
- **Section 3:** Semantic chunking strategies and their impact on retrieval quality
- **Section 4:** Embedding generation using Cohere and integration with Qdrant
- **Section 5:** Operational patterns: monitoring, error handling, verification

## Implementation Requirements

### Technology Stack
- **Primary Framework:** FastAPI (for API endpoints)
- **Programming Languages:** Python 3.10+
- **Key Libraries:**
  - requests (HTTP fetching)
  - BeautifulSoup4 (HTML/MD parsing)
  - cohere (embedding API)
  - qdrant-client (vector storage)
  - tiktoken (token counting)
  - python-dotenv (environment configuration)
  - logging (standard library)

### Code Standards
- Follow PEP 8 style guide
- Use type hints for all function signatures
- Include docstrings for classes and methods
- Use structured logging with context (URL, chunk_id, operation)
- No hardcoded secrets; all API keys via environment variables
- Error messages must be actionable (include URL, error type, retry status)

### Testing Requirements
- **Unit Tests:** Test chunking logic with edge cases (empty text, very long text, non-ASCII), embedding API mocks
- **Integration Tests:** Test URL crawling with sample Docusaurus pages, full pipeline with test Qdrant instance
- **Validation Tests:** Verify stored vectors can be queried by ID, metadata matches original chunks, embedding dimensions are correct

## Quality Assurance

### Technical Validation
- Verify chunking preserves semantic boundaries (no mid-sentence breaks)
- Test with sample book chapters and validate chunk quality
- Validate Cohere API integration with mock responses
- Test Qdrant storage and retrieval with sample embeddings
- Verify error handling for network timeouts, API rate limits, malformed URLs

### Operational Validation
- Monitor ingestion logs for success/failure rates
- Spot-check stored vectors to ensure metadata accuracy
- Test Free Tier Qdrant quota limits and implement safeguards
- Validate that all operations complete within performance targets

## Dependencies and Integrations

### Internal Dependencies
- Depends on: Backend FastAPI application structure
- No dependencies on retrieval or query components (this is ingestion-only)

### External Dependencies
- **Cohere API:** Latest stable embedding model (currently `embed-english-v3.0` or equivalent)
- **Qdrant Cloud:** Free Tier instance with appropriate API key
- **BeautifulSoup4:** For HTML/MD parsing (standard open-source)
- **tiktoken:** For accurate token counting (OpenAI library)

## Risks and Mitigation

### Technical Risks
- **Risk 1:** Website structure changes break content extraction
  - **Mitigation:** Implement fallback extraction strategies; log extraction failures with sample HTML for debugging

- **Risk 2:** Cohere API rate limits or failures interrupt ingestion
  - **Mitigation:** Implement exponential backoff with max 3 retries; queue failed chunks for retry batch

- **Risk 3:** Qdrant Free Tier quota exceeded during bulk ingestion
  - **Mitigation:** Monitor vector count before ingestion; implement progress checkpoints to allow resume-on-failure

### Operational Risks
- **Risk 1:** Missing or incorrect environment variables cause silent failures
  - **Mitigation:** Validate all required env vars at startup; provide clear error messages

- **Risk 2:** Large-scale ingestion consumes API quota without visibility
  - **Mitigation:** Log API costs (estimated based on token count); provide summary report after ingestion

## Success Criteria

### Quantitative Measures
- Successfully crawl and ingest 100% of provided URLs without data loss
- Chunks are created with semantic accuracy (no more than 5% mid-sentence breaks in sample review)
- Embeddings are stored with 100% retrieval accuracy (all vectors queryable by stored ID within 1 second)
- Ingestion logs capture 100% of operations with timestamp and status
- System completes processing of a 20 KB chapter within 30 seconds

### Qualitative Measures
- Error messages are actionable and include context (URL, operation, suggested next steps)
- Logging output enables developers to troubleshoot ingestion failures independently
- Metadata storage allows tracing retrieved chunks back to original URLs and page content
- Documentation provides clear setup, configuration, and usage examples

## Assumptions

- Target websites (Docusaurus documentation) are publicly accessible without authentication
- Text content is in English or translatable to vectors by Cohere
- Chunking at 1024 tokens preserves sufficient context for effective retrieval
- Qdrant Free Tier provides sufficient vector storage for initial book ingestion
- Cohere embedding model produces 1024-dimensional vectors (standard)
- Network connectivity to Cohere API and Qdrant Cloud is stable during ingestion

## References and Citations
- Docusaurus: https://docusaurus.io/
- Cohere API Documentation: https://docs.cohere.com/
- Qdrant Vector Database: https://qdrant.tech/
- BeautifulSoup4: https://www.crummy.com/software/BeautifulSoup/
- Token Counting (tiktoken): https://github.com/openai/tiktoken

---
*This specification defines the ingestion layer for the unified book RAG system and focuses on reliable, observable content preparation for retrieval workflows.*
