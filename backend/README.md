# RAG Website Ingestion & Vector Storage - Backend

Complete Python pipeline for ingesting Docusaurus websites, chunking content, generating embeddings, and storing vectors in Qdrant Cloud.

## Quick Start

### 1. Prerequisites

- Python 3.10+
- UV package manager (https://docs.astral.sh/uv/)
- Cohere API key (https://dashboard.cohere.com/)
- Qdrant Cloud account (https://cloud.qdrant.io/)

### 2. Setup

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
uv venv

# Activate virtual environment
# On Linux/macOS:
source .venv/bin/activate
# On Windows:
.venv\Scripts\activate

# Install dependencies
uv pip install -e .
```

### 3. Configure Environment

Copy `.env.ingestion.example` to `.env` and fill in your credentials:

```bash
cp .env.ingestion.example .env
# Edit .env with your API keys
```

Required variables:
- `COHERE_API_KEY` — Your Cohere API key
- `QDRANT_URL` — Your Qdrant Cloud endpoint (e.g., https://xxxxx.qdrant.io)
- `QDRANT_API_KEY` — Your Qdrant API key

Optional variables:
- `COHERE_EMBEDDING_MODEL` — Embedding model (default: `embed-english-v3.0`)
- `LOG_LEVEL` — Logging level (default: `INFO`)
- `MAX_RETRIES` — Max retry attempts (default: `3`)
- `REQUEST_TIMEOUT_SECONDS` — HTTP timeout (default: `30`)

### 4. Configure URLs

Edit `backend/main.py` and update the `urls` list in the `main()` function:

```python
urls = [
    "https://your-book.com/intro",
    "https://your-book.com/chapter-1",
    "https://your-book.com/chapter-2",
]
collection_name = "your_collection_name"
```

### 5. Run Pipeline

```bash
python main.py
```

Expected output:
```
============================================================
RAG Website Ingestion Pipeline Started
============================================================
2025-12-25 10:00:00,000 - __main__ - INFO - Initializing services...
2025-12-25 10:00:01,000 - __main__ - INFO - Creating collection 'book_embeddings'
2025-12-25 10:00:02,000 - __main__ - INFO - Processing URL: https://...
...
2025-12-25 10:00:30,000 - __main__ - INFO - ✓ Vector storage verification passed
============================================================
Ingestion Complete
  Total URLs processed: 3
  Successful URLs: 3
  Total chunks created: 45
  Total embeddings generated: 45
  Collection: book_embeddings
============================================================
```

---

## Project Structure

```
backend/
├── main.py                    # Main pipeline with all components
├── pyproject.toml             # Dependencies and project config
├── .env.ingestion.example     # Environment variable template
├── .gitignore                 # Git ignore rules
├── README.md                  # This file
└── logs/
    ├── .gitkeep               # Directory placeholder
    └── ingestion.log          # Pipeline logs (created at runtime)
```

---

## Components

### IngestionConfig
Loads and validates environment variables at startup.

**Validates:**
- COHERE_API_KEY (required)
- QDRANT_URL (required)
- QDRANT_API_KEY (required)

**Provides defaults:**
- COHERE_EMBEDDING_MODEL = "embed-english-v3.0"
- MAX_RETRIES = 3
- REQUEST_TIMEOUT_SECONDS = 30

### WebCrawler
Fetches HTML from URLs and extracts text content using BeautifulSoup.

**Methods:**
- `fetch_url(url) → (text, title) | (None, None)`

**Features:**
- Extracts from `<main>`, `<article>`, or `<body>` tags
- Logs HTTP errors and timeouts as warnings
- Returns None for empty or unparseable content

### SemanticChunker
Splits text into semantic chunks while preserving sentence boundaries.

**Methods:**
- `chunk_text(text, url, title) → List[chunk_dict]`

**Features:**
- Splits on paragraph boundaries first
- Respects max 1024 tokens per chunk
- Preserves metadata (URL, page title, chunk index)
- Uses tiktoken for accurate token counting

**Output format:**
```python
{
    "text": "chunk content here...",
    "tokens": 512,
    "chunk_index": 0,
    "metadata": {
        "url": "https://...",
        "page_title": "Page Title"
    }
}
```

### EmbeddingService
Generates embeddings using Cohere API with exponential backoff retry logic.

**Methods:**
- `embed_chunks(chunks) → List[List[float]] | None`

**Features:**
- Batches API calls (max 100 chunks per call)
- Exponential backoff retries (1s, 2s, 4s)
- Returns None after max 3 retries
- Logs all attempts

**Output:**
- List of 1024-dimensional embedding vectors

### QdrantStorage
Stores embeddings in Qdrant Cloud with metadata and verification.

**Methods:**
- `create_collection(name) → bool`
- `store_vectors(collection_name, embeddings, metadata) → bool`
- `verify_vectors(collection_name, sample_size) → bool`

**Features:**
- Creates cosine-distance collection if not exists
- Upserts vectors with payload metadata
- Verifies storage by sampling vectors
- Handles quota limits gracefully

---

## Customization

### Change Chunk Size
Modify `SemanticChunker` initialization:
```python
chunker = SemanticChunker(max_tokens=512)  # Smaller chunks
chunker = SemanticChunker(max_tokens=2048)  # Larger chunks
```

### Change Embedding Model
Update `.env`:
```bash
COHERE_EMBEDDING_MODEL=embed-english-light-v3.0
```

### Add Retry Logic Customization
Modify `EmbeddingService`:
```python
embedder = EmbeddingService(
    config.cohere_api_key,
    config.cohere_model,
    max_retries=5  # More retries
)
```

### Batch Processing Multiple Collections
Edit `main()` to iterate over collection names:
```python
collections = ["book1_embeddings", "book2_embeddings"]
for collection_name in collections:
    # Process URLs for each collection
```

---

## Troubleshooting

### Missing Environment Variables
```
ERROR: Missing required environment variables: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
```
**Solution:** Create `.env` file with required variables (see section 3 above)

### URL Fetch Failures
```
WARNING: Failed to fetch https://...: [error details]
```
**Solution:**
- Verify URL is accessible (`curl https://...` test first)
- Check timeout value: `REQUEST_TIMEOUT_SECONDS=60`
- Verify Docusaurus site structure (uses `<main>`, `<article>`, or `<body>`)

### Cohere API Errors
```
ERROR: Failed to embed chunks after 3 attempts: [error]
```
**Solution:**
- Verify COHERE_API_KEY is correct
- Check Cohere API quota and billing at https://dashboard.cohere.com/
- Reduce batch size or increase retries

### Qdrant Connection Issues
```
ERROR: Failed to create collection: [error]
```
**Solution:**
- Verify QDRANT_URL format: `https://xxxxx.qdrant.io` (with https://)
- Verify QDRANT_API_KEY is correct
- Check network connectivity to Qdrant Cloud

### Quota Exceeded
```
ERROR: Collection vector limit reached
```
**Solution:**
- Check current vector count: `collection_info = client.get_collection("name")`
- Delete old collection: `client.delete_collection("old_name")`
- Create new collection with different name
- Monitor vector usage weekly

---

## Performance Metrics

**Typical Performance (per 20 KB chapter):**
- Fetch: 2-5 seconds
- Chunking: 0.5 seconds
- Embedding (50 chunks): 10-15 seconds
- Storage: 3-5 seconds
- **Total: ~20-30 seconds**

**Throughput:**
- ~500-1000 chunks/hour
- ~5-10 chapters/hour
- ~50-100 pages/hour

---

## Next Steps

### Phase 2: FastAPI Server
- Create FastAPI endpoints for async ingestion
- Add Neon Postgres for audit logging
- Implement batch job queue

### Phase 3: Production
- Implement distributed processing
- Add caching and deduplication
- Support multiple embedding models
- Add horizontal scaling

---

## References

- **Specification:** `../specs/2-website-ingestion/spec.md`
- **Implementation Plan:** `../specs/2-website-ingestion/plan.md`
- **Data Model:** `../specs/2-website-ingestion/data-model.md`
- **API Contracts:** `../specs/2-website-ingestion/contracts/openapi.yaml`
- **Cohere Docs:** https://docs.cohere.com/
- **Qdrant Docs:** https://qdrant.tech/documentation/
- **BeautifulSoup Docs:** https://www.crummy.com/software/BeautifulSoup/

---

*RAG Website Ingestion Backend - Phase 1 MVP*
