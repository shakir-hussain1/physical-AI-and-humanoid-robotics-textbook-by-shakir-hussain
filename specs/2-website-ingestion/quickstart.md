# Website URL Ingestion & Vector Storage - Quick Start Guide

## Overview
This guide walks through setting up and using the RAG document ingestion pipeline to crawl Docusaurus URLs, generate embeddings, and store them in Qdrant Cloud.

## Prerequisites

### Required Accounts & Services
1. **Cohere API Key**
   - Sign up at https://cohere.com/
   - Create API key with access to embedding models
   - Keep key secure in environment variable

2. **Qdrant Cloud Account**
   - Sign up at https://cloud.qdrant.io/
   - Create a Free Tier cluster
   - Note API endpoint and API key

3. **Python Environment**
   - Python 3.10 or higher
   - UV package manager (https://docs.astral.sh/uv/)

### System Requirements
- Minimum 512MB RAM (more for large batch ingestion)
- Internet access to Cohere API and Qdrant Cloud
- 5-10GB free disk for logs and caches

---

## Setup & Installation

### 1. Initialize Backend Project

```bash
# Navigate to backend directory
cd backend

# Initialize Python project with UV
uv init --python 3.10

# Create virtual environment
uv venv

# Activate virtual environment
# On Linux/macOS:
source .venv/bin/activate
# On Windows:
.venv\Scripts\activate
```

### 2. Install Dependencies

Create `pyproject.toml`:
```toml
[project]
name = "rag-ingestion"
version = "0.1.0"
description = "RAG document ingestion pipeline"
requires-python = ">=3.10"
dependencies = [
    "fastapi>=0.104.0",
    "uvicorn[standard]>=0.24.0",
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "cohere>=4.37.0",
    "qdrant-client>=2.7.0",
    "tiktoken>=0.5.0",
    "python-dotenv>=1.0.0",
    "pydantic>=2.5.0",
    "pydantic-settings>=2.1.0",
]

[tool.uv.sources]
```

Install dependencies:
```bash
uv pip install -e .
# or
uv sync
```

### 3. Environment Configuration

Create `.env` file in backend directory:
```bash
# Cohere API
COHERE_API_KEY=your_cohere_api_key_here
COHERE_EMBEDDING_MODEL=embed-english-v3.0

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Application
APP_ENV=development
LOG_LEVEL=INFO
MAX_RETRIES=3
REQUEST_TIMEOUT_SECONDS=30
```

Create `.env.example` for version control (no secrets):
```bash
# Cohere API
COHERE_API_KEY=
COHERE_EMBEDDING_MODEL=embed-english-v3.0

# Qdrant Cloud
QDRANT_URL=
QDRANT_API_KEY=

# Application
APP_ENV=development
LOG_LEVEL=INFO
```

---

## Project Structure

```
backend/
├── main.py                    # Entry point with main() function
├── .env                       # Environment variables (gitignored)
├── .env.example               # Example env file (version control)
├── pyproject.toml             # Dependencies
└── logs/
    └── ingestion.log          # Pipeline logs
```

---

## Usage

### Basic Ingestion (main.py)

Create `main.py`:

```python
#!/usr/bin/env python3
"""
RAG Website Ingestion & Vector Storage Pipeline
Orchestrates: URL crawling → chunking → embedding → Qdrant storage
"""

import asyncio
import logging
import os
from datetime import datetime
from typing import List
from uuid import uuid4

import requests
from bs4 import BeautifulSoup
from cohere import Cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import tiktoken

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO"),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("logs/ingestion.log"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)


class IngestionConfig:
    """Configuration for the ingestion pipeline."""
    cohere_api_key: str = os.getenv("COHERE_API_KEY")
    cohere_model: str = os.getenv("COHERE_EMBEDDING_MODEL", "embed-english-v3.0")
    qdrant_url: str = os.getenv("QDRANT_URL")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY")
    max_chunk_tokens: int = 1024
    max_retries: int = int(os.getenv("MAX_RETRIES", "3"))
    timeout_seconds: int = int(os.getenv("REQUEST_TIMEOUT_SECONDS", "30"))

    def validate(self) -> bool:
        """Validate all required configuration is set."""
        required = [
            self.cohere_api_key,
            self.qdrant_url,
            self.qdrant_api_key,
        ]
        if not all(required):
            logger.error("Missing required environment variables")
            return False
        return True


class WebCrawler:
    """Fetch and extract text content from Docusaurus URLs."""

    def __init__(self, timeout: int = 30):
        self.timeout = timeout
        self.session = requests.Session()

    def fetch_url(self, url: str) -> tuple[str, str] | tuple[None, None]:
        """
        Fetch and extract text content from a URL.

        Returns:
            Tuple of (extracted_text, page_title) or (None, None) on failure
        """
        try:
            logger.info(f"Fetching URL: {url}")
            response = self.session.get(url, timeout=self.timeout)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, "html.parser")

            # Extract title
            title_tag = soup.find("title")
            page_title = title_tag.get_text(strip=True) if title_tag else "Untitled"

            # Remove script and style tags
            for tag in soup(["script", "style"]):
                tag.decompose()

            # Extract main content (Docusaurus uses main or article tags)
            main_content = soup.find("main") or soup.find("article") or soup.find("body")
            if not main_content:
                logger.warning(f"No main content found in {url}")
                return None, None

            text_content = main_content.get_text(separator="\n", strip=True)

            logger.info(f"Successfully extracted {len(text_content)} chars from {url}")
            return text_content, page_title

        except requests.RequestException as e:
            logger.error(f"Failed to fetch {url}: {e}")
            return None, None
        except Exception as e:
            logger.error(f"Unexpected error processing {url}: {e}")
            return None, None


class SemanticChunker:
    """Split text into semantic chunks while preserving context."""

    def __init__(self, max_tokens: int = 1024):
        self.max_tokens = max_tokens
        self.tokenizer = tiktoken.get_encoding("cl100k_base")

    def chunk_text(self, text: str, url: str, page_title: str) -> List[dict]:
        """
        Split text into semantic chunks.

        Returns:
            List of chunks with metadata
        """
        chunks = []

        # Split on paragraph boundaries first
        paragraphs = text.split("\n\n")
        current_chunk = ""
        chunk_index = 0

        for paragraph in paragraphs:
            # Count tokens
            chunk_candidate = current_chunk + "\n\n" + paragraph if current_chunk else paragraph
            token_count = len(self.tokenizer.encode(chunk_candidate))

            if token_count <= self.max_tokens:
                current_chunk = chunk_candidate
            else:
                # Save current chunk if it has content
                if current_chunk:
                    tokens = len(self.tokenizer.encode(current_chunk))
                    chunks.append({
                        "text": current_chunk.strip(),
                        "tokens": tokens,
                        "chunk_index": chunk_index,
                        "metadata": {
                            "url": url,
                            "page_title": page_title,
                        }
                    })
                    chunk_index += 1

                # Start new chunk with current paragraph
                current_chunk = paragraph

        # Add final chunk
        if current_chunk:
            tokens = len(self.tokenizer.encode(current_chunk))
            chunks.append({
                "text": current_chunk.strip(),
                "tokens": tokens,
                "chunk_index": chunk_index,
                "metadata": {
                    "url": url,
                    "page_title": page_title,
                }
            })

        logger.info(f"Created {len(chunks)} chunks from {url}")
        return chunks


class EmbeddingService:
    """Generate embeddings using Cohere API."""

    def __init__(self, api_key: str, model: str, max_retries: int = 3):
        self.client = Cohere(api_key=api_key)
        self.model = model
        self.max_retries = max_retries

    def embed_chunks(self, chunks: List[str]) -> List[List[float]] | None:
        """
        Generate embeddings for a batch of chunks.

        Args:
            chunks: List of text chunks (max 100)

        Returns:
            List of embedding vectors or None on failure
        """
        if not chunks:
            return []

        for attempt in range(self.max_retries):
            try:
                logger.info(f"Embedding {len(chunks)} chunks (attempt {attempt + 1})")
                response = self.client.embed(
                    model=self.model,
                    texts=chunks,
                    input_type="search_document",
                )
                embeddings = response.embeddings
                logger.info(f"Successfully embedded {len(chunks)} chunks")
                return embeddings

            except Exception as e:
                logger.warning(f"Embedding attempt {attempt + 1} failed: {e}")
                if attempt < self.max_retries - 1:
                    import time
                    wait_time = 2 ** attempt  # Exponential backoff: 1, 2, 4 seconds
                    logger.info(f"Retrying in {wait_time}s...")
                    time.sleep(wait_time)
                else:
                    logger.error(f"Failed to embed chunks after {self.max_retries} attempts")
                    return None

        return None


class QdrantStorage:
    """Store embeddings in Qdrant Cloud."""

    def __init__(self, url: str, api_key: str):
        self.client = QdrantClient(url=url, api_key=api_key)

    def create_collection(self, collection_name: str, vector_size: int = 1024):
        """Create a new Qdrant collection if it doesn't exist."""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            if any(c.name == collection_name for c in collections.collections):
                logger.info(f"Collection '{collection_name}' already exists")
                return True

            logger.info(f"Creating collection '{collection_name}'")
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
            )
            logger.info(f"Collection '{collection_name}' created successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to create collection: {e}")
            return False

    def store_vectors(
        self, collection_name: str, embeddings: List[List[float]], metadata: List[dict]
    ) -> bool:
        """Store embeddings and metadata in Qdrant."""
        try:
            if not embeddings or not metadata:
                logger.warning("No embeddings or metadata to store")
                return True

            points = []
            for i, (embedding, meta) in enumerate(zip(embeddings, metadata)):
                point_id = str(uuid4())
                points.append(
                    PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload=meta,
                    )
                )

            logger.info(f"Storing {len(points)} vectors in '{collection_name}'")
            self.client.upsert(
                collection_name=collection_name,
                points=points,
            )
            logger.info(f"Successfully stored {len(points)} vectors")
            return True

        except Exception as e:
            logger.error(f"Failed to store vectors: {e}")
            return False

    def verify_vectors(self, collection_name: str, sample_size: int = 10) -> bool:
        """Verify stored vectors by querying a sample."""
        try:
            collection_info = self.client.get_collection(collection_name)
            vector_count = collection_info.points_count

            if vector_count == 0:
                logger.warning(f"Collection '{collection_name}' is empty")
                return False

            logger.info(f"Collection has {vector_count} vectors. Verifying sample of {min(sample_size, vector_count)}")

            # Get a sample of points
            points = self.client.scroll(collection_name=collection_name, limit=sample_size)[0]

            if not points:
                logger.error("Failed to retrieve sample points")
                return False

            verified_count = 0
            for point in points:
                if point.id and point.vector and point.payload:
                    verified_count += 1

            logger.info(f"Verification passed: {verified_count}/{len(points)} points valid")
            return verified_count > 0

        except Exception as e:
            logger.error(f"Verification failed: {e}")
            return False


def main():
    """Main ingestion pipeline."""
    logger.info("=" * 60)
    logger.info("RAG Website Ingestion Pipeline Started")
    logger.info("=" * 60)

    # Load and validate configuration
    config = IngestionConfig()
    if not config.validate():
        logger.error("Configuration validation failed")
        return 1

    # Example URLs (Docusaurus book)
    urls = [
        "https://docs.example.com/intro",
        "https://docs.example.com/module-1/chapter-1",
        # Add more URLs as needed
    ]
    collection_name = "book_embeddings"

    # Step 1: Initialize services
    logger.info("Initializing services...")
    crawler = WebCrawler(timeout=config.timeout_seconds)
    chunker = SemanticChunker(max_tokens=config.max_chunk_tokens)
    embedder = EmbeddingService(config.cohere_api_key, config.cohere_model, config.max_retries)
    storage = QdrantStorage(config.qdrant_url, config.qdrant_api_key)

    # Step 2: Create Qdrant collection
    if not storage.create_collection(collection_name):
        logger.error("Failed to create Qdrant collection")
        return 1

    # Step 3: Process URLs
    total_chunks = 0
    total_embeddings = 0

    for url in urls:
        logger.info(f"\nProcessing URL: {url}")

        # Fetch content
        text_content, page_title = crawler.fetch_url(url)
        if not text_content:
            logger.warning(f"Skipping {url} due to fetch failure")
            continue

        # Chunk content
        chunks = chunker.chunk_text(text_content, url, page_title)
        if not chunks:
            logger.warning(f"No chunks created for {url}")
            continue

        total_chunks += len(chunks)

        # Extract chunk texts and metadata
        chunk_texts = [c["text"] for c in chunks]
        chunk_metadata = [c["metadata"] for c in chunks]

        # Generate embeddings
        embeddings = embedder.embed_chunks(chunk_texts)
        if not embeddings:
            logger.warning(f"Failed to embed chunks from {url}")
            continue

        total_embeddings += len(embeddings)

        # Store in Qdrant
        if not storage.store_vectors(collection_name, embeddings, chunk_metadata):
            logger.warning(f"Failed to store vectors from {url}")
            continue

    # Step 4: Verify storage
    logger.info("\nVerifying vector storage...")
    if storage.verify_vectors(collection_name, sample_size=10):
        logger.info("✓ Vector storage verification passed")
    else:
        logger.error("✗ Vector storage verification failed")
        return 1

    # Summary
    logger.info("\n" + "=" * 60)
    logger.info(f"Ingestion Complete")
    logger.info(f"  Total URLs processed: {len(urls)}")
    logger.info(f"  Total chunks created: {total_chunks}")
    logger.info(f"  Total embeddings generated: {total_embeddings}")
    logger.info(f"  Collection: {collection_name}")
    logger.info("=" * 60)

    return 0


if __name__ == "__main__":
    exit_code = main()
    exit(exit_code)
```

### 4. Create Logs Directory

```bash
mkdir -p logs
```

### 5. Run the Pipeline

```bash
# Activate virtual environment (if not already active)
source .venv/bin/activate  # Linux/macOS
# or
.venv\Scripts\activate  # Windows

# Run the ingestion pipeline
python main.py
```

---

## Expected Output

```
============================================================
RAG Website Ingestion Pipeline Started
============================================================
2025-12-25 10:00:00,000 - __main__ - INFO - Initializing services...
2025-12-25 10:00:01,000 - __main__ - INFO - Creating collection 'book_embeddings'
2025-12-25 10:00:02,000 - __main__ - INFO - Processing URL: https://docs.example.com/intro
2025-12-25 10:00:03,000 - __main__ - INFO - Fetching URL: https://docs.example.com/intro
2025-12-25 10:00:04,000 - __main__ - INFO - Successfully extracted 5432 chars from URL
2025-12-25 10:00:05,000 - __main__ - INFO - Created 6 chunks from URL
2025-12-25 10:00:06,000 - __main__ - INFO - Embedding 6 chunks (attempt 1)
2025-12-25 10:00:08,000 - __main__ - INFO - Successfully embedded 6 chunks
2025-12-25 10:00:09,000 - __main__ - INFO - Storing 6 vectors in 'book_embeddings'
2025-12-25 10:00:10,000 - __main__ - INFO - Successfully stored 6 vectors
...
2025-12-25 10:00:30,000 - __main__ - INFO - Verifying vector storage...
2025-12-25 10:00:31,000 - __main__ - INFO - ✓ Vector storage verification passed
============================================================
Ingestion Complete
  Total URLs processed: 2
  Total chunks created: 12
  Total embeddings generated: 12
  Collection: book_embeddings
============================================================
```

---

## Customization

### Change Input URLs
Edit the `urls` list in `main.py`:
```python
urls = [
    "https://your-book.com/page1",
    "https://your-book.com/page2",
    "https://your-book.com/module/chapter",
]
```

### Adjust Chunk Size
Modify `max_chunk_tokens` in `IngestionConfig`:
```python
max_chunk_tokens: int = 512  # Smaller chunks
```

### Change Embedding Model
Update `.env`:
```bash
COHERE_EMBEDDING_MODEL=embed-english-light-v3.0  # Lightweight model
```

### Enable FastAPI Server (Future)
The `main.py` can be extended to include FastAPI endpoints for async ingestion.

---

## Troubleshooting

### Missing API Keys
```
Error: Missing required environment variables
→ Check .env file has COHERE_API_KEY and QDRANT_API_KEY
```

### URL Fetch Failures
```
Error: Failed to fetch https://...
→ Check URL is accessible (curl test first)
→ Verify timeout is sufficient (increase REQUEST_TIMEOUT_SECONDS)
```

### Embedding API Errors
```
Error: Embedding attempt 3 failed
→ Check Cohere API quota and billing
→ Verify COHERE_API_KEY is correct
```

### Qdrant Connection Issues
```
Error: Failed to create collection
→ Verify QDRANT_URL and QDRANT_API_KEY
→ Check network connectivity to Qdrant Cloud
```

### Quota Exceeded
```
Error: Collection vector limit reached
→ Delete old collection or create new one
→ Monitor vector_count in QdrantCollection
```

---

## Next Steps

1. **Add More URLs** to the ingestion list and test
2. **Monitor Logs** in `logs/ingestion.log` for issues
3. **Verify Vectors** using the verification endpoint
4. **Set Up FastAPI Server** for async batch ingestion (Phase 2)
5. **Implement Database** for persistent tracking (Phase 2)

---

*Quick start guide for RAG Website Ingestion & Vector Storage pipeline.*
