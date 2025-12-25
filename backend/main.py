#!/usr/bin/env python3
"""
RAG Website Ingestion & Vector Storage Pipeline

Orchestrates: URL crawling → chunking → embedding → Qdrant storage

This module implements a complete ingestion pipeline for Docusaurus websites:
1. WebCrawler: Fetch and extract text content from URLs
2. SemanticChunker: Split text into semantic chunks (max 1024 tokens)
3. EmbeddingService: Generate embeddings using Cohere API with retries
4. QdrantStorage: Persist embeddings in Qdrant Cloud with verification
5. Main orchestration: Coordinate all components and log results
"""

import asyncio
import logging
import os
import sys
from datetime import datetime
from typing import List, Optional, Tuple
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
    """Configuration for the ingestion pipeline (T003)."""

    def __init__(self) -> None:
        """Initialize configuration from environment variables."""
        self.cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
        self.cohere_model: str = os.getenv("COHERE_EMBEDDING_MODEL", "embed-english-v3.0")
        self.qdrant_url: str = os.getenv("QDRANT_URL", "")
        self.qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
        self.max_chunk_tokens: int = 1024
        self.max_retries: int = int(os.getenv("MAX_RETRIES", "3"))
        self.timeout_seconds: int = int(os.getenv("REQUEST_TIMEOUT_SECONDS", "30"))

    def validate(self) -> bool:
        """Validate all required configuration is set."""
        required = [
            ("COHERE_API_KEY", self.cohere_api_key),
            ("QDRANT_URL", self.qdrant_url),
            ("QDRANT_API_KEY", self.qdrant_api_key),
        ]

        missing = [name for name, value in required if not value]

        if missing:
            logger.error(f"Missing required environment variables: {', '.join(missing)}")
            return False

        logger.info("Configuration validation successful")
        return True


class WebCrawler:
    """Fetch and extract text content from Docusaurus URLs (T004)."""

    def __init__(self, timeout: int = 30) -> None:
        """Initialize crawler with timeout."""
        self.timeout = timeout
        self.session = requests.Session()

    def fetch_url(self, url: str) -> Tuple[Optional[str], Optional[str]]:
        """
        Fetch and extract text content from a URL.

        Args:
            url: URL to fetch

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

            if not text_content or len(text_content) < 50:
                logger.warning(f"Extracted content too short ({len(text_content)} chars) from {url}")
                return None, None

            logger.info(f"Successfully extracted {len(text_content)} chars from {url}")
            return text_content, page_title

        except requests.exceptions.RequestException as e:
            logger.warning(f"Failed to fetch {url}: {e}")
            return None, None
        except Exception as e:
            logger.error(f"Unexpected error processing {url}: {e}")
            return None, None


class SemanticChunker:
    """Split text into semantic chunks while preserving context (T005)."""

    def __init__(self, max_tokens: int = 1024) -> None:
        """Initialize chunker with max tokens."""
        self.max_tokens = max_tokens
        self.tokenizer = tiktoken.get_encoding("cl100k_base")

    def chunk_text(
        self, text: str, url: str, page_title: str
    ) -> List[dict]:
        """
        Split text into semantic chunks.

        Args:
            text: Text to chunk
            url: Source URL for metadata
            page_title: Page title for metadata

        Returns:
            List of chunk dicts with text, tokens, chunk_index, metadata
        """
        chunks = []

        # Split on paragraph boundaries first
        paragraphs = text.split("\n\n")
        current_chunk = ""
        chunk_index = 0

        for paragraph in paragraphs:
            # Count tokens
            chunk_candidate = (
                current_chunk + "\n\n" + paragraph if current_chunk else paragraph
            )
            token_count = len(self.tokenizer.encode(chunk_candidate))

            if token_count <= self.max_tokens:
                current_chunk = chunk_candidate
            else:
                # Save current chunk if it has content
                if current_chunk:
                    tokens = len(self.tokenizer.encode(current_chunk))
                    chunks.append(
                        {
                            "text": current_chunk.strip(),
                            "tokens": tokens,
                            "chunk_index": chunk_index,
                            "metadata": {
                                "url": url,
                                "page_title": page_title,
                            },
                        }
                    )
                    chunk_index += 1

                # Start new chunk with current paragraph
                current_chunk = paragraph

        # Add final chunk
        if current_chunk:
            tokens = len(self.tokenizer.encode(current_chunk))
            chunks.append(
                {
                    "text": current_chunk.strip(),
                    "tokens": tokens,
                    "chunk_index": chunk_index,
                    "metadata": {
                        "url": url,
                        "page_title": page_title,
                    },
                }
            )

        logger.info(f"Created {len(chunks)} chunks from {url}")
        return chunks


class EmbeddingService:
    """Generate embeddings using Cohere API (T006)."""

    def __init__(self, api_key: str, model: str, max_retries: int = 3) -> None:
        """Initialize with Cohere API credentials."""
        self.client = Cohere(api_key=api_key)
        self.model = model
        self.max_retries = max_retries

    def embed_chunks(self, chunks: List[str]) -> Optional[List[List[float]]]:
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
                    logger.error(
                        f"Failed to embed chunks after {self.max_retries} attempts"
                    )
                    return None

        return None


class QdrantStorage:
    """Store embeddings in Qdrant Cloud (T007)."""

    def __init__(self, url: str, api_key: str) -> None:
        """Initialize with Qdrant credentials."""
        self.client = QdrantClient(url=url, api_key=api_key)

    def create_collection(self, collection_name: str, vector_size: int = 1024) -> bool:
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
        self,
        collection_name: str,
        embeddings: List[List[float]],
        metadata: List[dict],
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

            logger.info(
                f"Collection has {vector_count} vectors. Verifying sample of {min(sample_size, vector_count)}"
            )

            # Get a sample of points
            points, _ = self.client.scroll(
                collection_name=collection_name, limit=sample_size
            )

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


def main() -> int:
    """
    Main ingestion pipeline (T008).

    Orchestrates: fetch → chunk → embed → store for multiple URLs.

    Returns:
        0 on success, 1 on failure
    """
    logger.info("=" * 60)
    logger.info("RAG Website Ingestion Pipeline Started")
    logger.info("=" * 60)

    # Load and validate configuration (T003)
    config = IngestionConfig()
    if not config.validate():
        logger.error("Configuration validation failed")
        return 1

    # Example URLs (Docusaurus book) - EDIT THIS LIST
    # Testing with real Docusaurus sites (T010)
    urls = [
        "https://docusaurus.io/docs/intro",
        "https://docusaurus.io/docs/installation",
        "https://docusaurus.io/docs/markdown-features",
        "https://python.readthedocs.io/en/latest/tutorial/index.html",
        "https://python.readthedocs.io/en/latest/howto/regex.html",
    ]
    collection_name = "book_embeddings"

    # Step 1: Initialize services
    logger.info("Initializing services...")
    crawler = WebCrawler(timeout=config.timeout_seconds)
    chunker = SemanticChunker(max_tokens=config.max_chunk_tokens)
    embedder = EmbeddingService(
        config.cohere_api_key, config.cohere_model, config.max_retries
    )
    storage = QdrantStorage(config.qdrant_url, config.qdrant_api_key)

    # Step 2: Create Qdrant collection
    if not storage.create_collection(collection_name):
        logger.error("Failed to create Qdrant collection")
        return 1

    # Step 3: Process URLs
    total_chunks = 0
    total_embeddings = 0
    successful_urls = 0

    for url in urls:
        logger.info(f"\nProcessing URL: {url}")

        # Fetch content (T004)
        text_content, page_title = crawler.fetch_url(url)
        if not text_content:
            logger.warning(f"Skipping {url} due to fetch failure")
            continue

        # Chunk content (T005)
        chunks = chunker.chunk_text(text_content, url, page_title)
        if not chunks:
            logger.warning(f"No chunks created for {url}")
            continue

        total_chunks += len(chunks)

        # Extract chunk texts and metadata
        chunk_texts = [c["text"] for c in chunks]
        chunk_metadata = [c["metadata"] for c in chunks]

        # Generate embeddings (T006)
        embeddings = embedder.embed_chunks(chunk_texts)
        if not embeddings:
            logger.warning(f"Failed to embed chunks from {url}")
            continue

        total_embeddings += len(embeddings)

        # Store in Qdrant (T007)
        if not storage.store_vectors(collection_name, embeddings, chunk_metadata):
            logger.warning(f"Failed to store vectors from {url}")
            continue

        successful_urls += 1

    # Step 4: Verify storage
    logger.info("\nVerifying vector storage...")
    if storage.verify_vectors(collection_name, sample_size=10):
        logger.info("✓ Vector storage verification passed")
    else:
        logger.error("✗ Vector storage verification failed")
        return 1

    # Summary
    logger.info("\n" + "=" * 60)
    logger.info("Ingestion Complete")
    logger.info(f"  Total URLs processed: {len(urls)}")
    logger.info(f"  Successful URLs: {successful_urls}")
    logger.info(f"  Total chunks created: {total_chunks}")
    logger.info(f"  Total embeddings generated: {total_embeddings}")
    logger.info(f"  Collection: {collection_name}")
    logger.info("=" * 60)

    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
