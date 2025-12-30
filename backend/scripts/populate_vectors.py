"""
Data ingestion script to populate Qdrant with textbook chapters.
Reads markdown files, chunks them, generates embeddings, and stores in vector DB.
"""

import os
import sys
import glob
import json
import asyncio
from pathlib import Path
from typing import List, Dict, Any
import logging

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dotenv import load_dotenv
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import uuid

# Load environment
load_dotenv()

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
OPENAI_EMBED_MODEL = os.getenv("OPENAI_EMBED_MODEL", "text-embedding-3-small")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION", "textbook_embeddings")

# Initialize OpenAI client
client = OpenAI(api_key=OPENAI_API_KEY)


def extract_text_from_markdown(file_path: str) -> Dict[str, Any]:
    """Extract text content from markdown file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Remove frontmatter
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                content = parts[2]

        # Extract title
        title = "Unknown"
        if '# ' in content:
            title = content.split('# ', 1)[1].split('\n')[0].strip()

        # Get chapter info from path
        path_parts = Path(file_path).parts
        module = "General"
        chapter = "Unknown"

        for i, part in enumerate(path_parts):
            if 'module' in part:
                module = part
            if 'chapter' in part:
                chapter = part

        return {
            "title": title,
            "module": module,
            "chapter": chapter,
            "file_path": file_path,
            "content": content.strip(),
            "length": len(content)
        }
    except Exception as e:
        logger.error(f"Error reading {file_path}: {e}")
        return None


def chunk_text(text: str, chunk_size: int = 512, overlap: int = 100) -> List[str]:
    """Split text into overlapping chunks."""
    chunks = []
    start = 0

    while start < len(text):
        end = min(start + chunk_size, len(text))
        chunk = text[start:end]

        if chunk.strip():
            chunks.append(chunk.strip())

        start = end - overlap
        if start < 0:
            start = 0

    return chunks


def get_embedding(text: str) -> List[float]:
    """Get embedding from OpenAI."""
    try:
        response = client.embeddings.create(
            input=text,
            model=OPENAI_EMBED_MODEL
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {e}")
        return None


def get_embedding_batch(texts: List[str]) -> List[List[float]]:
    """Get embeddings for multiple texts at once."""
    if not texts:
        return []

    try:
        response = client.embeddings.create(
            input=texts,
            model=OPENAI_EMBED_MODEL
        )
        embeddings = [item.embedding for item in sorted(response.data, key=lambda x: x.index)]
        return embeddings
    except Exception as e:
        logger.error(f"Error getting batch embeddings: {e}")
        return [None] * len(texts)


def main():
    """Main data ingestion pipeline."""

    logger.info("=" * 80)
    logger.info("RAG CHATBOT - DATA INGESTION PIPELINE")
    logger.info("=" * 80)

    # 1. Find all markdown files
    logger.info("\n[1] Scanning for markdown files...")
    docs_path = "../frontend/docs"
    md_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)
    logger.info(f"Found {len(md_files)} markdown files")

    # 2. Extract text from files
    logger.info("\n[2] Extracting text from documents...")
    documents = []
    for file_path in md_files:
        doc = extract_text_from_markdown(file_path)
        if doc and doc['length'] > 100:  # Only include substantive content
            documents.append(doc)

    logger.info(f"Extracted {len(documents)} documents")

    # 3. Chunk documents
    logger.info("\n[3] Chunking documents into searchable units...")
    chunks = []
    for doc in documents:
        doc_chunks = chunk_text(doc['content'], chunk_size=512, overlap=100)
        for i, chunk in enumerate(doc_chunks):
            chunks.append({
                "id": str(uuid.uuid4()),
                "text": chunk,
                "title": doc['title'],
                "module": doc['module'],
                "chapter": doc['chapter'],
                "chunk_index": i,
                "source_file": doc['file_path']
            })

    logger.info(f"Created {len(chunks)} chunks")

    # 4. Generate embeddings
    logger.info("\n[4] Generating embeddings (this may take a minute)...")

    # Batch process for efficiency
    batch_size = 20
    embeddings_map = {}

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i+batch_size]
        texts = [c['text'] for c in batch]
        embeddings = get_embedding_batch(texts)

        for chunk, embedding in zip(batch, embeddings):
            if embedding:
                embeddings_map[chunk['id']] = embedding

        logger.info(f"  Processed {min(i + batch_size, len(chunks))}/{len(chunks)} chunks")

    logger.info(f"Generated embeddings for {len(embeddings_map)} chunks")

    # 5. Connect to Qdrant
    logger.info("\n[5] Connecting to Qdrant Cloud...")
    try:
        qdrant = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            timeout=30
        )
        logger.info("Connected to Qdrant successfully")
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        return False

    # 6. Check if collection exists
    logger.info(f"\n[6] Checking collection '{QDRANT_COLLECTION}'...")
    try:
        collections = qdrant.get_collections()
        collection_names = [c.name for c in collections.collections]

        if QDRANT_COLLECTION in collection_names:
            logger.info(f"Collection exists. Deleting old data...")
            qdrant.delete_collection(QDRANT_COLLECTION)

        # Create collection
        logger.info(f"Creating collection '{QDRANT_COLLECTION}'...")
        qdrant.create_collection(
            collection_name=QDRANT_COLLECTION,
            vectors_config=VectorParams(
                size=1536,  # OpenAI text-embedding-3-small dimension
                distance=Distance.COSINE
            )
        )
        logger.info(f"Collection created successfully")
    except Exception as e:
        logger.error(f"Error managing collection: {e}")
        return False

    # 7. Upload vectors to Qdrant
    logger.info(f"\n[7] Uploading {len(embeddings_map)} vectors to Qdrant...")
    try:
        points = []
        for chunk in chunks:
            if chunk['id'] in embeddings_map:
                points.append(
                    PointStruct(
                        id=hash(chunk['id']) % (2**63),  # Convert UUID to positive int
                        vector=embeddings_map[chunk['id']],
                        payload={
                            "text": chunk['text'],
                            "title": chunk['title'],
                            "module": chunk['module'],
                            "chapter": chunk['chapter'],
                            "chunk_index": chunk['chunk_index'],
                            "source_file": chunk['source_file']
                        }
                    )
                )

        # Upload in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i+batch_size]
            qdrant.upsert(
                collection_name=QDRANT_COLLECTION,
                points=batch
            )
            logger.info(f"  Uploaded {min(i + batch_size, len(points))}/{len(points)} vectors")

        logger.info(f"Successfully uploaded all vectors to Qdrant")
    except Exception as e:
        logger.error(f"Error uploading to Qdrant: {e}")
        return False

    # 8. Verify upload
    logger.info(f"\n[8] Verifying upload...")
    try:
        collection_info = qdrant.get_collection(QDRANT_COLLECTION)
        vector_count = collection_info.points_count
        logger.info(f"Collection now contains {vector_count} vectors")
    except Exception as e:
        logger.error(f"Error verifying: {e}")
        return False

    # 9. Test search
    logger.info(f"\n[9] Testing vector search...")
    try:
        test_query = "What is ROS 2?"
        test_embedding = get_embedding(test_query)

        if test_embedding:
            results = qdrant.search(
                collection_name=QDRANT_COLLECTION,
                query_vector=test_embedding,
                limit=3
            )

            logger.info(f"Search results for '{test_query}':")
            for i, result in enumerate(results, 1):
                logger.info(f"  {i}. Score: {result.score:.4f}")
                logger.info(f"     Title: {result.payload['title']}")
                logger.info(f"     Text: {result.payload['text'][:100]}...")
    except Exception as e:
        logger.error(f"Error testing search: {e}")
        return False

    logger.info("\n" + "=" * 80)
    logger.info("DATA INGESTION COMPLETED SUCCESSFULLY!")
    logger.info("=" * 80)
    logger.info(f"\nSummary:")
    logger.info(f"  - Documents processed: {len(documents)}")
    logger.info(f"  - Chunks created: {len(chunks)}")
    logger.info(f"  - Vectors uploaded: {vector_count}")
    logger.info(f"  - Collection: {QDRANT_COLLECTION}")
    logger.info(f"\nYour RAG chatbot is now ready to answer questions about the textbook!")

    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
