"""Construct grounded context from retrieved chunks."""

from typing import List, Dict
from .types import RetrievedChunk


class ContextConstructor:
    """Build context strings from retrieved chunks with metadata."""

    def __init__(self, max_tokens: int = 2000):
        """Initialize context constructor.

        Args:
            max_tokens: Maximum tokens for context (default 2000).
        """
        self.max_tokens = max_tokens

    def construct_context(
        self, retrieved_chunks: List[Dict], query: str
    ) -> str:
        """Construct context string from retrieved chunks.

        Args:
            retrieved_chunks: List of chunk dicts from RetrievalService.
            query: Original user query for context.

        Returns:
            Formatted context string with chunk metadata.
        """
        if not retrieved_chunks:
            return "No relevant content found in knowledge base."

        context_parts = ["RETRIEVED CONTEXT:"]

        for i, chunk in enumerate(retrieved_chunks, 1):
            # Extract metadata
            metadata = chunk.get("metadata", {})
            url = metadata.get("url", "unknown")
            title = metadata.get("page_title", "unknown")
            score = chunk.get("similarity_score", 0)

            # Add chunk with metadata
            chunk_text = f"""
Chunk {i} (Relevance: {score:.2f})
Source: {title}
URL: {url}
Content: {chunk.get('text', 'N/A')}"""
            context_parts.append(chunk_text)

        return "\n".join(context_parts)

    def truncate_context(self, context: str) -> str:
        """Truncate context to token budget.

        Args:
            context: Full context string.

        Returns:
            Truncated context if needed.
        """
        # Simple token estimation: 1 token ~= 4 characters
        estimated_tokens = len(context) // 4

        if estimated_tokens > self.max_tokens:
            # Calculate truncation point (75% of content)
            truncate_chars = int(len(context) * 0.75)
            return context[:truncate_chars] + "\n[Context truncated...]"

        return context

    def format_context_with_sources(
        self, retrieved_chunks: List[Dict]
    ) -> tuple[str, List[Dict]]:
        """Format context and extract source information.

        Args:
            retrieved_chunks: List of chunk dicts.

        Returns:
            Tuple of (context_string, sources_list).
        """
        context = self.construct_context(retrieved_chunks, "")
        sources = []

        for chunk in retrieved_chunks[:3]:  # Top-3 sources
            metadata = chunk.get("metadata", {})
            sources.append({
                "url": metadata.get("url", "unknown"),
                "page_title": metadata.get("page_title", "unknown"),
                "relevance_score": chunk.get("similarity_score", 0),
                "chunk_index": metadata.get("chunk_index", 0),
            })

        return context, sources
