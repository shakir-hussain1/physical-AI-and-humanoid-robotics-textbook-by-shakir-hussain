"""Format agent responses with sources and confidence."""

from typing import List, Dict
from .types import AgentResponse, SourceInfo


class ResponseFormatter:
    """Format responses with sources, confidence, and metadata."""

    def __init__(self):
        """Initialize response formatter."""
        pass

    def format_response(
        self,
        answer: str,
        sources: List[Dict],
        confidence: str = "medium",
        metadata: Dict = None,
    ) -> AgentResponse:
        """Format response with answer, sources, and metadata.

        Args:
            answer: Generated answer text.
            sources: List of source dicts with url, page_title, relevance_score, chunk_index.
            confidence: Confidence level (high, medium, low).
            metadata: Optional metadata dict.

        Returns:
            AgentResponse object.
        """
        # Truncate long answers
        if len(answer) > 2000:
            answer = answer[:2000] + "\n[Answer truncated. Ask for continuation.]"

        # Convert sources to SourceInfo objects
        source_objects = []
        for src in sources[:3]:  # Top-3 sources
            source_objects.append(
                SourceInfo(
                    url=src.get("url", "unknown"),
                    page_title=src.get("page_title", "unknown"),
                    relevance_score=src.get("relevance_score", 0.0),
                    chunk_index=src.get("chunk_index", 0),
                )
            )

        # Build metadata
        if metadata is None:
            metadata = {}

        # Add reasoning
        if source_objects:
            source_refs = [f"Chunk {s.chunk_index}" for s in source_objects]
            reasoning = f"Based on {', '.join(source_refs)}"
            metadata["reasoning"] = reasoning

        # Create response
        response = AgentResponse(
            answer=answer,
            sources=source_objects,
            confidence=confidence,
            metadata=metadata,
        )

        return response

    def add_follow_up_suggestions(
        self, response: AgentResponse, related_chunks: List[str]
    ) -> AgentResponse:
        """Add follow-up suggestions to response.

        Args:
            response: Original AgentResponse.
            related_chunks: List of related chunk titles.

        Returns:
            Updated AgentResponse with follow-ups.
        """
        follow_ups = [
            f"Tell me more about {chunk}" for chunk in related_chunks[:3]
        ]
        response.metadata["follow_up_suggestions"] = follow_ups
        return response
