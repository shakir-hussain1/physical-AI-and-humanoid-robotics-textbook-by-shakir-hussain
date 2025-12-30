"""Retrieval tool for OpenAI Agents SDK."""

from typing import List, Dict, Optional
import json


class RetrievalTool:
    """Wrapper for RetrievalService as OpenAI tool."""

    def __init__(self):
        """Initialize retrieval tool."""
        # Import here to avoid circular dependencies
        try:
            from ..retrieval.retrieval_service import RetrievalService
            from ..retrieval.config import RetrievalConfig

            config = RetrievalConfig()
            self.retrieval_service = RetrievalService(config)
        except Exception as e:
            # If retrieval service can't be initialized, set to None
            import logging
            logger = logging.getLogger(__name__)
            logger.warning(f"RetrievalTool initialization warning: {str(e)}")
            self.retrieval_service = None

    def get_tool_definition(self) -> Dict:
        """Get OpenAI tool definition.

        Returns:
            Tool definition for OpenAI Agents SDK.
        """
        return {
            "type": "function",
            "function": {
                "name": "search_knowledge_base",
                "description": "Search the textbook knowledge base for relevant content",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "Search query to find relevant content",
                        },
                        "k": {
                            "type": "integer",
                            "description": "Number of results to return (default 5)",
                            "default": 5,
                        },
                    },
                    "required": ["query"],
                },
            },
        }

    def search_knowledge_base(self, query: str, k: int = 5) -> List[Dict]:
        """Search knowledge base for relevant chunks.

        Args:
            query: Search query.
            k: Number of results to return.

        Returns:
            List of retrieved chunks with metadata.
        """
        if not self.retrieval_service:
            return {
                "error": "retrieval_unavailable",
                "details": "Retrieval service not initialized",
                "results": [],
            }

        try:
            results = self.retrieval_service.search(query, k=k)
            # Extract results from response if it has the 'results' key
            if isinstance(results, dict) and "results" in results:
                return results.get("results", [])
            return results if isinstance(results, list) else []
        except Exception as e:
            return {
                "error": "search_failed",
                "details": str(e),
                "results": [],
            }
