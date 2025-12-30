"""Query intent parsing and classification."""

from typing import Optional, Dict
from .types import Intent


class IntentParser:
    """Parse user queries to extract intent, topic, and scope."""

    def __init__(self):
        """Initialize intent parser with keyword mappings."""
        self._intent_cache: Dict[str, Intent] = {}

        # Keywords for out-of-domain detection
        self.out_of_domain_keywords = {
            "weather", "joke", "recipe", "song", "movie", "game",
            "stock market", "bitcoin", "sports score", "current time",
            "tell me a story", "help with homework", "dating advice"
        }

    def parse_intent(self, query: str) -> Intent:
        """Parse query to extract intent type, topic, and scope.

        Args:
            query: User query text.

        Returns:
            Intent object with query_type, primary_topic, scope.
        """
        # Check cache
        if query in self._intent_cache:
            return self._intent_cache[query]

        # Check for out-of-domain
        query_lower = query.lower()
        is_out_of_domain = self.detect_out_of_domain(query_lower)

        if is_out_of_domain:
            intent = Intent(
                query_text=query,
                query_type="out_of_scope",
                primary_topic=None,
                scope=None,
                confidence=0.9,
            )
        else:
            # Classify query type
            query_type = self._classify_query_type(query_lower)

            # Extract primary topic
            primary_topic = self.extract_topic(query_lower)

            intent = Intent(
                query_text=query,
                query_type=query_type,
                primary_topic=primary_topic,
                scope=None,
                confidence=0.8,
            )

        # Cache result
        self._intent_cache[query] = intent
        return intent

    def _classify_query_type(self, query_lower: str) -> str:
        """Classify query type based on patterns.

        Args:
            query_lower: Lowercase query text.

        Returns:
            Query type: factual, conceptual, how_to, clarification.
        """
        # How-to questions
        if query_lower.startswith(("how", "how to", "how do", "how can")):
            return "how_to"

        # Clarification questions
        if query_lower.startswith(("what", "which", "can you explain")):
            return "clarification"

        # Conceptual questions
        if query_lower.startswith(("why", "what is", "explain")):
            return "conceptual"

        # Default to factual
        return "factual"

    def extract_topic(self, query_lower: str) -> Optional[str]:
        """Extract primary topic from query.

        Args:
            query_lower: Lowercase query text.

        Returns:
            Primary topic or None.
        """
        # Keywords for textbook domains
        domain_keywords = {
            "ros": ["ros", "ros2", "middleware", "node", "topic", "service"],
            "simulation": ["simulation", "gazebo", "simulator", "digital twin"],
            "perception": ["perception", "camera", "lidar", "sensor"],
            "planning": ["planning", "path", "motion", "behavior"],
            "learning": ["learning", "neural", "network", "training"],
            "control": ["control", "controller", "feedback", "pid"],
        }

        for topic, keywords in domain_keywords.items():
            if any(kw in query_lower for kw in keywords):
                return topic

        return None

    def detect_out_of_domain(self, query_lower: str) -> bool:
        """Detect if query is outside textbook domain.

        Args:
            query_lower: Lowercase query text.

        Returns:
            True if out-of-domain, False otherwise.
        """
        return any(keyword in query_lower for keyword in self.out_of_domain_keywords)

    def clear_cache(self):
        """Clear intent cache."""
        self._intent_cache.clear()
