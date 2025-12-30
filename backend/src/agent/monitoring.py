"""Agent monitoring and performance tracking."""

import logging
import json
from typing import Dict, Any
from datetime import datetime

logger = logging.getLogger(__name__)


class AgentMonitor:
    """Monitor agent performance and quality metrics."""

    def __init__(self):
        """Initialize agent monitor."""
        self.metrics = {
            "total_queries": 0,
            "grounded_answers": 0,
            "hallucinated_answers": 0,
            "latencies": [],
            "confidence_distribution": {"high": 0, "medium": 0, "low": 0},
        }

    def track_query(self, query: str, latency_ms: float, confidence: str):
        """Track a query execution.

        Args:
            query: User query.
            latency_ms: Query latency in milliseconds.
            confidence: Confidence level.
        """
        self.metrics["total_queries"] += 1
        self.metrics["latencies"].append(latency_ms)
        self.metrics["confidence_distribution"][confidence] += 1

        # Log structured
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "operation": "query",
            "query_length": len(query),
            "latency_ms": latency_ms,
            "confidence": confidence,
            "total_queries": self.metrics["total_queries"],
        }

        logger.info(json.dumps(log_entry))

    def track_grounding(self, is_grounded: bool):
        """Track grounding validation result.

        Args:
            is_grounded: Whether answer was grounded.
        """
        if is_grounded:
            self.metrics["grounded_answers"] += 1
        else:
            self.metrics["hallucinated_answers"] += 1

    def get_metrics(self) -> Dict[str, Any]:
        """Get current metrics.

        Returns:
            Metrics dictionary.
        """
        if self.metrics["latencies"]:
            latencies = sorted(self.metrics["latencies"])
            p95_idx = int(len(latencies) * 0.95)
            p99_idx = int(len(latencies) * 0.99)

            return {
                **self.metrics,
                "avg_latency_ms": sum(self.metrics["latencies"])
                / len(self.metrics["latencies"]),
                "p95_latency_ms": latencies[p95_idx] if p95_idx < len(latencies) else latencies[-1],
                "p99_latency_ms": latencies[p99_idx] if p99_idx < len(latencies) else latencies[-1],
                "hallucination_rate": self.metrics["hallucinated_answers"]
                / self.metrics["total_queries"]
                if self.metrics["total_queries"] > 0
                else 0,
                "grounding_rate": self.metrics["grounded_answers"]
                / self.metrics["total_queries"]
                if self.metrics["total_queries"] > 0
                else 0,
            }
        return self.metrics

    def reset(self):
        """Reset metrics."""
        self.metrics = {
            "total_queries": 0,
            "grounded_answers": 0,
            "hallucinated_answers": 0,
            "latencies": [],
            "confidence_distribution": {"high": 0, "medium": 0, "low": 0},
        }
