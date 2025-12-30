"""Agent configuration management."""

import os
from typing import Optional


class AgentConfig:
    """Configuration for RAG agent with validation."""

    def __init__(self):
        """Initialize configuration from environment variables."""
        # OpenAI Configuration
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        self.openai_model = os.getenv("OPENAI_MODEL", "gpt-4")

        # Agent Configuration
        self.agent_role = os.getenv("AGENT_ROLE", "assistant")
        self.max_history = int(os.getenv("AGENT_MAX_HISTORY", "10"))
        self.max_context_tokens = int(os.getenv("AGENT_MAX_CONTEXT_TOKENS", "2000"))

        # Retrieval Configuration
        self.retrieval_k = int(os.getenv("AGENT_RETRIEVAL_K", "5"))
        self.confidence_threshold = float(os.getenv("AGENT_CONFIDENCE_THRESHOLD", "0.5"))

        # Temperature for LLM
        self.temperature = float(os.getenv("AGENT_TEMPERATURE", "0.7"))

        # Validation
        self._validate()

    def _validate(self):
        """Validate configuration on initialization."""
        # OpenAI API key is optional - if not provided, system will run in demo mode
        if not self.openai_api_key:
            import logging
            logger = logging.getLogger(__name__)
            logger.warning("OPENAI_API_KEY not set - Agent will run in demo mode")

        if self.max_history < 1:
            raise ValueError("AGENT_MAX_HISTORY must be >= 1")

        if self.retrieval_k < 1:
            raise ValueError("AGENT_RETRIEVAL_K must be >= 1")

        if not 0 <= self.temperature <= 2:
            raise ValueError("AGENT_TEMPERATURE must be between 0 and 2")

        if not 0 <= self.confidence_threshold <= 1:
            raise ValueError("AGENT_CONFIDENCE_THRESHOLD must be between 0 and 1")

    def to_dict(self) -> dict:
        """Convert configuration to dictionary (excluding secrets)."""
        return {
            "openai_model": self.openai_model,
            "agent_role": self.agent_role,
            "max_history": self.max_history,
            "max_context_tokens": self.max_context_tokens,
            "retrieval_k": self.retrieval_k,
            "confidence_threshold": self.confidence_threshold,
            "temperature": self.temperature,
        }

    @staticmethod
    def load_from_env() -> "AgentConfig":
        """Factory method to load configuration from environment."""
        return AgentConfig()
