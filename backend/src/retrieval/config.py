"""Configuration management for retrieval service.

Loads and validates configuration from environment variables.
"""

import os
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class RetrievalConfig:
    """Retrieval service configuration."""

    def __init__(self):
        """Initialize configuration from environment variables."""
        # Qdrant configuration
        self.qdrant_url: str = os.getenv("QDRANT_URL", "")
        self.qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
        self.qdrant_collection: str = os.getenv("QDRANT_COLLECTION", "textbook_embeddings")

        # Cohere configuration
        self.cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
        self.cohere_model: str = os.getenv("COHERE_MODEL", "embed-english-v3.0")

        # Retrieval settings
        self.default_k: int = int(os.getenv("RETRIEVAL_DEFAULT_K", "5"))
        self.max_k: int = int(os.getenv("RETRIEVAL_MAX_K", "100"))
        self.request_timeout: int = int(os.getenv("RETRIEVAL_TIMEOUT_SECONDS", "30"))

        # Logging configuration
        self.log_level: str = os.getenv("LOG_LEVEL", "INFO")
        self.log_format: str = os.getenv("LOG_FORMAT", "json")

        # Validation at initialization
        self._validate()

    def _validate(self) -> None:
        """Validate required configuration is present.

        Raises:
            ValueError: If any required configuration is missing
        """
        errors = []

        if not self.qdrant_url:
            errors.append("QDRANT_URL is not set")
        if not self.qdrant_api_key:
            errors.append("QDRANT_API_KEY is not set")
        if not self.cohere_api_key:
            errors.append("COHERE_API_KEY is not set")

        if errors:
            logger.error(f"Configuration validation failed: {'; '.join(errors)}")
            raise ValueError(f"Missing required configuration: {'; '.join(errors)}")

        logger.info("Configuration validation passed")

    def to_dict(self) -> dict:
        """Export configuration as dictionary (excluding secrets).

        Returns:
            Dictionary with non-secret configuration values
        """
        return {
            "qdrant_url": self.qdrant_url,
            "qdrant_collection": self.qdrant_collection,
            "cohere_model": self.cohere_model,
            "default_k": self.default_k,
            "max_k": self.max_k,
            "request_timeout": self.request_timeout,
            "log_level": self.log_level,
        }

    @staticmethod
    def load_from_env() -> "RetrievalConfig":
        """Load configuration from environment variables.

        Returns:
            RetrievalConfig instance

        Raises:
            ValueError: If required configuration is missing
        """
        try:
            config = RetrievalConfig()
            logger.info("Configuration loaded successfully from environment")
            return config
        except ValueError as e:
            logger.error(f"Failed to load configuration: {str(e)}")
            raise


# Default instance
_config: Optional[RetrievalConfig] = None


def get_config() -> RetrievalConfig:
    """Get global configuration instance (lazy loading).

    Returns:
        RetrievalConfig instance

    Raises:
        ValueError: If configuration cannot be loaded
    """
    global _config
    if _config is None:
        _config = RetrievalConfig.load_from_env()
    return _config
