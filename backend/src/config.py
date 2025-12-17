"""Configuration management for RAG Chatbot Backend."""

from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    """Application settings from environment variables."""

    # API Keys
    ANTHROPIC_API_KEY: str
    COHERE_API_KEY: str

    # Database
    DATABASE_URL: str = "postgresql://postgres:postgres@localhost:5432/ragchatbot"

    # Vector Database
    QDRANT_URL: str = "http://localhost:6333"
    QDRANT_API_KEY: Optional[str] = None

    # FastAPI Configuration
    FASTAPI_ENV: str = "development"
    FASTAPI_HOST: str = "0.0.0.0"
    FASTAPI_PORT: int = 8000

    # Logging
    LOG_LEVEL: str = "INFO"

    # Application Settings
    QUERY_MAX_LENGTH: int = 1000
    QUERY_MIN_LENGTH: int = 1
    CONFIDENCE_THRESHOLD: float = 0.60
    RETRIEVAL_TOP_K: int = 5
    TIMEOUT_SECONDS: int = 5
    MIN_TOKEN_COUNT: int = 256
    MAX_TOKEN_COUNT: int = 512

    # Models
    COHERE_EMBED_MODEL: str = "embed-english-v3.0"
    CLAUDE_MODEL: str = "claude-3-5-sonnet-20241022"
    CLAUDE_TEMPERATURE: float = 0.2

    # Feature Flags
    ENABLE_CONTEXT_RESTRICTED_MODE: bool = True
    ENABLE_FACT_CHECKING_GRADE: bool = True

    # Rollback Thresholds
    HALLUCINATION_ROLLBACK_THRESHOLD: float = 0.05  # 5% hallucination rate
    ACCURACY_DROP_ALERT_THRESHOLD: float = 0.02  # 2% drop
    RETRIEVAL_FAILURE_ALERT_THRESHOLD: float = 0.10  # 10% failure rate
    LATENCY_P95_ALERT_THRESHOLD_MS: int = 3000  # 3 seconds

    class Config:
        env_file = ".env"
        case_sensitive = True

# Global settings instance
settings = Settings()
