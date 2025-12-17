"""Application constants."""

# Query settings
QUERY_MIN_LENGTH = 1
QUERY_MAX_LENGTH = 1000
CONFIDENCE_THRESHOLD = 0.60
RETRIEVAL_TOP_K = 5
TIMEOUT_SECONDS = 5

# Chunking settings
MIN_TOKEN_COUNT = 256
MAX_TOKEN_COUNT = 512
CHUNK_OVERLAP_RATIO = 0.20

# Models
COHERE_EMBED_MODEL = "embed-english-v3.0"
CLAUDE_MODEL = "claude-3-5-sonnet-20241022"
CLAUDE_TEMPERATURE = 0.2

# Confidence levels
CONFIDENCE_HIGH_THRESHOLD = 0.85
CONFIDENCE_MEDIUM_THRESHOLD = 0.70

# Error codes
ERROR_INVALID_QUERY = "INVALID_QUERY"
ERROR_LOW_CONFIDENCE = "LOW_CONFIDENCE"
ERROR_TIMEOUT = "TIMEOUT"
ERROR_NO_MATCH = "NO_MATCH"
ERROR_PASSAGE_INSUFFICIENT = "PASSAGE_INSUFFICIENT"
ERROR_UNAUTHORIZED = "UNAUTHORIZED"
ERROR_SERVER_ERROR = "SERVER_ERROR"

# Error messages
MSG_INVALID_QUERY = "Query must be 1-1000 characters"
MSG_LOW_CONFIDENCE = "Unable to find relevant content. Please rephrase your question."
MSG_TIMEOUT = "Query processing timed out. Please try a simpler question."
MSG_OUT_OF_SCOPE = "This question is not covered in the Physical AI textbook"
MSG_PASSAGE_INSUFFICIENT = "This question cannot be fully answered from your selected passage. Would you like to expand to the full chapter?"
MSG_UNAUTHORIZED = "Unauthorized: Invalid API key"

# Hallucination thresholds
HALLUCINATION_ROLLBACK_THRESHOLD = 0.05  # 5%
ACCURACY_DROP_ALERT_THRESHOLD = 0.02  # 2%
RETRIEVAL_FAILURE_ALERT_THRESHOLD = 0.10  # 10%
LATENCY_P95_ALERT_THRESHOLD_MS = 3000  # 3 seconds

# Fact-checking
FACT_CHECK_SAMPLE_SIZE = 30  # 20-30 answers per month
FACT_CHECK_REQUIRED_ACCURACY = 0.95  # 95%
