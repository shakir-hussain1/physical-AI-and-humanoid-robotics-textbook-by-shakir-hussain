"""Audit logging service for tracking queries and responses."""
from typing import Dict, Any, Optional
import logging
import uuid
from datetime import datetime

from src.models.entities import AuditLogORM, QueryORM
from src.db.postgres import get_db_session

logger = logging.getLogger(__name__)

class AuditLoggerService:
    """Service for logging all operations for audit and fact-checking."""

    def __init__(self, db_session=None):
        self.db_session = db_session

    async def log_query(self, user_query: str, source_mode: str = "standard") -> str:
        """Log a query to the audit system."""
        query_id = str(uuid.uuid4())

        # For MVP, we'll just log to console
        # In a real implementation, this would write to PostgreSQL
        log_entry = {
            "query_id": query_id,
            "user_query_text": user_query,
            "source_mode": source_mode,
            "submitted_at": datetime.utcnow().isoformat(),
            "status": "logged"
        }

        logger.info(f"Audit log - Query: {log_entry}")
        return query_id

    async def log_retrieval(self, query_id: str, retrieval_query: str, chunks_retrieved_count: int,
                           chunks_returned: list, retrieval_latency_ms: float) -> str:
        """Log retrieval operation details."""
        retrieval_id = str(uuid.uuid4())

        log_entry = {
            "retrieval_id": retrieval_id,
            "query_id": query_id,
            "retrieval_query": retrieval_query,
            "chunks_retrieved_count": chunks_retrieved_count,
            "chunks_returned": chunks_returned,
            "retrieval_latency_ms": retrieval_latency_ms,
            "logged_at": datetime.utcnow().isoformat()
        }

        logger.info(f"Audit log - Retrieval: {log_entry}")
        return retrieval_id

    async def log_answer(self, query_id: str, llm_prompt: str, llm_response: str,
                        answer_text: str, confidence_score: float, citations: list) -> str:
        """Log the generated answer and related data."""
        answer_id = str(uuid.uuid4())

        log_entry = {
            "answer_id": answer_id,
            "query_id": query_id,
            "llm_prompt": llm_prompt,
            "llm_response": llm_response,
            "answer_text": answer_text,
            "confidence_score": confidence_score,
            "citations": citations,
            "logged_at": datetime.utcnow().isoformat()
        }

        logger.info(f"Audit log - Answer: {log_entry}")
        return answer_id

    async def log_full_trace(self, query_id: str, full_trace_data: Dict[str, Any]) -> str:
        """Log complete operation trace for fact-checking."""
        trace_id = str(uuid.uuid4())

        log_entry = {
            "trace_id": trace_id,
            "query_id": query_id,
            "full_trace_data": full_trace_data,
            "logged_at": datetime.utcnow().isoformat()
        }

        logger.info(f"Audit log - Full trace: {log_entry}")
        return trace_id

    async def log_operation(self, operation_data: Dict[str, Any]) -> bool:
        """Generic method to log an operation."""
        try:
            logger.info(f"Operation logged: {operation_data}")
            return True
        except Exception as e:
            logger.error(f"Error logging operation: {e}")
            return False