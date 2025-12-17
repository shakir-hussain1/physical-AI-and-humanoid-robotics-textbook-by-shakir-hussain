"""Audit models for fast JSON serialization."""
from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime
import uuid

class AuditLog(BaseModel):
    """Audit log model for fast JSON serialization, independent of ORM."""

    id: str
    query_id: str
    operation_type: str
    operation_data: Dict[str, Any]
    timestamp: datetime
    user_id: Optional[str] = None
    ip_address: Optional[str] = None
    success: bool = True
    error_code: Optional[str] = None
    response_time_ms: Optional[float] = None
    trace_id: Optional[str] = None

    class Config:
        # Allow serialization of datetime objects
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

class FactCheckGrade(BaseModel):
    """Model for fact-check grading by educators."""

    grade_id: str = str(uuid.uuid4())
    answer_id: str
    reviewer_id: str
    accuracy_score: float  # 0-100%
    hallucination_detected: bool = False
    approved_for_production: bool = False
    comments: Optional[str] = None
    graded_at: datetime = datetime.utcnow()

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }