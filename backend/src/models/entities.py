"""SQLAlchemy ORM models for RAG Chatbot Backend database."""

from sqlalchemy import Column, String, Text, Integer, Float, Boolean, DateTime, ForeignKey, ARRAY, JSON, UniqueConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
import uuid

Base = declarative_base()


class Document(Base):
    """Book chapter/section metadata."""
    __tablename__ = "documents"

    document_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter_id = Column(String(50), nullable=False, unique=True, index=True)
    section_id = Column(String(100), nullable=True)
    title = Column(String(255), nullable=False)
    subtitle = Column(String(255), nullable=True)
    text = Column(Text, nullable=False)
    page_range = Column(String(20), nullable=True)
    start_page = Column(Integer, nullable=True)
    end_page = Column(Integer, nullable=True)
    doc_metadata = Column(JSON, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow, index=True)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    status = Column(String(20), default="active")


class Chunk(Base):
    """512-token atomic searchable units of book content."""
    __tablename__ = "chunks"

    chunk_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    document_id = Column(UUID(as_uuid=True), ForeignKey("documents.document_id"), nullable=False, index=True)
    chunk_index = Column(Integer, nullable=False)
    text = Column(Text, nullable=False)
    token_count = Column(Integer, nullable=False)
    start_page = Column(Integer, nullable=True)
    end_page = Column(Integer, nullable=True)
    start_sentence = Column(Integer, nullable=True)
    end_sentence = Column(Integer, nullable=True)
    embedding_model = Column(String(50), default="cohere-embed")
    embedding_confidence = Column(Float, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    status = Column(String(20), default="active")


class Query(Base):
    """User question submitted to chatbot."""
    __tablename__ = "queries"

    query_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_query_text = Column(Text, nullable=False)
    source_mode = Column(String(20), default="full-book")
    selected_passage_id = Column(UUID(as_uuid=True), ForeignKey("chunks.chunk_id"), nullable=True)
    selected_passage_text = Column(Text, nullable=True)
    conversation_id = Column(UUID(as_uuid=True), nullable=True, index=True)
    client_api_key_hash = Column(String(64), nullable=False)
    submitted_at = Column(DateTime, default=datetime.utcnow, index=True)
    response_latency_ms = Column(Integer, nullable=True)
    status = Column(String(20), default="submitted")


class Answer(Base):
    """Generated response with provenance and citations."""
    __tablename__ = "answers"

    answer_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_id = Column(UUID(as_uuid=True), ForeignKey("queries.query_id"), nullable=False, unique=True)
    answer_text = Column(Text, nullable=False)
    answer_summary = Column(String(500), nullable=True)
    source_chunk_ids = Column(ARRAY(UUID(as_uuid=True)), nullable=False)
    confidence_score = Column(Float, nullable=False)
    retrieval_confidence = Column(Float, nullable=False)
    generation_confidence = Column(Float, nullable=False)
    citations = Column(JSON, nullable=False)
    llm_model = Column(String(50), default="claude-3-5-sonnet-20241022")
    llm_temperature = Column(Float, default=0.2)
    generated_at = Column(DateTime, default=datetime.utcnow, index=True)
    status = Column(String(20), default="pending_review")
    audit_trail_id = Column(UUID(as_uuid=True), nullable=True)


class Citation(Base):
    """Detailed citation reference to source chunk."""
    __tablename__ = "citations"

    citation_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    answer_id = Column(UUID(as_uuid=True), ForeignKey("answers.answer_id"), nullable=False, index=True)
    chunk_id = Column(UUID(as_uuid=True), ForeignKey("chunks.chunk_id"), nullable=False, index=True)
    excerpt = Column(Text, nullable=False)
    page_number = Column(Integer, nullable=False)
    section_heading = Column(String(255), nullable=False)
    sentence_indices = Column(ARRAY(Integer), nullable=True)
    source_confidence = Column(Float, nullable=False)
    is_primary = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.utcnow)


class AuditLog(Base):
    """Complete operation trace for compliance and debugging."""
    __tablename__ = "audit_logs"

    log_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_id = Column(UUID(as_uuid=True), ForeignKey("queries.query_id"), nullable=False, index=True)
    answer_id = Column(UUID(as_uuid=True), ForeignKey("answers.answer_id"), nullable=True, index=True)
    user_query_text = Column(Text, nullable=False)
    source_mode = Column(String(20), nullable=False)
    retrieval_query = Column(Text, nullable=True)
    chunks_retrieved_count = Column(Integer, nullable=True)
    chunks_returned = Column(JSON, nullable=True)
    retrieval_latency_ms = Column(Integer, nullable=True)
    llm_prompt = Column(Text, nullable=True)
    llm_response = Column(Text, nullable=True)
    answer_text = Column(Text, nullable=True)
    confidence_score = Column(Float, nullable=True)
    citations = Column(JSON, nullable=True)
    success = Column(Boolean, nullable=False)
    error_code = Column(String(50), nullable=True)
    error_message = Column(Text, nullable=True)
    response_time_ms = Column(Integer, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow, index=True)
    client_api_key_hash = Column(String(64), nullable=False)


class FactCheckGrade(Base):
    """Domain expert fact-checking review."""
    __tablename__ = "fact_check_grades"

    grade_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    answer_id = Column(UUID(as_uuid=True), ForeignKey("answers.answer_id"), nullable=False, unique=True)
    reviewer_id = Column(String(100), nullable=False)
    accuracy_score = Column(Integer, nullable=False)
    hallucination_detected = Column(Boolean, nullable=False)
    factual_errors_count = Column(Integer, default=0)
    citation_errors_count = Column(Integer, default=0)
    comments = Column(Text, nullable=True)
    corrections = Column(JSON, nullable=True)
    approved_for_production = Column(Boolean, nullable=False)
    reviewed_at = Column(DateTime, default=datetime.utcnow, index=True)
    review_duration_minutes = Column(Integer, nullable=True)


class UserChapterTranslation(Base):
    """Cached translations for chapters, per user and language."""
    __tablename__ = "user_chapter_translations"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
    chapter_id = Column(String(100), nullable=False, index=True)
    target_language = Column(String(20), nullable=False)
    original_content = Column(Text, nullable=False)
    translated_content = Column(Text, nullable=False)
    confidence_score = Column(Float, default=0.9)
    from_cache = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.utcnow, index=True)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    expires_at = Column(DateTime, nullable=True, index=True)  # For 30-day TTL

    # Composite unique key: (user_id, chapter_id, target_language)
    # This ensures one cached translation per user per chapter per language
    __table_args__ = (
        UniqueConstraint('user_id', 'chapter_id', 'target_language', name='uq_user_chapter_language'),
    )
