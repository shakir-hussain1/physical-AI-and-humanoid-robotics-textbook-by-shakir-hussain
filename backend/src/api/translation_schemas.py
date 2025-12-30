"""Pydantic schemas for translation endpoints."""

from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from datetime import datetime


class TranslateContentRequest(BaseModel):
    """Request to translate content."""

    content_id: str = Field(..., description="Unique content/chapter ID")
    content_text: str = Field(..., min_length=1, max_length=100000, description="Content to translate")
    content_type: str = Field(default="section", description="Type: section or chapter")
    target_language: str = Field(default="ur", description="Target language code (ur=Urdu)")


class BatchTranslateRequest(BaseModel):
    """Request to translate multiple chapters."""

    chapters: List[Dict[str, str]] = Field(
        ..., description="List of chapters with id and content"
    )
    target_language: str = Field(default="ur", description="Target language code")


class TranslationResponse(BaseModel):
    """Translation response."""

    id: str = Field(..., description="Translation ID")
    content_id: str = Field(..., description="Content ID")
    language: str = Field(..., description="Language code")
    content_type: str = Field(..., description="Content type: section or chapter")
    original_content: str = Field(..., description="Original content")
    translated_content: str = Field(..., description="Translated content")
    status: str = Field(..., description="Translation status")
    accuracy_score: Optional[float] = Field(None, description="Accuracy score 0-100")
    word_count: int = Field(..., description="Word count of original")
    terminology_used: List[str] = Field(default=[], description="Terminology IDs used")
    custom_terms_applied: int = Field(..., description="Number of custom terms applied")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")


class TranslationStatusResponse(BaseModel):
    """Translation status response."""

    content_id: str = Field(..., description="Content ID")
    status: str = Field(..., description="Translation status")
    available: bool = Field(..., description="Is translation available")
    accuracy_score: Optional[float] = Field(None, description="Accuracy score")
    word_count: Optional[int] = Field(None, description="Word count")
    terminology_used: Optional[int] = Field(None, description="Terminology terms used")
    created_at: Optional[datetime] = Field(None, description="Creation timestamp")


class LoadTerminologyRequest(BaseModel):
    """Request to load terminology."""

    terms: List[Dict[str, Any]] = Field(
        ...,
        description="List of terms with en, ur, category, approved, priority",
    )


class TerminologyResponse(BaseModel):
    """Single terminology entry."""

    id: str = Field(..., description="Term ID")
    english: str = Field(..., description="English term")
    urdu: str = Field(..., description="Urdu translation")
    category: str = Field(..., description="Category")
    priority: int = Field(..., description="Priority for replacement")


class ToggleLanguageRequest(BaseModel):
    """Request to toggle language preference."""

    chapter_id: str = Field(..., description="Chapter ID")
    language: str = Field(default="ur", description="Target language code")


class RTLResponse(BaseModel):
    """RTL layout configuration."""

    is_rtl: bool = Field(..., description="Is RTL layout needed")
    text_align: str = Field(..., description="Text alignment: right or left")
    direction: str = Field(..., description="Text direction: rtl or ltr")
    font_family: str = Field(..., description="Recommended font family")
