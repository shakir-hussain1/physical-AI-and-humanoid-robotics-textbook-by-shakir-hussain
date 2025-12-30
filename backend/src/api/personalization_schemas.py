"""Pydantic schemas for personalization endpoints."""

from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from datetime import datetime


class SetPreferenceRequest(BaseModel):
    """Set content preference request."""

    content_level: str = Field(..., description="Content level: beginner, intermediate, advanced")
    show_math: bool = Field(default=True, description="Show mathematical content")
    show_code: bool = Field(default=True, description="Show code examples")
    show_diagrams: bool = Field(default=True, description="Show diagrams and visuals")
    show_advanced_topics: bool = Field(default=False, description="Show advanced topics")
    focus_areas: Optional[List[str]] = Field(None, description="Focus areas for personalization")


class SetChapterPreferenceRequest(SetPreferenceRequest):
    """Set chapter-specific preference request."""

    chapter_id: str = Field(..., description="Chapter ID")
    module_id: Optional[str] = Field(None, description="Module ID (optional)")


class SetModulePreferenceRequest(SetPreferenceRequest):
    """Set module-specific preference request."""

    module_id: str = Field(..., description="Module ID")


class PreferenceResponse(BaseModel):
    """Individual preference response."""

    id: str = Field(..., description="Preference ID")
    module_id: Optional[str] = Field(None, description="Module ID")
    chapter_id: Optional[str] = Field(None, description="Chapter ID")
    content_level: str = Field(..., description="Content level")
    show_math: bool = Field(..., description="Show math content")
    show_code: bool = Field(..., description="Show code")
    show_diagrams: bool = Field(..., description="Show diagrams")
    show_advanced_topics: bool = Field(..., description="Show advanced topics")
    focus_areas: List[str] = Field(..., description="Focus areas")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")


class AllPreferencesResponse(BaseModel):
    """All user preferences organized by level."""

    global_preference: Optional[PreferenceResponse] = Field(None, description="Global preference")
    module_preferences: Dict[str, PreferenceResponse] = Field(
        default_factory=dict, description="Module-level preferences"
    )
    chapter_preferences: Dict[str, PreferenceResponse] = Field(
        default_factory=dict, description="Chapter-level preferences"
    )


class ContentFilterRequest(BaseModel):
    """Request to get filtered content based on preferences."""

    chapter_id: str = Field(..., description="Chapter ID")
    module_id: Optional[str] = Field(None, description="Module ID (optional)")


class EffectivePreferenceResponse(BaseModel):
    """Effective preference response with hierarchy applied."""

    content_level: str = Field(..., description="Effective content level")
    show_math: bool = Field(..., description="Show math content")
    show_code: bool = Field(..., description="Show code")
    show_diagrams: bool = Field(..., description="Show diagrams")
    show_advanced_topics: bool = Field(..., description="Show advanced topics")
    focus_areas: List[str] = Field(..., description="Effective focus areas")
    applied_from: str = Field(..., description="Source of preference: chapter, module, or global")
