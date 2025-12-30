"""Database models for authentication and user profiles."""

from datetime import datetime
from typing import Optional
from sqlalchemy import Column, String, DateTime, Boolean, ForeignKey, Text, Integer, Float
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
import uuid

Base = declarative_base()


class User(Base):
    """User model for authentication."""

    __tablename__ = "users"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    first_name = Column(String(255), nullable=True)
    last_name = Column(String(255), nullable=True)
    is_active = Column(Boolean, default=True, nullable=False)
    is_verified = Column(Boolean, default=False, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    # Relationships
    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")


class UserProfile(Base):
    """User profile with background information for personalization."""

    __tablename__ = "user_profiles"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String(36), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, unique=True)

    # Background information
    software_background = Column(String(50), nullable=True)  # beginner, intermediate, advanced, expert
    hardware_background = Column(String(50), nullable=True)  # beginner, intermediate, advanced, expert
    programming_languages = Column(Text, nullable=True)  # JSON array
    robotics_experience = Column(String(50), nullable=True)  # none, some, intermediate, advanced
    ai_ml_experience = Column(String(50), nullable=True)  # none, some, intermediate, advanced
    preferred_language = Column(String(10), default="en", nullable=False)  # Language preference

    # Preferences
    theme = Column(String(20), default="light", nullable=False)  # light, dark
    notifications_enabled = Column(Boolean, default=True, nullable=False)

    # Content preferences
    show_advanced_content = Column(Boolean, default=False, nullable=False)
    focus_areas = Column(Text, nullable=True)  # JSON array

    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    # Relationships
    user = relationship("User", back_populates="profile")
    preferences = relationship("ContentPreference", back_populates="profile", cascade="all, delete-orphan")


class ContentPreference(Base):
    """User content preferences for personalization."""

    __tablename__ = "content_preferences"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    user_profile_id = Column(String(36), ForeignKey("user_profiles.id", ondelete="CASCADE"), nullable=False, index=True)

    # Content level scoping
    module_id = Column(String(100), nullable=True, index=True)  # NULL = global preference
    chapter_id = Column(String(100), nullable=True, index=True)  # NULL = module preference

    # Preference settings
    content_level = Column(String(20), nullable=False, default="beginner")  # beginner, intermediate, advanced
    show_math = Column(Boolean, default=True)
    show_code = Column(Boolean, default=True)
    show_diagrams = Column(Boolean, default=True)
    show_advanced_topics = Column(Boolean, default=False)
    focus_areas = Column(Text, nullable=True)  # JSON array of focus areas

    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    # Relationships
    profile = relationship("UserProfile", back_populates="preferences")
    translations = relationship("Translation", back_populates="content_preference", cascade="all, delete-orphan")


class Translation(Base):
    """Translation cache for content."""

    __tablename__ = "translations"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    content_preference_id = Column(String(36), ForeignKey("content_preferences.id", ondelete="CASCADE"), nullable=False, index=True)

    # Translation metadata
    content_id = Column(String(200), nullable=False, index=True)  # Chapter ID or content section ID
    language = Column(String(10), nullable=False, default="ur")  # Language code
    content_type = Column(String(50), nullable=False)  # "chapter" or "section"

    # Original and translated content
    original_content = Column(Text, nullable=False)
    translated_content = Column(Text, nullable=False)

    # Quality tracking
    translation_status = Column(String(50), default="auto_generated")  # auto_generated, verified, expert_reviewed
    accuracy_score = Column(Float, nullable=True)  # 0-100 accuracy percentage
    word_count = Column(Integer, nullable=False, default=0)
    is_cached = Column(Boolean, default=True)

    # Terminology usage
    terminology_used = Column(Text, nullable=True)  # JSON array of term IDs used
    custom_terms_applied = Column(Integer, default=0)

    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)

    # Relationships
    content_preference = relationship("ContentPreference", back_populates="translations")


class TranslationTerminology(Base):
    """Pre-translated robotics terminology for consistency."""

    __tablename__ = "translation_terminology"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    english_term = Column(String(255), nullable=False, unique=True, index=True)
    urdu_translation = Column(String(255), nullable=False)

    # Metadata
    category = Column(String(50), nullable=False, index=True)  # hardware, software, ai, general, etc.
    is_approved = Column(Boolean, default=True)
    usage_count = Column(Integer, default=0)
    priority = Column(Integer, default=0)  # Higher priority = preferred in translations

    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
