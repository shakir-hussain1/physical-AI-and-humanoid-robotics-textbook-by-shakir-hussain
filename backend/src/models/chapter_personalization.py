"""Chapter Personalization model - Per-user, per-chapter preferences."""

from sqlalchemy import Column, Integer, String, JSON, DateTime, ForeignKey, UniqueConstraint
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime

Base = declarative_base()


class ChapterPersonalization(Base):
    """Per-user, per-chapter personalization preferences."""

    __tablename__ = "chapter_personalizations"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False, index=True)
    chapter_id = Column(String(255), nullable=False, index=True)  # e.g., "modules/module-1-ros2/chapter-01"

    # Personalization settings
    difficulty_level = Column(String(50), default="intermediate")  # beginner, intermediate, advanced, expert
    content_style = Column(String(50), default="balanced")  # text, visual, code, balanced
    example_density = Column(String(50), default="moderate")  # minimal, moderate, rich
    learning_pace = Column(String(50), default="standard")  # concise, standard, detailed

    # Additional metadata for future extensions
    custom_preferences = Column(JSON, default=dict)

    # Timestamps
    created_at = Column(DateTime, default=datetime.utcnow, index=True)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Unique constraint: one personalization per user per chapter
    __table_args__ = (
        UniqueConstraint('user_id', 'chapter_id', name='unique_user_chapter_personalization'),
    )

    def __repr__(self):
        return f"<ChapterPersonalization user_id={self.user_id} chapter_id={self.chapter_id}>"
