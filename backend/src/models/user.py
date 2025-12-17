"""User model with authentication and background profile."""

from sqlalchemy import Column, Integer, String, Boolean, DateTime, JSON
from sqlalchemy.ext.declarative import declarative_base
from datetime import datetime
from enum import Enum

Base = declarative_base()

class SoftwareLevel(str, Enum):
    """Software experience levels."""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
    EXPERT = "expert"

class HardwareLevel(str, Enum):
    """Hardware experience levels."""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
    EXPERT = "expert"

class User(Base):
    """User model with authentication and personalization."""

    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String(255), unique=True, index=True, nullable=False)
    username = Column(String(100), unique=True, index=True, nullable=False)
    full_name = Column(String(255), nullable=True)
    hashed_password = Column(String(255), nullable=False)
    is_active = Column(Boolean, default=True)
    is_verified = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Background & Personalization
    software_background = Column(String(50), default=SoftwareLevel.BEGINNER)  # beginner, intermediate, advanced, expert
    hardware_background = Column(String(50), default=HardwareLevel.BEGINNER)  # beginner, intermediate, advanced, expert

    # Programming languages experience
    programming_languages = Column(JSON, default=list)  # ["Python", "C++", "ROS"]

    # Robotics interest areas
    interest_areas = Column(JSON, default=list)  # ["ROS", "Kinematics", "Control", "Perception"]

    # Professional background
    profession = Column(String(100), nullable=True)  # Student, Professional, Researcher, etc.
    organization = Column(String(255), nullable=True)
    years_of_experience = Column(Integer, nullable=True)

    # Learning preferences
    preferred_learning_style = Column(String(50), nullable=True)  # visual, text, interactive, hands-on
    learning_pace = Column(String(50), default="moderate")  # slow, moderate, fast

    # Personalization metadata
    personalization_data = Column(JSON, default=dict)  # Custom personalization data

    def __repr__(self):
        return f"<User {self.username} ({self.email})>"
