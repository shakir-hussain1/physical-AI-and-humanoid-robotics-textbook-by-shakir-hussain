"""Pydantic schemas for authentication endpoints."""

from typing import Optional, List
from pydantic import BaseModel, Field, EmailStr, field_validator
from datetime import datetime


class SignupRequest(BaseModel):
    """Signup request model."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, max_length=100, description="User password (min 8 chars)")
    first_name: Optional[str] = Field(None, max_length=255, description="User first name")
    last_name: Optional[str] = Field(None, max_length=255, description="User last name")

    # Background information
    software_background: Optional[str] = Field(None, description="Software background level")
    hardware_background: Optional[str] = Field(None, description="Hardware background level")
    programming_languages: Optional[List[str]] = Field(None, description="Known programming languages")
    robotics_experience: Optional[str] = Field(None, description="Robotics experience level")
    ai_ml_experience: Optional[str] = Field(None, description="AI/ML experience level")

    @field_validator("password")
    @classmethod
    def validate_password(cls, v: str) -> str:
        """Validate password strength."""
        if not any(char.isupper() for char in v):
            raise ValueError("Password must contain at least one uppercase letter")
        if not any(char.isdigit() for char in v):
            raise ValueError("Password must contain at least one digit")
        return v

    @field_validator("software_background", "hardware_background")
    @classmethod
    def validate_background_level(cls, v: Optional[str]) -> Optional[str]:
        """Validate background level values."""
        if v and v not in ["beginner", "intermediate", "advanced", "expert"]:
            raise ValueError("Background level must be one of: beginner, intermediate, advanced, expert")
        return v

    @field_validator("robotics_experience", "ai_ml_experience")
    @classmethod
    def validate_experience_level(cls, v: Optional[str]) -> Optional[str]:
        """Validate experience level values."""
        if v and v not in ["none", "some", "intermediate", "advanced"]:
            raise ValueError("Experience level must be one of: none, some, intermediate, advanced")
        return v


class LoginRequest(BaseModel):
    """Login request model."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")


class TokenResponse(BaseModel):
    """Token response model."""

    access_token: str = Field(..., description="JWT access token")
    refresh_token: str = Field(..., description="JWT refresh token")
    token_type: str = Field(default="bearer", description="Token type")
    expires_in: int = Field(..., description="Access token expiration in seconds")


class UserResponse(BaseModel):
    """User response model."""

    user_id: str = Field(..., description="User ID")
    email: str = Field(..., description="User email")
    first_name: Optional[str] = Field(None, description="First name")
    last_name: Optional[str] = Field(None, description="Last name")
    is_active: bool = Field(default=True, description="Is user active")
    created_at: datetime = Field(..., description="User creation timestamp")


class AuthResponse(BaseModel):
    """Authentication response model."""

    user: UserResponse = Field(..., description="User information")
    tokens: TokenResponse = Field(..., description="Access and refresh tokens")


class RefreshTokenRequest(BaseModel):
    """Refresh token request model."""

    refresh_token: str = Field(..., description="JWT refresh token")


class UserProfileResponse(BaseModel):
    """User profile response model."""

    user_id: str = Field(..., description="User ID")
    email: str = Field(..., description="User email")
    first_name: Optional[str] = Field(None, description="First name")
    last_name: Optional[str] = Field(None, description="Last name")
    software_background: Optional[str] = Field(None, description="Software background")
    hardware_background: Optional[str] = Field(None, description="Hardware background")
    programming_languages: List[str] = Field(default=[], description="Programming languages")
    robotics_experience: Optional[str] = Field(None, description="Robotics experience")
    ai_ml_experience: Optional[str] = Field(None, description="AI/ML experience")
    preferred_language: str = Field(default="en", description="Preferred language")
    theme: str = Field(default="light", description="UI theme preference")
    show_advanced_content: bool = Field(default=False, description="Show advanced content")


class UpdateProfileRequest(BaseModel):
    """Update user profile request model."""

    first_name: Optional[str] = Field(None, max_length=255, description="First name")
    last_name: Optional[str] = Field(None, max_length=255, description="Last name")
    software_background: Optional[str] = Field(None, description="Software background level")
    hardware_background: Optional[str] = Field(None, description="Hardware background level")
    programming_languages: Optional[List[str]] = Field(None, description="Programming languages")
    robotics_experience: Optional[str] = Field(None, description="Robotics experience level")
    ai_ml_experience: Optional[str] = Field(None, description="AI/ML experience level")
    preferred_language: Optional[str] = Field(None, description="Preferred language")
    theme: Optional[str] = Field(None, description="UI theme preference")
    show_advanced_content: Optional[bool] = Field(None, description="Show advanced content")


class ErrorResponse(BaseModel):
    """Error response model."""

    error: str = Field(..., description="Error code/type")
    message: str = Field(..., description="Human-readable error message")
    request_id: str = Field(..., description="Request ID for debugging")
