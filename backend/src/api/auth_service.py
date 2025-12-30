"""Authentication service for user registration and login."""

import os
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from jose import JWTError, jwt
from passlib.context import CryptContext
from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
import json

from .auth_models import User, UserProfile

# Try to get logger, fall back to basic logging if not available
try:
    from ..utils.logging import get_logger
    logger = get_logger(__name__)
except ImportError:
    import logging
    logger = logging.getLogger(__name__)

# Password hashing configuration
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto", bcrypt__rounds=12)

# JWT configuration
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-super-secret-jwt-key-change-in-production-12345678")
JWT_ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
JWT_ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("JWT_EXPIRE_MINUTES", "15"))
JWT_REFRESH_TOKEN_EXPIRE_DAYS = int(os.getenv("JWT_REFRESH_EXPIRE_DAYS", "7"))


class PasswordService:
    """Service for password hashing and verification."""

    @staticmethod
    def hash_password(password: str) -> str:
        """Hash a password using bcrypt with cost factor 12."""
        return pwd_context.hash(password)

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify a plain password against a hashed password."""
        return pwd_context.verify(plain_password, hashed_password)


class TokenService:
    """Service for JWT token generation and validation."""

    @staticmethod
    def create_access_token(user_id: str, email: str, expires_delta: Optional[timedelta] = None) -> str:
        """Create a JWT access token."""
        if expires_delta is None:
            expires_delta = timedelta(minutes=JWT_ACCESS_TOKEN_EXPIRE_MINUTES)

        expire = datetime.utcnow() + expires_delta
        payload = {
            "sub": user_id,
            "email": email,
            "exp": expire,
            "type": "access",
        }

        encoded_jwt = jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)
        return encoded_jwt

    @staticmethod
    def create_refresh_token(user_id: str, email: str) -> str:
        """Create a JWT refresh token."""
        expires_delta = timedelta(days=JWT_REFRESH_TOKEN_EXPIRE_DAYS)
        expire = datetime.utcnow() + expires_delta
        payload = {
            "sub": user_id,
            "email": email,
            "exp": expire,
            "type": "refresh",
        }

        encoded_jwt = jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)
        return encoded_jwt

    @staticmethod
    def verify_token(token: str) -> Optional[Dict[str, Any]]:
        """Verify and decode a JWT token."""
        try:
            payload = jwt.decode(token, JWT_SECRET_KEY, algorithms=[JWT_ALGORITHM])
            user_id: str = payload.get("sub")
            if user_id is None:
                return None
            return payload
        except JWTError:
            return None

    @staticmethod
    def refresh_access_token(refresh_token: str) -> Optional[str]:
        """Generate a new access token from a refresh token."""
        payload = TokenService.verify_token(refresh_token)
        if payload is None or payload.get("type") != "refresh":
            return None

        user_id = payload.get("sub")
        email = payload.get("email")

        if user_id is None or email is None:
            return None

        return TokenService.create_access_token(user_id, email)


class AuthService:
    """Service for user registration and authentication."""

    def __init__(self, db: Session):
        """Initialize auth service with database session."""
        self.db = db
        self.password_service = PasswordService()
        self.token_service = TokenService()

    def register_user(
        self,
        email: str,
        password: str,
        first_name: Optional[str] = None,
        last_name: Optional[str] = None,
        software_background: Optional[str] = None,
        hardware_background: Optional[str] = None,
        programming_languages: Optional[list] = None,
        robotics_experience: Optional[str] = None,
        ai_ml_experience: Optional[str] = None,
    ) -> Optional[User]:
        """Register a new user with optional background information."""
        try:
            # Check if user already exists
            existing_user = self.db.query(User).filter(User.email == email).first()
            if existing_user:
                logger.warning(f"Registration attempt with existing email: {email}")
                raise ValueError(f"User with email {email} already exists")

            # Hash password
            hashed_password = self.password_service.hash_password(password)

            # Create new user
            new_user = User(
                email=email,
                password_hash=hashed_password,
                first_name=first_name,
                last_name=last_name,
            )

            self.db.add(new_user)
            self.db.flush()  # Flush to get the user ID

            # Create user profile with background information
            profile = UserProfile(
                user_id=new_user.id,
                software_background=software_background,
                hardware_background=hardware_background,
                programming_languages=json.dumps(programming_languages) if programming_languages else None,
                robotics_experience=robotics_experience,
                ai_ml_experience=ai_ml_experience,
            )

            self.db.add(profile)
            self.db.commit()

            logger.info(f"User registered successfully: {email}")
            return new_user

        except IntegrityError as e:
            self.db.rollback()
            logger.error(f"Database integrity error during registration: {str(e)}")
            raise ValueError("Failed to register user - database error")
        except Exception as e:
            self.db.rollback()
            logger.error(f"Error during user registration: {str(e)}")
            raise

    def authenticate_user(self, email: str, password: str) -> Optional[Dict[str, Any]]:
        """Authenticate user and return tokens."""
        try:
            # Find user by email
            user = self.db.query(User).filter(User.email == email).first()

            if not user:
                logger.warning(f"Login attempt with non-existent email: {email}")
                return None

            if not user.is_active:
                logger.warning(f"Login attempt with inactive user: {email}")
                return None

            # Verify password
            if not self.password_service.verify_password(password, user.password_hash):
                logger.warning(f"Failed login attempt for user: {email}")
                return None

            # Generate tokens
            access_token = self.token_service.create_access_token(user.id, user.email)
            refresh_token = self.token_service.create_refresh_token(user.id, user.email)

            logger.info(f"User authenticated successfully: {email}")

            return {
                "user_id": user.id,
                "email": user.email,
                "first_name": user.first_name,
                "last_name": user.last_name,
                "access_token": access_token,
                "refresh_token": refresh_token,
                "token_type": "bearer",
            }

        except Exception as e:
            logger.error(f"Error during authentication: {str(e)}")
            return None

    def get_user_by_id(self, user_id: str) -> Optional[User]:
        """Get user by ID."""
        try:
            user = self.db.query(User).filter(User.id == user_id).first()
            return user
        except Exception as e:
            logger.error(f"Error fetching user: {str(e)}")
            return None

    def get_user_profile(self, user_id: str) -> Optional[Dict[str, Any]]:
        """Get user profile with background information."""
        try:
            user = self.get_user_by_id(user_id)
            if not user:
                return None

            profile = user.profile
            if not profile:
                return None

            return {
                "user_id": user.id,
                "email": user.email,
                "first_name": user.first_name,
                "last_name": user.last_name,
                "software_background": profile.software_background,
                "hardware_background": profile.hardware_background,
                "programming_languages": json.loads(profile.programming_languages) if profile.programming_languages else [],
                "robotics_experience": profile.robotics_experience,
                "ai_ml_experience": profile.ai_ml_experience,
                "preferred_language": profile.preferred_language,
                "theme": profile.theme,
                "show_advanced_content": profile.show_advanced_content,
            }

        except Exception as e:
            logger.error(f"Error fetching user profile: {str(e)}")
            return None

    def update_user_profile(self, user_id: str, **kwargs) -> Optional[Dict[str, Any]]:
        """Update user profile information."""
        try:
            user = self.get_user_by_id(user_id)
            if not user or not user.profile:
                return None

            profile = user.profile

            # Update allowed fields
            allowed_fields = [
                "software_background",
                "hardware_background",
                "programming_languages",
                "robotics_experience",
                "ai_ml_experience",
                "preferred_language",
                "theme",
                "show_advanced_content",
            ]

            for field, value in kwargs.items():
                if field in allowed_fields:
                    if field == "programming_languages" and isinstance(value, list):
                        setattr(profile, field, json.dumps(value))
                    else:
                        setattr(profile, field, value)

            profile.updated_at = datetime.utcnow()
            self.db.commit()

            logger.info(f"User profile updated: {user_id}")
            return self.get_user_profile(user_id)

        except Exception as e:
            self.db.rollback()
            logger.error(f"Error updating user profile: {str(e)}")
            return None
