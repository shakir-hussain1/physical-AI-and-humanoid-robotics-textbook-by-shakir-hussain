"""Authentication endpoints for signup, signin, and token refresh."""

from datetime import timedelta, datetime
from fastapi import APIRouter, Depends, HTTPException, status, Request
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session

from ..auth_service import AuthService, TokenService
from ..auth_schemas import (
    SignupRequest,
    LoginRequest,
    TokenResponse,
    AuthResponse,
    UserResponse,
    RefreshTokenRequest,
    UserProfileResponse,
    UpdateProfileRequest,
)
from ..auth_models import User
from ...utils.logging import get_logger

logger = get_logger(__name__)

router = APIRouter(prefix="/api/auth", tags=["auth"])

# Dependency to get database session
def get_db() -> Session:
    """Get database session."""
    from ..database import SessionLocal
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


def get_current_user(
    request: Request,
    db: Session = Depends(get_db),
) -> User:
    """Get current authenticated user from token."""
    token = None

    # Try to get token from Authorization header
    auth_header = request.headers.get("Authorization")
    if auth_header and auth_header.startswith("Bearer "):
        token = auth_header[7:]

    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Verify token
    payload = TokenService.verify_token(token)
    if not payload or payload.get("type") != "access":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user_id = payload.get("sub")
    auth_service = AuthService(db)
    user = auth_service.get_user_by_id(user_id)

    if not user or not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found or inactive",
        )

    return user


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    request: Request,
    signup_data: SignupRequest,
    db: Session = Depends(get_db),
):
    """Register a new user with background information.

    Returns user information and access/refresh tokens.
    """
    try:
        auth_service = AuthService(db)

        # Register user
        user = auth_service.register_user(
            email=signup_data.email,
            password=signup_data.password,
            first_name=signup_data.first_name,
            last_name=signup_data.last_name,
            software_background=signup_data.software_background,
            hardware_background=signup_data.hardware_background,
            programming_languages=signup_data.programming_languages,
            robotics_experience=signup_data.robotics_experience,
            ai_ml_experience=signup_data.ai_ml_experience,
        )

        if not user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Failed to register user",
            )

        # Generate tokens
        access_token = TokenService.create_access_token(user.id, user.email)
        refresh_token = TokenService.create_refresh_token(user.id, user.email)

        user_response = UserResponse(
            user_id=user.id,
            email=user.email,
            first_name=user.first_name,
            last_name=user.last_name,
            is_active=user.is_active,
            created_at=user.created_at,
        )

        tokens = TokenResponse(
            access_token=access_token,
            refresh_token=refresh_token,
            expires_in=15 * 60,  # 15 minutes in seconds
        )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(f"User signup successful: {user.email}", extra={"request_id": request_id})

        return AuthResponse(user=user_response, tokens=tokens)

    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error during signup: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during signup",
        )


@router.post("/signin", response_model=AuthResponse)
async def signin(
    request: Request,
    login_data: LoginRequest,
    db: Session = Depends(get_db),
):
    """Authenticate user and return tokens."""
    try:
        auth_service = AuthService(db)
        result = auth_service.authenticate_user(
            email=login_data.email,
            password=login_data.password,
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password",
            )

        user_response = UserResponse(
            user_id=result["user_id"],
            email=result["email"],
            first_name=result.get("first_name") or "",
            last_name=result.get("last_name") or "",
            is_active=True,
            created_at=datetime.utcnow(),
        )

        tokens = TokenResponse(
            access_token=result["access_token"],
            refresh_token=result["refresh_token"],
            expires_in=15 * 60,
        )

        return AuthResponse(user=user_response, tokens=tokens)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signin error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during signin",
        )


@router.post("/refresh", response_model=TokenResponse)
async def refresh_token(
    request: Request,
    refresh_data: RefreshTokenRequest,
    db: Session = Depends(get_db),
):
    """Refresh access token using refresh token.

    Returns a new access token.
    """
    try:
        # Verify refresh token and generate new access token
        new_access_token = TokenService.refresh_access_token(refresh_data.refresh_token)

        if not new_access_token:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid or expired refresh token",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info("Token refreshed successfully", extra={"request_id": request_id})

        return TokenResponse(
            access_token=new_access_token,
            refresh_token=refresh_data.refresh_token,
            expires_in=15 * 60,  # 15 minutes in seconds
        )

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error during token refresh: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error during token refresh",
        )


@router.post("/logout", status_code=status.HTTP_204_NO_CONTENT)
async def logout(
    request: Request,
    current_user: User = Depends(get_current_user),
):
    """Logout user.

    Note: Token blacklisting would require server-side state.
    For now, client should discard tokens.
    """
    request_id = getattr(request.state, "request_id", "unknown")
    logger.info(f"User logout: {current_user.email}", extra={"request_id": request_id})
    return None


@router.get("/me", response_model=UserProfileResponse)
async def get_current_user_profile(
    request: Request,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Get current authenticated user's profile."""
    try:
        auth_service = AuthService(db)
        profile = auth_service.get_user_profile(current_user.id)

        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User profile not found",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(f"User profile retrieved: {current_user.email}", extra={"request_id": request_id})

        return UserProfileResponse(**profile)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error fetching user profile: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error fetching profile",
        )


@router.put("/me/profile", response_model=UserProfileResponse)
async def update_current_user_profile(
    request: Request,
    profile_data: UpdateProfileRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """Update current user's profile."""
    try:
        auth_service = AuthService(db)

        # Filter out None values
        update_data = {k: v for k, v in profile_data.dict().items() if v is not None}

        updated_profile = auth_service.update_user_profile(current_user.id, **update_data)

        if not updated_profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User profile not found",
            )

        request_id = getattr(request.state, "request_id", "unknown")
        logger.info(f"User profile updated: {current_user.email}", extra={"request_id": request_id})

        return UserProfileResponse(**updated_profile)

    except HTTPException:
        raise
    except Exception as e:
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(f"Error updating user profile: {str(e)}", extra={"request_id": request_id})
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error updating profile",
        )
