"""Authentication endpoints - Signup, Signin, Profile."""

from fastapi import APIRouter, HTTPException, Depends, Header
from pydantic import BaseModel, EmailStr, validator
from typing import Optional, List
import logging

from src.services.auth_service import get_auth_service
from src.services.personalization_service import get_personalization_service

logger = logging.getLogger(__name__)
router = APIRouter()

# Models
class SignupRequest(BaseModel):
    """Signup request model."""
    email: EmailStr
    username: str
    password: str
    full_name: Optional[str] = None
    confirm_password: str

    @validator('username')
    def username_valid(cls, v):
        if len(v) < 3:
            raise ValueError('Username must be at least 3 characters')
        if not v.isalnum() or ' ' in v:
            raise ValueError('Username must be alphanumeric (no spaces)')
        return v

    @validator('password')
    def password_strong(cls, v):
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters')
        if not any(c.isupper() for c in v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not any(c.isdigit() for c in v):
            raise ValueError('Password must contain at least one number')
        return v

    @validator('confirm_password')
    def passwords_match(cls, v, values):
        if 'password' in values and v != values['password']:
            raise ValueError('Passwords do not match')
        return v

class BackgroundProfile(BaseModel):
    """User background profile for personalization."""
    software_background: str  # beginner, intermediate, advanced, expert
    hardware_background: str  # beginner, intermediate, advanced, expert
    programming_languages: List[str]
    interest_areas: List[str]  # ROS, Kinematics, Control, Perception, Humanoid, etc.
    profession: Optional[str] = None
    organization: Optional[str] = None
    years_of_experience: Optional[int] = None
    preferred_learning_style: str = "text"  # visual, text, interactive, hands-on
    learning_pace: str = "moderate"  # slow, moderate, fast

class SigninRequest(BaseModel):
    """Signin request model."""
    email: str
    password: str

class TokenResponse(BaseModel):
    """Token response model."""
    access_token: str
    refresh_token: str
    token_type: str
    user: dict

class UserResponse(BaseModel):
    """User response model."""
    id: int
    email: str
    username: str
    full_name: Optional[str]
    software_background: str
    hardware_background: str
    programming_languages: List[str]
    interest_areas: List[str]
    profession: Optional[str]
    organization: Optional[str]
    years_of_experience: Optional[int]
    preferred_learning_style: str
    learning_pace: str

# In-memory user storage (replace with database in production)
users_db = {}  # {email: {user_data}}
next_user_id = 1

@router.post('/signup', response_model=dict)
async def signup(request: SignupRequest):
    """
    Register a new user.

    - **email**: User's email address
    - **username**: Unique username
    - **password**: Strong password (8+ chars, uppercase, number)
    - **full_name**: Optional full name
    """
    try:
        # Check if user already exists
        if request.email in users_db:
            raise HTTPException(status_code=400, detail='Email already registered')

        # Hash password
        auth_service = get_auth_service()
        hashed_password = auth_service.hash_password(request.password)

        # Create user
        global next_user_id
        user = {
            'id': next_user_id,
            'email': request.email,
            'username': request.username,
            'full_name': request.full_name,
            'hashed_password': hashed_password,
            'created_at': str(__import__('datetime').datetime.utcnow()),
            # Default profile
            'software_background': 'beginner',
            'hardware_background': 'beginner',
            'programming_languages': [],
            'interest_areas': [],
            'profession': None,
            'organization': None,
            'years_of_experience': None,
            'preferred_learning_style': 'text',
            'learning_pace': 'moderate'
        }

        users_db[request.email] = user
        next_user_id += 1

        return {
            'status': 'success',
            'message': 'Account created successfully. Please complete your profile.',
            'user_id': user['id'],
            'email': user['email']
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f'Signup error: {e}')
        raise HTTPException(status_code=500, detail=str(e))

@router.post('/signin', response_model=TokenResponse)
async def signin(request: SigninRequest):
    """
    Sign in with email and password.

    - **email**: User's email address
    - **password**: User's password

    Returns access and refresh tokens.
    """
    try:
        # Find user
        user = users_db.get(request.email)
        if not user:
            raise HTTPException(status_code=401, detail='Invalid email or password')

        # Verify password
        auth_service = get_auth_service()
        if not auth_service.verify_password(request.password, user['hashed_password']):
            raise HTTPException(status_code=401, detail='Invalid email or password')

        # Create tokens
        tokens = auth_service.create_tokens(user['id'], user['email'], user['username'])

        user_response = {
            'id': user['id'],
            'email': user['email'],
            'username': user['username'],
            'full_name': user.get('full_name'),
            'software_background': user.get('software_background'),
            'hardware_background': user.get('hardware_background'),
            'programming_languages': user.get('programming_languages', []),
            'interest_areas': user.get('interest_areas', [])
        }

        return TokenResponse(
            access_token=tokens['access_token'],
            refresh_token=tokens['refresh_token'],
            token_type=tokens['token_type'],
            user=user_response
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f'Signin error: {e}')
        raise HTTPException(status_code=500, detail=str(e))

@router.post('/profile/background')
async def save_background_profile(
    profile: BackgroundProfile,
    authorization: str = Header(None)
):
    """
    Save user's background profile for personalization.

    Expects Bearer token in Authorization header.
    """
    try:
        if not authorization or not authorization.startswith('Bearer '):
            raise HTTPException(status_code=401, detail='Missing or invalid token')

        token = authorization.split(' ')[1]
        auth_service = get_auth_service()
        payload = auth_service.verify_token(token)

        if not payload:
            raise HTTPException(status_code=401, detail='Invalid or expired token')

        # Find and update user
        user_email = payload.get('email')
        if user_email not in users_db:
            raise HTTPException(status_code=404, detail='User not found')

        user = users_db[user_email]
        user.update({
            'software_background': profile.software_background,
            'hardware_background': profile.hardware_background,
            'programming_languages': profile.programming_languages,
            'interest_areas': profile.interest_areas,
            'profession': profile.profession,
            'organization': profile.organization,
            'years_of_experience': profile.years_of_experience,
            'preferred_learning_style': profile.preferred_learning_style,
            'learning_pace': profile.learning_pace
        })

        return {
            'status': 'success',
            'message': 'Profile saved successfully',
            'user': user
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f'Profile save error: {e}')
        raise HTTPException(status_code=500, detail=str(e))

@router.get('/profile', response_model=UserResponse)
async def get_user_profile(authorization: str = Header(None)):
    """
    Get current user's profile.

    Expects Bearer token in Authorization header.
    """
    try:
        if not authorization or not authorization.startswith('Bearer '):
            raise HTTPException(status_code=401, detail='Missing or invalid token')

        token = authorization.split(' ')[1]
        auth_service = get_auth_service()
        payload = auth_service.verify_token(token)

        if not payload:
            raise HTTPException(status_code=401, detail='Invalid or expired token')

        user_email = payload.get('email')
        if user_email not in users_db:
            raise HTTPException(status_code=404, detail='User not found')

        user = users_db[user_email]
        return UserResponse(**{
            'id': user['id'],
            'email': user['email'],
            'username': user['username'],
            'full_name': user.get('full_name'),
            'software_background': user.get('software_background'),
            'hardware_background': user.get('hardware_background'),
            'programming_languages': user.get('programming_languages', []),
            'interest_areas': user.get('interest_areas', []),
            'profession': user.get('profession'),
            'organization': user.get('organization'),
            'years_of_experience': user.get('years_of_experience'),
            'preferred_learning_style': user.get('preferred_learning_style'),
            'learning_pace': user.get('learning_pace')
        })

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f'Profile fetch error: {e}')
        raise HTTPException(status_code=500, detail=str(e))

@router.get('/personalization')
async def get_personalization_data(authorization: str = Header(None)):
    """
    Get personalized content recommendations based on user profile.

    Expects Bearer token in Authorization header.
    """
    try:
        if not authorization or not authorization.startswith('Bearer '):
            raise HTTPException(status_code=401, detail='Missing or invalid token')

        token = authorization.split(' ')[1]
        auth_service = get_auth_service()
        payload = auth_service.verify_token(token)

        if not payload:
            raise HTTPException(status_code=401, detail='Invalid or expired token')

        user_email = payload.get('email')
        if user_email not in users_db:
            raise HTTPException(status_code=404, detail='User not found')

        user = users_db[user_email]
        personalization_service = get_personalization_service()
        personalization_data = personalization_service.get_personalization_context(user)

        return {
            'status': 'success',
            'data': personalization_data
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f'Personalization fetch error: {e}')
        raise HTTPException(status_code=500, detail=str(e))
