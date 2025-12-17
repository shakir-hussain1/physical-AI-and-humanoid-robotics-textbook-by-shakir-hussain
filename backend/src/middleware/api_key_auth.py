"""Bearer token authentication middleware."""
from fastapi import HTTPException, Request, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
import os

# For MVP, using a simple API key approach
# In production, use proper API key hashing and database lookup
DEFAULT_API_KEY = os.getenv("DEFAULT_API_KEY", "sk-fake-test-key-for-mvp")

security = HTTPBearer(auto_error=False)

def validate_api_key(credentials: Optional[HTTPAuthorizationCredentials] = None) -> bool:
    """Validate the provided API key."""
    if not credentials:
        # For MVP, allow without API key in development
        return os.getenv("FASTAPI_ENV", "development") != "production"

    provided_key = credentials.credentials
    # In a real implementation, this would hash-verify against stored keys
    return provided_key == DEFAULT_API_KEY

async def api_key_auth_middleware(request: Request, call_next):
    """Middleware to authenticate requests with API key."""
    # Extract authorization header
    auth_header = request.headers.get("Authorization")

    is_valid = False
    if auth_header:
        # Handle Bearer token format
        if auth_header.startswith("Bearer "):
            token = auth_header[7:]  # Remove "Bearer " prefix
            is_valid = token == DEFAULT_API_KEY
        elif auth_header.startswith("ApiKey "):
            token = auth_header[7:]  # Remove "ApiKey " prefix
            is_valid = token == DEFAULT_API_KEY
        else:
            # For backward compatibility or simple key
            is_valid = auth_header == DEFAULT_API_KEY
    else:
        # For MVP, allow requests without API key in development
        is_valid = os.getenv("FASTAPI_ENV", "development") != "production"

    if not is_valid and os.getenv("FASTAPI_ENV", "development") == "production":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API Key",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Add API key info to request state for later use
    request.state.api_key_valid = is_valid
    request.state.api_key_used = bool(auth_header)

    response = await call_next(request)
    return response