from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from typing import Optional
import jwt
from jwt.exceptions import InvalidTokenError
from config import settings
from models.task_models import Task
import datetime

# Define security scheme
security = HTTPBearer()

# Define token data model
class TokenData(BaseModel):
    user_id: Optional[str] = None


def verify_token(token: str) -> dict:
    """
    Verify JWT token and return decoded data
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        payload = jwt.decode(
            token,
            settings.SECRET_KEY,
            algorithms=[settings.ALGORITHM]
        )

        # Extract user_id from token
        user_id: str = payload.get("user_id")
        if user_id is None:
            raise credentials_exception

        token_data = TokenData(user_id=user_id)
        return {"user_id": token_data.user_id}

    except InvalidTokenError:
        raise credentials_exception


async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    Get current user from JWT token
    """
    token_data = verify_token(credentials.credentials)
    return token_data


def verify_user_access(token_user_id: str, requested_user_id: str) -> bool:
    """
    Verify that the authenticated user can access resources for the requested user_id
    This enforces user isolation - users can only access their own data
    """
    return token_user_id == requested_user_id


def create_access_token(data: dict, expires_delta: Optional[datetime.timedelta] = None):
    """
    Create access token with expiration
    """
    to_encode = data.copy()

    if expires_delta:
        expire = datetime.datetime.utcnow() + expires_delta
    else:
        expire = datetime.datetime.utcnow() + datetime.timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode.update({"exp": expire})

    encoded_jwt = jwt.encode(to_encode, settings.SECRET_KEY, algorithm=settings.ALGORITHM)
    return encoded_jwt