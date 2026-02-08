from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from datetime import timedelta
from sqlmodel import Session, select
from typing import Optional
import jwt
import bcrypt
from datetime import datetime, timezone
from pydantic import BaseModel

from models.task_models import User
from config import settings
from db import get_session

router = APIRouter(prefix="/auth", tags=["auth"])

security = HTTPBearer()


class SignupRequest(BaseModel):
    email: str
    name: str
    password: str


class SigninRequest(BaseModel):
    email: str
    password: str


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """Create JWT access token"""
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode.update({"exp": expire})

    # Ensure the token has the expected 'user_id' field for compatibility
    if 'sub' in to_encode and 'user_id' not in to_encode:
        to_encode['user_id'] = to_encode['sub']

    encoded_jwt = jwt.encode(to_encode, settings.SECRET_KEY, algorithm=settings.ALGORITHM)
    return encoded_jwt


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify plain password against hashed password"""
    return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))


def hash_password(password: str) -> str:
    """Hash password using bcrypt"""
    salt = bcrypt.gensalt()
    hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
    return hashed.decode('utf-8')


@router.post("/signup")
def signup_user(request: SignupRequest, session: Session = Depends(get_session)):
    """Register a new user"""
    # Check if user already exists
    existing_user = session.exec(select(User).where(User.email == request.email)).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Hash the password
    hashed_password = hash_password(request.password)

    # Create new user
    user = User(
        email=request.email,
        name=request.name,
        password_hash=hashed_password
    )

    session.add(user)
    session.commit()
    session.refresh(user)

    # Create access token
    access_token = create_access_token(data={"sub": user.id, "email": user.email})

    return {
        "user": {
            "id": user.id,
            "email": user.email,
            "name": user.name
        },
        "token": access_token
    }


@router.post("/signin")
def signin_user(request: SigninRequest, session: Session = Depends(get_session)):
    """Authenticate user and return token"""
    # Find user by email
    user = session.exec(select(User).where(User.email == request.email)).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # Verify password
    if not verify_password(request.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # Create access token
    access_token = create_access_token(data={"sub": user.id, "email": user.email})

    return {
        "user": {
            "id": user.id,
            "email": user.email,
            "name": user.name
        },
        "token": access_token
    }


@router.post("/signout")
def signout_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Sign out user (client-side token removal is sufficient)"""
    # In a real application, you might want to blacklist the token
    # For now, we just return a success response
    return {"message": "Successfully signed out"}


@router.get("/me")
def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security), session: Session = Depends(get_session)):
    """Get current user info from token"""
    try:
        payload = jwt.decode(credentials.credentials, settings.SECRET_KEY, algorithms=[settings.ALGORITHM])
        user_id: str = payload.get("sub")

        if user_id is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials"
            )

        user = session.get(User, user_id)
        if user is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found"
            )

        return {
            "id": user.id,
            "email": user.email,
            "name": user.name
        }
    except jwt.PyJWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials"
        )