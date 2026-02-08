from pydantic_settings import BaseSettings
from typing import Optional
import os

class Settings(BaseSettings):
    """
    Application settings using Pydantic BaseSettings
    """
    # Database settings
    DATABASE_URL: str = os.getenv("DATABASE_URL", "sqlite:///./task_management.db")

    # JWT settings
    SECRET_KEY: str = os.getenv("SECRET_KEY", "your-secret-key-change-in-production")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # Better Auth settings
    BETTER_AUTH_SECRET: str = os.getenv("BETTER_AUTH_SECRET", "your-better-auth-secret")

    class Config:
        env_file = ".env"

# Create settings instance
settings = Settings()