from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class TaskBase(SQLModel):
    """Base model for Task containing shared attributes"""
    title: str = Field(min_length=1, max_length=255)
    description: Optional[str] = Field(default=None, max_length=1000)
    completed: bool = Field(default=False)
    user_id: str  # Foreign key linking to user account


class Task(TaskBase, table=True):
    """Task model for database table"""
    __tablename__ = "tasks"

    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: str = Field(index=True)  # Index for user isolation
    completed: bool = Field(default=False, index=True)  # Index for performance
    created_at: Optional[datetime] = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = Field(default_factory=datetime.utcnow)


class TaskCreate(SQLModel):
    """Model for creating a new task - user_id comes from the path parameter"""
    title: str = Field(min_length=1, max_length=255)
    description: Optional[str] = Field(default=None, max_length=1000)
    completed: bool = Field(default=False)


class TaskUpdate(SQLModel):
    """Model for updating an existing task"""
    title: Optional[str] = Field(default=None, min_length=1, max_length=255)
    description: Optional[str] = Field(default=None, max_length=1000)
    completed: Optional[bool] = None


class TaskResponse(TaskBase):
    """Model for task response to API clients"""
    id: int
    user_id: str
    created_at: datetime
    updated_at: datetime


class User(SQLModel, table=True):
    """User model for authentication"""
    __tablename__ = "users"

    id: Optional[str] = Field(default=None, primary_key=True, nullable=False)
    email: str = Field(unique=True, index=True, max_length=255)
    name: str = Field(max_length=255)
    password_hash: str = Field(max_length=255)
    created_at: Optional[datetime] = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = Field(default_factory=datetime.utcnow)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if self.id is None:
            import uuid
            self.id = str(uuid.uuid4())