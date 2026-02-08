from sqlmodel import SQLModel
from models.task_models import Task, User
from db import engine

def create_db_and_tables():
    """Create database tables"""
    SQLModel.metadata.create_all(engine)

if __name__ == "__main__":
    create_db_and_tables()
    print("Database tables created successfully!")