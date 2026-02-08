from sqlmodel import create_engine, Session
from sqlalchemy import event
from sqlalchemy.pool import Pool
from config import settings
from typing import Generator

# Create the database engine
engine = create_engine(
    settings.DATABASE_URL,
    echo=False,  # Set to True for SQL debugging
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,  # Recycle connections every 5 minutes
)


def get_session() -> Generator[Session, None, None]:
    """
    Get a database session for dependency injection
    """
    with Session(engine) as session:
        yield session


# Optional: Add connection pooling configuration
@event.listens_for(engine, "connect")
def set_sqlite_pragma(dbapi_connection, connection_record):
    """
    Set SQLite pragmas for better performance and concurrency
    """
    if 'sqlite' in settings.DATABASE_URL:
        cursor = dbapi_connection.cursor()
        # Enable foreign key constraints
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.close()