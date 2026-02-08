from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from routes import tasks, auth

# Create FastAPI app instance
app = FastAPI(
    title="Task Management API",
    description="RESTful API for managing user tasks with JWT authentication",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(tasks.router, prefix="/api")
app.include_router(auth.router, prefix="/api")

@app.get("/")
def read_root():
    return {"message": "Welcome to the Task Management API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}