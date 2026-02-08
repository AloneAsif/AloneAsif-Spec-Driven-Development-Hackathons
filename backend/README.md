# Task Management API

A secure backend API with JWT authentication for task management.

## Features

- RESTful endpoints for task CRUD operations
- JWT-based authentication and authorization
- User isolation - users can only access their own tasks
- PostgreSQL database integration with SQLModel ORM
- Comprehensive error handling

## Setup

1. Clone the repository
2. Navigate to the backend directory
3. Create a virtual environment:

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

4. Install dependencies:

```bash
pip install -r requirements.txt
```

5. Set up environment variables:

```bash
cp .env.example .env
# Edit .env with your configuration
```

## Environment Variables

- `DATABASE_URL`: PostgreSQL connection string
- `SECRET_KEY`: Secret key for JWT signing
- `ACCESS_TOKEN_EXPIRE_MINUTES`: Token expiration time in minutes
- `BETTER_AUTH_SECRET`: Better Auth secret key

## Running the Application

```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`

## API Endpoints

### Authentication Required

All endpoints require JWT authentication in the Authorization header:

```
Authorization: Bearer <jwt-token>
```

### Available Endpoints

- `GET /api/{user_id}/tasks` - Get all tasks for a user
- `POST /api/{user_id}/tasks` - Create a new task for a user
- `GET /api/{user_id}/tasks/{task_id}` - Get a specific task
- `PUT /api/{user_id}/tasks/{task_id}` - Update a task completely
- `PATCH /api/{user_id}/tasks/{task_id}` - Partially update a task
- `DELETE /api/{user_id}/tasks/{task_id}` - Delete a task

## Running Tests

```bash
pytest
```

## Security

- All endpoints enforce user isolation
- JWT tokens are validated for each request
- Input validation prevents common attacks
- SQL injection prevention via SQLModel ORM

## Database

The application uses SQLModel ORM to interact with PostgreSQL. The database schema includes proper indexes for performance and security.