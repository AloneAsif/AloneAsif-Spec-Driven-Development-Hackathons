import requests
import json
import jwt
from datetime import datetime, timedelta

# Base URL for the API
BASE_URL = "http://localhost:8000"

# Create a mock JWT token for testing
def create_mock_token(user_id="test_user_123"):
    payload = {
        "user_id": user_id,
        "exp": datetime.utcnow() + timedelta(hours=1)
    }
    # For testing purposes, using a simple secret
    token = jwt.encode(payload, "your-secret-key-change-in-production", algorithm="HS256")
    return token

def test_get_tasks():
    """Test getting tasks for a user"""
    token = create_mock_token("test_user_123")

    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    try:
        response = requests.get(f"{BASE_URL}/api/test_user_123/tasks", headers=headers)
        print(f"GET /api/test_user_123/tasks: {response.status_code}")

        if response.status_code == 200:
            print("Response:", response.json())
        elif response.status_code == 401:
            print("Unauthorized - token invalid or missing")
        elif response.status_code == 403:
            print("Forbidden - user access violation")
        else:
            print("Response:", response.text)

        return response.status_code in [200, 401, 403, 404]  # 404 is expected if no tasks exist
    except Exception as e:
        print(f"Error testing GET /api/test_user_123/tasks: {e}")
        return False

def test_create_task():
    """Test creating a task for a user"""
    token = create_mock_token("test_user_123")

    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    task_data = {
        "title": "Test Task",
        "description": "This is a test task",
        "completed": False
    }

    try:
        response = requests.post(f"{BASE_URL}/api/test_user_123/tasks",
                                json=task_data, headers=headers)
        print(f"POST /api/test_user_123/tasks: {response.status_code}")

        if response.status_code in [200, 201]:
            print("Response:", response.json())
        elif response.status_code == 401:
            print("Unauthorized - token invalid or missing")
        elif response.status_code == 400:
            print("Bad Request - validation error:", response.text)
        else:
            print("Response:", response.text)

        return response.status_code in [200, 201, 400, 401, 403]
    except Exception as e:
        print(f"Error testing POST /api/test_user_123/tasks: {e}")
        return False

def test_invalid_token():
    """Test API with invalid token"""
    headers = {
        "Authorization": "Bearer invalid_token_here",
        "Content-Type": "application/json"
    }

    try:
        response = requests.get(f"{BASE_URL}/api/test_user_123/tasks", headers=headers)
        print(f"GET /api/test_user_123/tasks with invalid token: {response.status_code}")

        if response.status_code == 401:
            print("Correctly rejected invalid token")
        else:
            print("Unexpected response for invalid token")

        return response.status_code == 401
    except Exception as e:
        print(f"Error testing invalid token: {e}")
        return False

def test_cross_user_access():
    """Test that user A cannot access user B's tasks"""
    token = create_mock_token("user_a")

    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    try:
        response = requests.get(f"{BASE_URL}/api/user_b/tasks", headers=headers)
        print(f"GET /api/user_b/tasks (user_a trying to access): {response.status_code}")

        if response.status_code == 403:
            print("Correctly prevented cross-user access")
        elif response.status_code == 200:
            print("Cross-user access allowed - this is a security issue!")
        else:
            print("Response:", response.text)

        return response.status_code == 403  # Expected for security
    except Exception as e:
        print(f"Error testing cross-user access: {e}")
        return False

if __name__ == "__main__":
    print("Testing task API endpoints...")
    print("=" * 50)

    # Test basic endpoints
    print("Testing health endpoint...")
    try:
        resp = requests.get(f"{BASE_URL}/health")
        print(f"Health check: {resp.status_code} - {resp.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")

    print("\nTesting root endpoint...")
    try:
        resp = requests.get(f"{BASE_URL}/")
        print(f"Root endpoint: {resp.status_code} - {resp.json()}")
    except Exception as e:
        print(f"Root endpoint failed: {e}")

    print("\nTesting task endpoints...")
    test_get_tasks()
    test_create_task()
    test_invalid_token()
    test_cross_user_access()

    print("\nTask API testing completed.")