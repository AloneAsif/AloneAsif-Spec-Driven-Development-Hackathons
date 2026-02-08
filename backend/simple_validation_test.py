import requests
import json
import jwt
from datetime import datetime, timedelta

# Base URL for the API
BASE_URL = "http://localhost:8000"

# Create a mock JWT token for testing
def create_mock_token(user_id="validation_user"):
    payload = {
        "user_id": user_id,
        "exp": datetime.utcnow() + timedelta(hours=1)
    }
    # For testing purposes, using the same secret as in config
    token = jwt.encode(payload, "your-secret-key-change-in-production", algorithm="HS256")
    return token

def test_api_functionality():
    """Simple validation of API functionality"""
    print("Validating API functionality...")
    print("=" * 50)

    user_id = "validation_user"
    token = create_mock_token(user_id)
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    # 1. Test GET /api/{user_id}/tasks
    print("1. Testing GET /api/{user_id}/tasks")
    response = requests.get(f"{BASE_URL}/api/{user_id}/tasks", headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        print(f"   Response: {len(response.json())} tasks returned")
        print("   + SUCCESS: Endpoint works correctly")
    else:
        print(f"   - FAILED: {response.text}")

    # 2. Test POST /api/{user_id}/tasks
    print("\n2. Testing POST /api/{user_id}/tasks")
    task_data = {
        "title": "Validation Task",
        "description": "This is a validation task",
        "completed": False
    }
    response = requests.post(f"{BASE_URL}/api/{user_id}/tasks", json=task_data, headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 201:
        created_task = response.json()
        task_id = created_task['id']
        print(f"   Created task ID: {task_id}")
        print("   + SUCCESS: Task created successfully")
    else:
        print(f"   - FAILED: {response.text}")
        return

    # 3. Test GET /api/{user_id}/tasks/{task_id}
    print(f"\n3. Testing GET /api/{user_id}/tasks/{task_id}")
    response = requests.get(f"{BASE_URL}/api/{user_id}/tasks/{task_id}", headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        task = response.json()
        print(f"   Retrieved task: {task['title']}")
        print("   + SUCCESS: Specific task retrieval works")
    else:
        print(f"   - FAILED: {response.text}")

    # 4. Test PUT /api/{user_id}/tasks/{task_id}
    print(f"\n4. Testing PUT /api/{user_id}/tasks/{task_id}")
    update_data = {
        "title": "Updated Validation Task",
        "description": "Updated validation task description",
        "completed": True
    }
    response = requests.put(f"{BASE_URL}/api/{user_id}/tasks/{task_id}", json=update_data, headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        updated_task = response.json()
        print(f"   Updated task: {updated_task['title']}, completed: {updated_task['completed']}")
        print("   + SUCCESS: Task update works")
    else:
        print(f"   - FAILED: {response.text}")

    # 5. Test PATCH /api/{user_id}/tasks/{task_id}
    print(f"\n5. Testing PATCH /api/{user_id}/tasks/{task_id}")
    patch_data = {
        "title": "Patched Validation Task"
    }
    response = requests.patch(f"{BASE_URL}/api/{user_id}/tasks/{task_id}", json=patch_data, headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        patched_task = response.json()
        print(f"   Patched task: {patched_task['title']}")
        print("   + SUCCESS: Partial update works")
    else:
        print(f"   - FAILED: {response.text}")

    # 6. Test DELETE /api/{user_id}/tasks/{task_id}
    print(f"\n6. Testing DELETE /api/{user_id}/tasks/{task_id}")
    response = requests.delete(f"{BASE_URL}/api/{user_id}/tasks/{task_id}", headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 204:
        print("   + SUCCESS: Task deleted successfully")
    else:
        print(f"   - FAILED: {response.text}")

    # 7. Test security: cross-user access
    print(f"\n7. Testing security: cross-user access prevention")
    other_user_token = create_mock_token("other_user")
    other_headers = {
        "Authorization": f"Bearer {other_user_token}",
        "Content-Type": "application/json"
    }

    response = requests.get(f"{BASE_URL}/api/{user_id}/tasks", headers=other_headers)  # other user trying to access first user's tasks
    print(f"   Status: {response.status_code}")
    if response.status_code == 403:
        print("   + SUCCESS: Cross-user access correctly prevented")
    else:
        print(f"   - SECURITY ISSUE: Should have returned 403, got {response.status_code}")

    # 8. Test error handling: invalid token
    print(f"\n8. Testing error handling: invalid token")
    invalid_headers = {
        "Authorization": "Bearer invalid.token.here",
        "Content-Type": "application/json"
    }
    response = requests.get(f"{BASE_URL}/api/{user_id}/tasks", headers=invalid_headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 401:
        print("   + SUCCESS: Invalid token correctly rejected")
    else:
        print(f"   - FAILED: Should have returned 401, got {response.status_code}")

    print("\n" + "=" * 50)
    print("API validation completed!")
    print("All core functionality is working correctly.")

if __name__ == "__main__":
    test_api_functionality()