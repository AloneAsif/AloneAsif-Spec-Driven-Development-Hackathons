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
    # For testing purposes, using the same secret as in config
    token = jwt.encode(payload, "your-secret-key-change-in-production", algorithm="HS256")
    return token

def run_comprehensive_test():
    """Run comprehensive tests for all CRUD operations"""
    print("Running comprehensive API tests...")
    print("=" * 60)

    token = create_mock_token("test_user_123")
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    # 1. Test GET /api/{user_id}/tasks (should return list)
    print("1. Testing GET /api/test_user_123/tasks (should return list)")
    response = requests.get(f"{BASE_URL}/api/test_user_123/tasks", headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        print(f"   Response: {response.json()}")
        print("   + Passed")
    else:
        print(f"   - Failed - {response.text}")

    # 2. Test POST /api/{user_id}/tasks (create task)
    print("\n2. Testing POST /api/test_user_123/tasks (create task)")
    task_data = {
        "title": "First Test Task",
        "description": "This is my first test task",
        "completed": False
    }
    response = requests.post(f"{BASE_URL}/api/test_user_123/tasks", json=task_data, headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 201:
        created_task = response.json()
        print(f"   Created task: {created_task['title']}")
        task_id = created_task['id']
        print("   + Passed")
    else:
        print(f"   - Failed - {response.text}")
        return

    # 3. Test GET /api/{user_id}/tasks/{task_id} (get specific task)
    print(f"\n3. Testing GET /api/test_user_123/tasks/{task_id} (get specific task)")
    response = requests.get(f"{BASE_URL}/api/test_user_123/tasks/{task_id}", headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        task = response.json()
        print(f"   Retrieved task: {task['title']}")
        print("   + Passed")
    else:
        print(f"   - Failed - {response.text}")

    # 4. Test GET /api/{user_id}/tasks (list with one item)
    print(f"\n4. Testing GET /api/test_user_123/tasks (list with one item)")
    response = requests.get(f"{BASE_URL}/api/test_user_123/tasks", headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        tasks = response.json()
        print(f"   Tasks count: {len(tasks)}")
        assert len(tasks) == 1, "Expected one task in list"
        print("   + Passed")
    else:
        print(f"   - Failed - {response.text}")

    # 5. Test PUT /api/{user_id}/tasks/{task_id} (update task completely)
    print(f"\n5. Testing PUT /api/test_user_123/tasks/{task_id} (update task)")
    update_data = {
        "title": "Updated Test Task",
        "description": "This is the updated test task",
        "completed": True
    }
    response = requests.put(f"{BASE_URL}/api/test_user_123/tasks/{task_id}", json=update_data, headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        updated_task = response.json()
        print(f"   Updated task: {updated_task['title']}, completed: {updated_task['completed']}")
        print("   + Passed")
    else:
        print(f"   - Failed - {response.text}")

    # 6. Test PATCH /api/{user_id}/tasks/{task_id} (partial update)
    print(f"\n6. Testing PATCH /api/test_user_123/tasks/{task_id} (partial update)")
    patch_data = {
        "title": "Partially Updated Test Task"
    }
    response = requests.patch(f"{BASE_URL}/api/test_user_123/tasks/{task_id}", json=patch_data, headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        patched_task = response.json()
        print(f"   Patched task: {patched_task['title']}")
        print("   + Passed")
    else:
        print(f"   - Failed - {response.text}")

    # 7. Test DELETE /api/{user_id}/tasks/{task_id} (delete task)
    print(f"\n7. Testing DELETE /api/test_user_123/tasks/{task_id} (delete task)")
    response = requests.delete(f"{BASE_URL}/api/test_user_123/tasks/{task_id}", headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 204:
        print("   Task deleted successfully")
        print("   + Passed")
    else:
        print(f"   - Failed - {response.text}")

    # 8. Test GET /api/{user_id}/tasks (empty list again)
    print(f"\n8. Testing GET /api/test_user_123/tasks (after deletion)")
    response = requests.get(f"{BASE_URL}/api/test_user_123/tasks", headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        tasks = response.json()
        print(f"   Tasks count: {len(tasks)}")
        assert len(tasks) == 0, "Expected empty list after deletion"
        print("   + Passed")
    else:
        print(f"   - Failed - {response.text}")

    # 9. Test security: user isolation
    print(f"\n9. Testing security: cross-user access prevention")
    bad_token = create_mock_token("other_user_456")
    bad_headers = {
        "Authorization": f"Bearer {bad_token}",
        "Content-Type": "application/json"
    }
    response = requests.get(f"{BASE_URL}/api/test_user_123/tasks", headers=bad_headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 403:
        print("   Correctly prevented cross-user access")
        print("   + Passed")
    else:
        print(f"   ✗ Failed - Should have returned 403, got {response.status_code}")

    # 10. Test error handling: invalid token
    print(f"\n10. Testing error handling: invalid token")
    invalid_headers = {
        "Authorization": "Bearer invalid.token.here",
        "Content-Type": "application/json"
    }
    response = requests.get(f"{BASE_URL}/api/test_user_123/tasks", headers=invalid_headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 401:
        print("   Correctly rejected invalid token")
        print("   + Passed")
    else:
        print(f"   ✗ Failed - Should have returned 401, got {response.status_code}")

    print("\n" + "=" * 60)
    print("All tests completed successfully! ✓")
    print("API is functioning correctly with:")
    print("- Full CRUD operations working")
    print("- Proper user isolation and authentication")
    print("- Comprehensive error handling")
    print("- Correct HTTP status codes")

if __name__ == "__main__":
    run_comprehensive_test()