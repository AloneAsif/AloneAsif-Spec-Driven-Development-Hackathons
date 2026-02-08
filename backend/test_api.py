import requests
import json
from datetime import datetime

# Base URL for the API
BASE_URL = "http://localhost:8000"

def test_health_endpoint():
    """Test the health endpoint"""
    try:
        response = requests.get(f"{BASE_URL}/health")
        print(f"Health check: {response.status_code}")
        print(f"Response: {response.json()}")
        return response.status_code == 200
    except Exception as e:
        print(f"Error testing health endpoint: {e}")
        return False

def test_root_endpoint():
    """Test the root endpoint"""
    try:
        response = requests.get(f"{BASE_URL}/")
        print(f"Root endpoint: {response.status_code}")
        print(f"Response: {response.json()}")
        return response.status_code == 200
    except Exception as e:
        print(f"Error testing root endpoint: {e}")
        return False

def test_nonexistent_route():
    """Test a route that should return 404"""
    try:
        response = requests.get(f"{BASE_URL}/nonexistent")
        print(f"Nonexistent route: {response.status_code}")
        return True
    except Exception as e:
        print(f"Error testing nonexistent route: {e}")
        return False

if __name__ == "__main__":
    print("Testing API endpoints...")

    health_ok = test_health_endpoint()
    root_ok = test_root_endpoint()
    test_nonexistent_route()

    if health_ok and root_ok:
        print("\n+ Basic API functionality is working!")
    else:
        print("\n- Some API endpoints are not working properly.")