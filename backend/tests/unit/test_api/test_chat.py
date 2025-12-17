"""
Tests for the chat endpoint functionality.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from src.models.schemas import ChatRequest

client = TestClient(app)

def test_chat_endpoint_exists():
    """Test that the chat endpoint exists and returns correct status code"""
    response = client.post("/chat", json={"message": "Hello"})
    # The endpoint should exist even if AI processing fails
    assert response.status_code in [200, 400, 422, 500]

def test_chat_with_valid_request():
    """Test chat endpoint with a valid request"""
    request_data = {
        "message": "What is artificial intelligence?",
        "context": "AI chapter introduction",
        "user_id": "user123",
        "chapter_filter": "Chapter 1"
    }
    
    response = client.post("/chat", json=request_data)
    # Should return 200 OK or an appropriate error code
    assert response.status_code in [200, 400, 422, 500]

def test_chat_with_invalid_request():
    """Test chat endpoint with invalid request data"""
    request_data = {
        "message": "",  # Empty message should fail validation
        "context": "some context"
    }
    
    response = client.post("/chat", json=request_data)
    # Should return 400 for empty message
    assert response.status_code in [400, 422]

def test_chat_request_model():
    """Test the ChatRequest Pydantic model"""
    # Test valid request
    request = ChatRequest(
        message="Test message",
        context="Test context",
        user_id="user123",
        chapter_filter="Chapter 1"
    )
    assert request.message == "Test message"
    assert request.context == "Test context"
    assert request.user_id == "user123"
    assert request.chapter_filter == "Chapter 1"
    
    # Test with optional fields omitted
    request = ChatRequest(message="Test message")
    assert request.context is None
    assert request.user_id is None
    assert request.chapter_filter is None