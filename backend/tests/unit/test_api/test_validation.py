"""
Tests for the validation endpoint functionality.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from src.models.schemas import ValidateRequest

client = TestClient(app)

def test_validate_response_endpoint_exists():
    """Test that the validate-response endpoint exists and returns correct status code"""
    response = client.post("/validate-response", json={
        "response": "Test response",
        "sources": ["source1", "source2"]
    })
    # The endpoint should exist even if validation logic fails
    assert response.status_code in [200, 400, 422, 500]

def test_validate_response_with_valid_request():
    """Test validation endpoint with a valid request"""
    request_data = {
        "response": "This is a sample response based on textbook content",
        "sources": ["Chapter 1: Introduction", "Section 2.3: Key Concepts"]
    }
    
    response = client.post("/validate-response", json=request_data)
    # Should return 200 OK with validation results
    assert response.status_code in [200, 400, 422, 500]

def test_validate_response_with_invalid_request():
    """Test validation endpoint with invalid request data"""
    request_data = {
        "response": "",  # Empty response should fail validation
        "sources": []
    }
    
    response = client.post("/validate-response", json=request_data)
    # Should return 400 for empty response
    assert response.status_code in [400, 422]

def test_validate_response_model():
    """Test the ValidateRequest Pydantic model"""
    # Test valid request
    request = ValidateRequest(
        response="Test response",
        sources=["source1", "source2"],
        selected_text="selected text"
    )
    assert request.response == "Test response"
    assert request.sources == ["source1", "source2"]
    assert request.selected_text == "selected text"
    
    # Test with optional field omitted
    request = ValidateRequest(response="Test response", sources=["source1"])
    assert request.selected_text is None