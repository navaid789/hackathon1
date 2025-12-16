"""
Unit tests for the Cohere service.
"""
import pytest
from unittest.mock import Mock, patch
from src.services.cohere_service import CohereService


@pytest.mark.asyncio
async def test_cohere_service_initialization():
    """Test that Cohere service can be initialized."""
    service = CohereService()
    assert service is not None
    assert service.client is not None


@pytest.mark.asyncio
@patch('cohere.Client.embed')
async def test_generate_embeddings(mock_embed):
    """Test the generate_embeddings method with mocked embedding."""
    # Configure mock
    mock_response = Mock()
    mock_response.embeddings = [[0.1, 0.2, 0.3]]
    mock_embed.return_value = mock_response

    service = CohereService()
    results = await service.generate_embeddings(["test text"])

    assert len(results) == 1
    assert len(results[0]) == 3  # 3-dimensional mock embedding