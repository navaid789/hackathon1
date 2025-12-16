"""
Unit tests for the Qdrant service.
"""
import pytest
from unittest.mock import Mock, patch
from src.services.qdrant_service import QdrantService
from src.models.schemas import ContentChunk


@pytest.mark.asyncio
async def test_qdrant_service_initialization():
    """Test that Qdrant service can be initialized."""
    service = QdrantService()
    assert service is not None
    assert service.client is not None
    assert service.collection_name is not None


@pytest.mark.asyncio
@patch('qdrant_client.qdrant_client.QdrantClient.query_points')
async def test_search_similar(mock_query_points):
    """Test the search_similar method with mocked query_points."""
    # Configure mock
    mock_result = Mock()
    mock_result.payload = {
        'text': 'test content',
        'url': 'https://example.com',
        'section': 'Chapter 1'
    }
    mock_result.vector = [0.1] * 1024  # Mock embedding
    mock_response = Mock()
    mock_response.points = [mock_result]
    mock_query_points.return_value = mock_response

    service = QdrantService()
    results = await service.search_similar([0.1] * 1536, top_k=1)

    assert len(results) == 1
    assert isinstance(results[0], ContentChunk)
    assert results[0].text == 'test content'
    assert results[0].url == 'https://example.com'
    assert results[0].section == 'Chapter 1'