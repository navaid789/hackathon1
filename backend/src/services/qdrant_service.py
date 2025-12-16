"""
Qdrant service for vector database operations in the AI Agent with FastAPI application.
"""
import logging
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from src.config.settings import settings
from src.models.schemas import ContentChunk, RetrievalResult

logger = logging.getLogger(__name__)


class QdrantService:
    """
    Service class for interacting with Qdrant vector database
    """

    def __init__(self):
        """
        Initialize Qdrant client with configuration from settings
        """
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=30
        )
        self.collection_name = settings.qdrant_collection_name

    async def search_similar(
        self,
        query_vector: List[float],
        top_k: int = 5
    ) -> List[ContentChunk]:
        """
        Search for similar content chunks in the Qdrant collection

        Args:
            query_vector: The embedding vector to search for similar items
            top_k: Number of results to return (default: 5)

        Returns:
            List of ContentChunk objects containing similar content
        """
        try:
            # Perform similarity search using query_points (new API)
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                with_payload=True,
                with_vectors=True
            ).points

            # Convert search results to ContentChunk objects
            content_chunks = []
            for result in search_results:
                payload = result.payload
                content_chunk = ContentChunk(
                    text=payload.get('text', ''),
                    url=payload.get('url', ''),
                    section=payload.get('section', ''),
                    embedding=result.vector if result.vector else query_vector,  # Use result vector if available
                    metadata=payload.get('metadata')
                )
                content_chunks.append(content_chunk)

            logger.info(f"Found {len(content_chunks)} similar content chunks")
            return content_chunks

        except Exception as e:
            logger.error(f"Error searching Qdrant: {e}")
            raise

    async def get_collection_info(self) -> dict:
        """
        Get information about the collection

        Returns:
            Dictionary containing collection information
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors_count,
                "vector_count": collection_info.points_count,
                "config": collection_info.config.dict() if hasattr(collection_info.config, 'dict') else str(collection_info.config)
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            raise

    async def validate_connection(self) -> bool:
        """
        Validate connection to Qdrant

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Try to get collection info to validate connection
            await self.get_collection_info()
            return True
        except Exception as e:
            logger.error(f"Qdrant connection validation failed: {e}")
            return False


# Global instance of QdrantService
qdrant_service = QdrantService()