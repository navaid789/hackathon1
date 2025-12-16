"""
Cohere service for embedding generation in the AI Agent with FastAPI application.
"""
import logging
import cohere
from typing import List
from src.config.settings import settings

logger = logging.getLogger(__name__)


class CohereService:
    """
    Service class for interacting with Cohere API for embedding generation
    """

    def __init__(self):
        """
        Initialize Cohere client with configuration from settings
        """
        self.client = cohere.Client(settings.cohere_api_key)

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        try:
            # Generate embeddings using Cohere
            response = self.client.embed(
                texts=texts,
                model="embed-english-v3.0",  # Using the same model as in the backend
                input_type="search_document"  # Specify input type for better results
            )

            # Extract embeddings from response
            embeddings = response.embeddings
            logger.info(f"Generated embeddings for {len(texts)} texts")
            return embeddings

        except Exception as e:
            logger.error(f"Error generating embeddings with Cohere: {e}")
            raise

    async def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text

        Args:
            text: Text to generate embedding for

        Returns:
            Embedding vector as a list of floats
        """
        try:
            embeddings = await self.generate_embeddings([text])
            return embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            logger.error(f"Error generating single embedding: {e}")
            raise

    async def validate_connection(self) -> bool:
        """
        Validate connection to Cohere by generating a test embedding

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Generate a test embedding to validate connection
            test_embedding = await self.generate_single_embedding("test")
            return len(test_embedding) > 0
        except Exception as e:
            logger.error(f"Cohere connection validation failed: {e}")
            return False


# Global instance of CohereService
cohere_service = CohereService()