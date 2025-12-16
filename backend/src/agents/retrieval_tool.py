"""
Qdrant retrieval tool for the AI Agent with FastAPI application.
"""
import logging
from typing import List, Dict, Any
from src.services.qdrant_service import qdrant_service
from src.services.cohere_service import cohere_service
from src.models.schemas import SourceReference, ContentChunk

logger = logging.getLogger(__name__)


class RetrievalTool:
    """
    Tool for retrieving relevant content from Qdrant vector database
    """

    async def retrieve_content(self, query: str, top_k: int = 5) -> List[SourceReference]:
        """
        Retrieve relevant content chunks from Qdrant based on a query

        Args:
            query: Query text to search for
            top_k: Number of results to return (default: 5)

        Returns:
            List of SourceReference objects containing relevant content
        """
        try:
            # Generate embedding for the query using Cohere
            query_embedding = await cohere_service.generate_single_embedding(query)

            # Search for similar content in Qdrant
            content_chunks = await qdrant_service.search_similar(
                query_vector=query_embedding,
                top_k=top_k
            )

            # Convert content chunks to source references
            source_references = []
            for chunk in content_chunks:
                source_ref = SourceReference(
                    url=chunk.url,
                    section=chunk.section,
                    text=chunk.text,
                    relevance_score=1.0  # For now, set to 1.0; in a real implementation, this would come from search score
                )
                source_references.append(source_ref)

            logger.info(f"Retrieved {len(source_references)} source references for query: {query[:50]}...")
            return source_references

        except Exception as e:
            logger.error(f"Error retrieving content: {e}")
            raise

    async def retrieve_content_chunks(self, query: str, top_k: int = 5) -> List[ContentChunk]:
        """
        Retrieve relevant content chunks from Qdrant based on a query

        Args:
            query: Query text to search for
            top_k: Number of results to return (default: 5)

        Returns:
            List of ContentChunk objects containing relevant content
        """
        try:
            # Generate embedding for the query using Cohere
            query_embedding = await cohere_service.generate_single_embedding(query)

            # Search for similar content in Qdrant
            content_chunks = await qdrant_service.search_similar(
                query_vector=query_embedding,
                top_k=top_k
            )

            logger.info(f"Retrieved {len(content_chunks)} content chunks for query: {query[:50]}...")
            return content_chunks

        except Exception as e:
            logger.error(f"Error retrieving content chunks: {e}")
            raise

    async def validate_tool(self) -> bool:
        """
        Validate that the retrieval tool is working correctly

        Returns:
            True if validation is successful, False otherwise
        """
        try:
            # Test with a simple query
            test_sources = await self.retrieve_content("test", top_k=1)
            return len(test_sources) >= 0  # Should return at least an empty list
        except Exception as e:
            logger.error(f"Retrieval tool validation failed: {e}")
            return False


# Global instance of RetrievalTool
retrieval_tool = RetrievalTool()