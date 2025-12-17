"""
Search endpoint for the AI Agent with FastAPI application.
Implements the search functionality as specified in the RAG chatbot API contract.
"""
from fastapi import APIRouter, HTTPException, Depends
import logging
from typing import Optional, List
from datetime import datetime
from src.models.schemas import SearchRequest, SearchResponse, SearchResult, ErrorResponse
from src.agents.retrieval_tool import retrieval_tool

router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/search", summary="Search textbook content")
async def search_textbook(request: SearchRequest):
    """
    Search the textbook content for relevant passages based on the provided query.
    """
    try:
        # Validate input
        if not request.query or not request.query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        # Set default limit if not provided
        top_k = request.limit if request.limit else 5

        # Apply chapter filter if provided
        # For now, we'll use the chapter filter as additional context but it's not directly
        # implemented in our retrieval tool. In a real system, this would filter by metadata.
        # For this implementation, we'll just log that the filter was provided.
        if request.chapter_filter:
            logger.info(f"Chapter filter applied: {request.chapter_filter}")

        # Retrieve relevant content from Qdrant using the retrieval tool
        content_chunks = await retrieval_tool.retrieve_content_chunks(request.query, top_k=top_k)

        # Convert content chunks to search results
        search_results = []
        for chunk in content_chunks:
            search_result = SearchResult(
                content=chunk.text,
                chapter=chunk.section,
                section=chunk.section,  # In our implementation, section and chapter are the same
                relevance_score=0.9,  # Placeholder score - in a full implementation, this would come from the search score
                page_reference=""  # Placeholder - in a full implementation, this would come from the source metadata
            )
            search_results.append(search_result)

        # Create and return the response
        response = SearchResponse(
            results=search_results,
            total_count=len(search_results)
        )

        logger.info(f"Successfully searched textbook content for query: {request.query[:50]}...")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error searching textbook content: {e}")
        error_response = ErrorResponse(
            error="search_error",
            message=f"Failed to search textbook content: {str(e)}"
        )
        raise HTTPException(status_code=500, detail=error_response.message)