"""
Query endpoint for the AI Agent with FastAPI application.
"""
from fastapi import APIRouter, HTTPException, Depends
import logging
from typing import Optional
from datetime import datetime
from src.models.schemas import QueryRequest, QueryResponse, ErrorResponse, SourceReference
from src.agents.ai_agent import openai_agent
from src.agents.retrieval_tool import retrieval_tool

router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/query", summary="Submit a query to the AI agent")
async def query_agent(request: QueryRequest):
    """
    Send a question to the AI agent and receive a response based on book content.
    """
    try:
        # Validate input
        if not request.query or not request.query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        # Use provided thread_id or create a new one
        thread_id = request.thread_id
        if not thread_id:
            thread_id = await openai_agent.create_thread()

        # Retrieve relevant content from Qdrant using the retrieval tool
        context_sources: Optional[List[SourceReference]] = None
        try:
            context_sources = await retrieval_tool.retrieve_content(request.query, top_k=5)
            logger.info(f"Retrieved {len(context_sources)} sources for query: {request.query[:50]}...")
        except Exception as e:
            logger.warning(f"Could not retrieve context for query: {e}")
            # Continue without context sources

        # Process the query with the AI agent using the retrieved context
        response_text = await openai_agent.process_query_with_context(
            query=request.query,
            thread_id=thread_id,
            context_sources=context_sources
        )

        # Create and return the response
        response = QueryResponse(
            response=response_text,
            sources=context_sources,
            thread_id=thread_id,
            timestamp=datetime.utcnow()
        )

        logger.info(f"Successfully processed query for thread: {thread_id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        error_response = ErrorResponse(
            error="query_processing_error",
            message=f"Failed to process query: {str(e)}"
        )
        raise HTTPException(status_code=500, detail=error_response.message)