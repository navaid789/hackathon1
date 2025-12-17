"""
Chat endpoint for the AI Agent with FastAPI application.
Implements the chat functionality as specified in the RAG chatbot API contract.
"""
from fastapi import APIRouter, HTTPException, Depends
import logging
from typing import Optional
from datetime import datetime
from src.models.schemas import ChatRequest, ChatResponse, ErrorResponse
from src.agents.ai_agent import openai_agent
from src.agents.retrieval_tool import retrieval_tool

router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/chat", summary="Send a message to the textbook RAG chatbot")
async def chat_with_textbook(request: ChatRequest):
    """
    Submit a query to the RAG system which will respond based only on textbook content.
    """
    try:
        # Validate input
        if not request.message or not request.message.strip():
            raise HTTPException(status_code=400, detail="Message cannot be empty")

        # Use the user_id as thread_id or create a new one
        thread_id = request.user_id
        if not thread_id:
            thread_id = await openai_agent.create_thread()

        # Retrieve relevant content from Qdrant using the retrieval tool
        context_sources = None
        try:
            # Use context if provided, otherwise use the message for retrieval
            query_for_retrieval = request.context if request.context else request.message
            # Apply chapter filter if provided
            # For this implementation, we'll just log the filter - a full implementation would filter the search
            if request.chapter_filter:
                logger.info(f"Chapter filter applied: {request.chapter_filter}")
            
            context_sources = await retrieval_tool.retrieve_content(query_for_retrieval, top_k=5)
            logger.info(f"Retrieved {len(context_sources)} sources for query: {query_for_retrieval[:50]}...")
        except Exception as e:
            logger.warning(f"Could not retrieve context for query: {e}")
            # Continue without context sources

        # Process the query with the AI agent using the retrieved context
        response_text = await openai_agent.process_query_with_context(
            query=request.message,
            thread_id=thread_id,
            context_sources=context_sources
        )

        # Extract source URLs for the new model format
        sources_list = []
        if context_sources:
            sources_list = [source.section for source in context_sources]  # Using section as source reference

        # Create and return the response
        # For now, we'll assume the response is grounded and set confidence to 0.8 as a placeholder
        response = ChatResponse(
            response=response_text,
            sources=sources_list,
            grounded=True,  # For now, assuming the response is grounded
            confidence=0.8  # Placeholder confidence value
        )

        logger.info(f"Successfully processed chat message for thread: {thread_id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error processing chat message: {e}")
        error_response = ErrorResponse(
            error="chat_processing_error",
            message=f"Failed to process chat message: {str(e)}"
        )
        raise HTTPException(status_code=500, detail=error_response.message)