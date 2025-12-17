"""
Data models and request/response schemas for the AI Agent with FastAPI application.
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid


class SourceReference(BaseModel):
    """
    Reference to a specific piece of content from the book that was used in the response
    """
    url: str = Field(..., description="URL where the content was originally sourced from")
    section: str = Field(..., description="Section or chapter where the content appears")
    text: str = Field(..., description="The actual text content that was referenced", max_length=5000)
    relevance_score: float = Field(..., description="Similarity score between 0-1 indicating relevance", ge=0.0, le=1.0)


class QueryRequest(BaseModel):
    """
    Input from users containing their questions about book content
    """
    query: str = Field(..., description="The user's question about book content", min_length=1, max_length=1000)
    thread_id: Optional[str] = Field(None, description="Optional thread ID to maintain conversation context")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional context for the query")


class QueryResponse(BaseModel):
    """
    Response from the AI agent containing the answer and relevant information
    """
    response: str = Field(..., description="The agent's answer to the user's question", min_length=1, max_length=10000)
    sources: Optional[List[SourceReference]] = Field(None, description="List of documents used to generate the response", max_items=10)
    thread_id: str = Field(..., description="Thread identifier for conversation continuity")
    timestamp: datetime = Field(..., description="When the response was generated")


class ChatRequest(BaseModel):
    """
    Request model for the chat endpoint as specified in the RAG chatbot API contract
    """
    message: str = Field(..., description="The user's message or question", example="Explain how ROS 2 handles real-time constraints")
    context: Optional[str] = Field(None, description="The selected text context from the textbook (optional)", example="ROS 2 architecture includes real-time constraints...")
    user_id: Optional[str] = Field(None, description="Optional user identifier for personalization")
    chapter_filter: Optional[str] = Field(None, description="Optional filter to limit search to specific chapter")


class ChatResponse(BaseModel):
    """
    Response model for the chat endpoint as specified in the RAG chatbot API contract
    """
    response: str = Field(..., description="The chatbot's response to the user")
    sources: List[str] = Field(..., description="List of textbook sections/chapters used as sources")
    grounded: bool = Field(..., description="Whether the response is properly grounded in textbook content")
    confidence: Optional[float] = Field(None, ge=0, le=1, description="Confidence score for the response")


class SearchRequest(BaseModel):
    """
    Request model for the search endpoint as specified in the RAG chatbot API contract
    """
    query: str = Field(..., description="The search query", example="sim-to-real transfer challenges")
    limit: Optional[int] = Field(5, ge=1, le=20, description="Maximum number of results to return")
    chapter_filter: Optional[str] = Field(None, description="Optional filter to search within specific chapter")


class SearchResult(BaseModel):
    """
    Result model for individual search results as specified in the RAG chatbot API contract
    """
    content: str = Field(..., description="The relevant content excerpt")
    chapter: str = Field(..., description="The chapter where the content appears")
    section: str = Field(..., description="The section within the chapter")
    relevance_score: float = Field(..., ge=0, le=1, description="Relevance score for the result")
    page_reference: Optional[str] = Field(None, description="Page reference if available")


class SearchResponse(BaseModel):
    """
    Response model for the search endpoint as specified in the RAG chatbot API contract
    """
    results: List[SearchResult] = Field(..., description="List of search results")
    total_count: int = Field(..., description="Total number of results available")


class ValidateRequest(BaseModel):
    """
    Request model for the validate-response endpoint as specified in the RAG chatbot API contract
    """
    response: str = Field(..., description="The response to validate")
    sources: List[str] = Field(..., description="Potential sources for the response")
    selected_text: Optional[str] = Field(None, description="Text selected by user (if any)")


class ValidateResponse(BaseModel):
    """
    Response model for the validate-response endpoint as specified in the RAG chatbot API contract
    """
    is_valid: bool = Field(..., description="Whether the response is properly grounded")
    issues: List[str] = Field(..., description="List of validation issues found")
    suggestions: Optional[List[str]] = Field(None, description="Suggestions for improving grounding")


class ErrorResponse(BaseModel):
    """
    Error response model for the API
    """
    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Human-readable error message")


class Message(BaseModel):
    """
    A single message in a conversation thread
    """
    message_id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique identifier for the message")
    role: str = Field(..., description="Either 'user' or 'assistant'", pattern="^(user|assistant)$")
    content: str = Field(..., description="The text content of the message", min_length=1, max_length=10000)
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the message was created")


class AgentThread(BaseModel):
    """
    Represents a conversation thread with the AI agent
    """
    thread_id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique identifier for the conversation thread")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When the thread was created")
    last_accessed: datetime = Field(default_factory=datetime.utcnow, description="When the thread was last used")
    messages: List[Message] = Field(..., description="History of messages in the thread", min_items=1, max_items=100)


class ContentChunk(BaseModel):
    """
    A segment of book content stored in the vector database
    """
    text: str = Field(..., description="The actual text content", max_length=5000)
    url: str = Field(..., description="Source URL of the content")
    section: str = Field(..., description="Section or chapter identifier", max_length=200)
    embedding: List[float] = Field(..., description="Vector embedding of the content")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata about the content")


class RetrievalResult(BaseModel):
    """
    Result from the Qdrant vector database retrieval
    """
    content_chunks: List[ContentChunk] = Field(..., description="Retrieved relevant content", max_items=20)
    query_embedding: List[float] = Field(..., description="The embedding vector of the query")
    search_metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata about the search")