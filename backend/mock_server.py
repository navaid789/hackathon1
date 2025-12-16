#!/usr/bin/env python3
"""
Mock API server for local development without external dependencies.
This provides mock responses for the RAG chatbot functionality.
"""

import json
import time
from datetime import datetime
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import uvicorn

app = FastAPI(
    title="Mock AI Agent API",
    version="1.0.0",
    description="Mock API for frontend-backend integration testing"
)

# Add CORS middleware to allow frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://0.0.0.0:3000",
        "http://127.0.0.1:3000",
        "http://localhost:3001",
        "http://localhost:3002",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Data models (matching the real API)
class SourceReference(BaseModel):
    url: str
    section: str
    text: str
    relevance_score: float

class QueryRequest(BaseModel):
    query: str
    thread_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class QueryResponse(BaseModel):
    response: str
    sources: Optional[List[SourceReference]] = None
    thread_id: str
    timestamp: datetime

class ErrorResponse(BaseModel):
    error: str
    message: str

# In-memory storage for mock conversations
conversations = {}

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow(),
        "service": "Mock AI Agent API"
    }

@app.post("/query", response_model=QueryResponse)
async def query_agent(request: QueryRequest):
    """Mock query endpoint that simulates AI responses"""
    try:
        # Validate input
        if not request.query or not request.query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        # Use provided thread_id or create a new one
        thread_id = request.thread_id or f"thread_{int(time.time())}_{hash(request.query) % 10000}"

        # Store conversation if it doesn't exist
        if thread_id not in conversations:
            conversations[thread_id] = []

        # Create mock response based on the query
        mock_responses = {
            "hello": "Hello! I'm your AI assistant. How can I help you with the Physical AI Humanoid textbook?",
            "test": "This is a test response from the mock API. The RAG chatbot is working!",
            "book": "The Physical AI Humanoid textbook covers topics in embodied AI, including perception, decision-making, and action in real-world environments.",
            "theme": "The main themes include embodied intelligence, simulation-to-reality transfer, and the integration of AI with physical systems.",
            "ai": "Artificial Intelligence in physical systems focuses on creating intelligent agents that can interact with their environment through perception and action.",
            "physical": "Physical AI combines artificial intelligence with real-world interaction, enabling systems to perceive, reason, and act in physical spaces."
        }

        query_lower = request.query.lower()
        response_text = "I'm a mock AI assistant. In a real implementation, I would analyze the book content and provide a relevant response based on your query. "

        # Try to match the query to provide more relevant responses
        for key, response in mock_responses.items():
            if key in query_lower:
                response_text = response
                break

        # Add some context about the project if it's a general query
        if response_text.startswith("I'm a mock AI"):
            response_text += f"Your query was: '{request.query}'. In the real system, this would search the Physical AI Humanoid textbook content and return relevant information with citations."

        # Create mock sources
        sources = [
            SourceReference(
                url="https://example.com/physical-ai-humanoid/chapter1",
                section="Chapter 1: Introduction",
                text="The Physical AI Humanoid textbook explores the integration of artificial intelligence with physical systems...",
                relevance_score=0.85
            )
        ] if "book" in query_lower or "content" in query_lower else None

        # Create and return the response
        response = QueryResponse(
            response=response_text,
            sources=sources,
            thread_id=thread_id,
            timestamp=datetime.utcnow()
        )

        # Store the conversation
        conversations[thread_id].append({
            "query": request.query,
            "response": response_text,
            "timestamp": datetime.utcnow().isoformat()
        })

        return response

    except HTTPException:
        raise
    except Exception as e:
        error_response = ErrorResponse(
            error="query_processing_error",
            message=f"Mock server error: {str(e)}"
        )
        raise HTTPException(status_code=500, detail=error_response.message)

@app.get("/conversations/{thread_id}")
async def get_conversation(thread_id: str):
    """Get conversation history"""
    if thread_id not in conversations:
        raise HTTPException(status_code=404, detail="Conversation not found")
    return {
        "thread_id": thread_id,
        "messages": conversations[thread_id]
    }

if __name__ == "__main__":
    print("Starting Mock API Server for RAG Chatbot...")
    print("Server will be available at http://127.0.0.1:8002")
    print("This provides mock responses for frontend-backend integration without external dependencies.")
    uvicorn.run(app, host="127.0.0.1", port=8002, log_level="info")