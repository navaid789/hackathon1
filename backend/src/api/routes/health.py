"""
Health check endpoint for the AI Agent with FastAPI application.
"""
from fastapi import APIRouter
from datetime import datetime
from typing import Dict, Any

router = APIRouter()

@router.get("/health", summary="Health check endpoint")
async def health_check() -> Dict[str, Any]:
    """
    Health check endpoint to verify if the AI agent service is running and healthy.
    """
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "service": "AI Agent with FastAPI"
    }