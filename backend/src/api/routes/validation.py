"""
Validation endpoint for the AI Agent with FastAPI application.
Implements the response validation functionality as specified in the RAG chatbot API contract.
"""
from fastapi import APIRouter, HTTPException, Depends
import logging
from typing import List, Optional
from datetime import datetime
from src.models.schemas import ValidateRequest, ValidateResponse, ErrorResponse
from src.agents.retrieval_tool import retrieval_tool

router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/validate-response", summary="Validate chatbot response for grounding")
async def validate_response(request: ValidateRequest):
    """
    Check if a response is properly grounded in textbook content.
    """
    try:
        # Validate input
        if not request.response or not request.response.strip():
            raise HTTPException(status_code=400, detail="Response cannot be empty")

        # For now, we'll implement a basic validation approach.
        # In a full implementation, this would check if the response is supported by the sources.
        is_valid = True  # Placeholder - in a real implementation, we'd validate against sources
        issues = []  # Placeholder - list of validation issues
        suggestions = []  # Placeholder - suggestions for improvement

        # Basic validation: check if sources are provided and if parts of the response
        # can be found in the sources
        if request.sources and len(request.sources) > 0:
            response_text = request.response.lower()
            source_content = " ".join(request.sources).lower()

            # Simple check: see if key phrases from response appear in sources
            # This is a very basic check - in a full implementation, we would use semantic validation
            response_words = response_text.split()
            if len(response_words) > 0:
                # Take first few words of response to check against sources
                sample_text = " ".join(response_words[:min(5, len(response_words))])
                if sample_text not in source_content:
                    is_valid = False
                    issues.append("Key phrases from the response were not found in the provided sources")
                    suggestions.append("Ensure the response is grounded in the provided source content")
        else:
            # If no sources are provided, the response cannot be validated
            is_valid = False
            issues.append("No sources provided for validation")
            suggestions.append("Provide sources that support the response")

        # Create and return the validation response
        response = ValidateResponse(
            is_valid=is_valid,
            issues=issues,
            suggestions=suggestions
        )

        logger.info(f"Successfully validated response with grounding: {is_valid}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error validating response: {e}")
        error_response = ErrorResponse(
            error="validation_error",
            message=f"Failed to validate response: {str(e)}"
        )
        raise HTTPException(status_code=500, detail=error_response.message)