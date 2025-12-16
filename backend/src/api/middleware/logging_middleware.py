"""
Logging middleware for the AI Agent with FastAPI application.
"""
import time
import logging
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response

logger = logging.getLogger(__name__)

class LoggingMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        start_time = time.time()

        # Process the request
        response = await call_next(request)

        # Calculate processing time
        process_time = time.time() - start_time

        # Log the request details
        logger.info(
            f"{request.method} {request.url.path} - Status: {response.status_code} - "
            f"Process time: {process_time:.4f}s"
        )

        return response