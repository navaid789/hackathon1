"""
Main FastAPI application for the AI Agent with FastAPI feature.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.config.settings import settings
from src.api.middleware.logging_middleware import LoggingMiddleware
import logging

# Configure logging
logging.basicConfig(
    level=settings.log_level,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Create FastAPI app instance
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    debug=settings.debug,
    description="AI Agent with FastAPI for question answering over book content"
)

# Add CORS middleware for frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Docusaurus default
        "http://0.0.0.0:3000",   # Alternative Docusaurus address
        "http://127.0.0.1:3000", # Alternative Docusaurus address
        "http://localhost:3001", # Alternative port for Docusaurus
        "http://localhost:3002", # Additional alternative port
    ],
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods (GET, POST, OPTIONS, etc.)
    allow_headers=["*"],  # Allow all headers
    # Additional security: Only allow specific headers in production
    # allow_headers=["Content-Type", "Authorization"],
)

# Add other middleware
app.add_middleware(LoggingMiddleware)

# Import and include routes
from src.api.routes import query, health
app.include_router(health.router, prefix="", tags=["health"])
app.include_router(query.router, prefix="", tags=["query"])

@app.on_event("startup")
async def startup_event():
    logger.info(f"Starting {settings.app_name} v{settings.app_version}")
    # Any startup logic can go here

@app.on_event("shutdown")
async def shutdown_event():
    logger.info(f"Shutting down {settings.app_name}")
    # Any cleanup logic can go here

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug
    )