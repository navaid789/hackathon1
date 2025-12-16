#!/usr/bin/env python3
"""
Simplified script to run just the FastAPI server without RAG pipeline initialization.
This avoids the Qdrant connection issues while still providing the API for the frontend.
"""

import uvicorn
import sys
import os
from src.api.main import app

def main():
    """Run the FastAPI server without initializing the RAG pipeline."""
    print("Starting FastAPI server for frontend-backend integration...")
    print("Server will be available at http://127.0.0.1:8000")
    print("Press Ctrl+C to stop the server")

    uvicorn.run(
        app,
        host="127.0.0.1",
        port=8000,
        reload=False  # Set to False for production use
    )

if __name__ == "__main__":
    main()