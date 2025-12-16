#!/usr/bin/env python3
"""
Wrapper for main.py that handles missing external services gracefully.
This allows uv run main.py to work in local development without external dependencies.
"""

import sys
import os
import argparse
from typing import Optional

def main_with_fallback():
    """
    Main function that attempts to run the RAG pipeline but falls back gracefully
    if external services are not available.
    """
    import argparse

    # Set up command-line argument parsing (same as original)
    parser = argparse.ArgumentParser(description='Docusaurus RAG Pipeline with local fallback')
    parser.add_argument('--base-url', type=str, default=None,
                        help='Base URL of the Docusaurus site to process (default: from environment)')
    parser.add_argument('--collection-name', type=str, default='rag_embeddings',
                        help='Name of the Qdrant collection to use (default: rag_embeddings)')
    parser.add_argument('--chunk-size', type=int, default=1000,
                        help='Maximum size of text chunks (default: 1000)')
    parser.add_argument('--top-k', type=int, default=3,
                        help='Number of results to return in validation search (default: 3)')
    parser.add_argument('--validation-query', type=str, default='What is this documentation about?',
                        help='Query to use for validation search (default: "What is this documentation about?")')
    parser.add_argument('--local-mode', action='store_true',
                        help='Run in local mode without external dependencies')

    args = parser.parse_args()

    if args.local_mode:
        # Run in local mode - just start the API server
        print("Running in local mode without external dependencies...")
        print("Starting API server only...")

        # Import and run the API server without RAG pipeline
        try:
            from src.api.main import app
            import uvicorn
            print("API server is available at http://127.0.0.1:8000")
            print("This provides the backend for frontend-backend integration.")
            print("Press Ctrl+C to stop the server.")
            uvicorn.run(app, host="127.0.0.1", port=8000, log_level="info")
        except Exception as e:
            print(f"Error starting API server: {e}")
            sys.exit(1)
    else:
        # Try to run the full pipeline, but handle errors gracefully
        try:
            # Import the original main function
            from main import main as original_main
            original_main()
        except Exception as e:
            error_msg = str(e).lower()
            if "qdrant" in error_msg or "getaddrinfo" in error_msg or "connection" in error_msg:
                print(f"External service connection failed: {e}")
                print("\nTo run in local development mode without external dependencies, use:")
                print("  python main.py --local-mode")
                print("\nThis will start the API server only, which is sufficient for frontend-backend integration.")
                sys.exit(1)
            else:
                # Re-raise if it's a different error
                raise e

if __name__ == "__main__":
    main_with_fallback()