"""
Vercel serverless function entry point for the FastAPI application.
"""
import sys
import os

# Add the src directory to the path so imports work correctly
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.api.main import app

# Vercel serverless functions expect a default handler
from mangum import Mangum

handler = Mangum(app)

# Also export the FastAPI app for direct use
def main():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()