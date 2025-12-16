# AI Agent with FastAPI

Backend service for an AI agent that answers questions based on book content using OpenAI, Qdrant, and Cohere.

## Features

- FastAPI-based web service with async support
- OpenAI Assistant integration for conversational AI
- Qdrant vector database for content retrieval
- Cohere embeddings for semantic search
- Thread-based conversation management
- Source attribution for grounded responses

## Prerequisites

- Python 3.13 or higher
- OpenAI API key
- Qdrant Cloud account and API key
- Cohere API key

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables:
   ```bash
   cp .env .env.local
   # Edit .env.local and add your API keys
   ```

3. Run the application:
   ```bash
   python -m uvicorn src.api.main:app --reload --port 8000
   ```

## Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `COHERE_API_KEY`: Your Cohere API key
- `DEBUG`: Set to "true" for debug mode (default: "false")
- `LOG_LEVEL`: Logging level (default: "INFO")

## API Endpoints

- `GET /health`: Health check endpoint
- `POST /query`: Query the AI agent with a question

## Example Query

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main themes in this book?",
    "thread_id": "optional-thread-id"
  }'
```

## Testing

Run the tests:
```bash
cd backend
pytest
```

## Architecture

The application is organized into the following modules:

- `src/agents/`: AI agent and retrieval tool implementations
- `src/api/`: FastAPI application and route definitions
- `src/services/`: External service integrations (Qdrant, Cohere)
- `src/models/`: Data models and schemas
- `src/config/`: Configuration and settings
- `tests/`: Unit and integration tests

## License

[Specify your license here]