# Quickstart Guide: AI Agent with FastAPI

## Feature Context
Building a backend AI agent using OpenAI Agents SDK and FastAPI, integrating vector-based retrieval to enable question answering over book content.

## Prerequisites

- Python 3.13 or higher
- OpenAI API key
- Qdrant Cloud account and API key
- Cohere API key
- Git

## Setup Instructions

### 1. Clone and Navigate to Project
```bash
git clone <your-repo-url>
cd <project-directory>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install fastapi uvicorn openai cohere qdrant-client python-dotenv
```

### 4. Set Up Environment Variables
Create a `.env` file in the project root with the following:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
COHERE_API_KEY=your_cohere_api_key
```

## Running the Application

### 1. Start the FastAPI Server
```bash
cd backend
uvicorn src.api.main:app --reload --port 8000
```

### 2. Test the API
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main themes in this book?"
  }'
```

## Key Components

### 1. AI Agent (`src/agents/ai_agent.py`)
- Implements the OpenAI Assistant API
- Manages conversation threads
- Integrates with retrieval tools

### 2. Retrieval Tool (`src/agents/retrieval_tool.py`)
- Queries Qdrant vector database
- Generates embeddings using Cohere
- Returns relevant book content

### 3. API Routes (`src/api/routes/query.py`)
- Handles query requests
- Processes responses
- Manages thread state

### 4. Services (`src/services/`)
- Qdrant service for vector database operations
- Cohere service for embedding generation

## Configuration

### Environment Variables
- `OPENAI_API_KEY`: API key for OpenAI services
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_URL`: URL of your Qdrant cluster
- `COHERE_API_KEY`: API key for Cohere services
- `OPENAI_MODEL`: OpenAI model to use (default: gpt-4-turbo)

## Testing

### Run Unit Tests
```bash
pytest tests/unit/
```

### Run Integration Tests
```bash
pytest tests/integration/
```

## API Endpoints

### Query Endpoint
- **POST** `/query`
- Submit questions to the AI agent
- Returns grounded responses based on book content

### Health Check
- **GET** `/health`
- Verify service is running

## Troubleshooting

### Common Issues

1. **API Key Errors**: Ensure all required API keys are set in `.env`
2. **Qdrant Connection**: Verify QDRANT_URL and QDRANT_API_KEY are correct
3. **Rate Limits**: Monitor API usage to avoid exceeding rate limits

### Debugging Tips
- Enable debug logging by setting `LOG_LEVEL=debug`
- Check the logs for detailed error information
- Verify the vector database has been populated with book content