# Quickstart Guide: Frontend-Backend Integration

## Overview
This guide provides instructions for setting up and running the integrated frontend-backend system for the RAG chatbot feature.

## Prerequisites
- Node.js (v18 or higher)
- Python (v3.13 or higher)
- Git
- API keys for OpenAI, Qdrant, and Cohere

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd hackathon1
```

### 2. Backend Setup
```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env .env.local
# Edit .env.local and add your API keys:
# OPENAI_API_KEY=your_openai_key
# QDRANT_API_KEY=your_qdrant_key
# QDRANT_URL=your_qdrant_url
# COHERE_API_KEY=your_cohere_key
```

### 3. Frontend Setup
```bash
# From the project root
cd ..

# Install dependencies (if using Docusaurus)
npm install
```

### 4. Start Backend Server
```bash
# From the backend directory
cd backend
source venv/bin/activate  # Activate virtual environment
uvicorn src.api.main:app --reload --port 8000
```

### 5. Start Frontend Server
```bash
# From the project root
npm run start
# or if using Docusaurus:
npm run start
```

## API Configuration

### CORS Settings
The backend is configured to allow requests from:
- `http://localhost:3000` (default Docusaurus port)
- `http://0.0.0.0:3000`
- `http://127.0.0.1:3000`
- `http://localhost:3000` (Docusaurus development server)

### API Endpoints
- **Query Endpoint**: `POST http://localhost:8000/query`
- **Health Check**: `GET http://localhost:8000/health`

### Environment Configuration
Make sure to set the following environment variables for both frontend and backend:

**Backend (.env file):**
```
OPENAI_API_KEY=your_openai_key
QDRANT_API_KEY=your_qdrant_key
QDRANT_URL=your_qdrant_url
COHERE_API_KEY=your_cohere_key
```

**Frontend (.env file):**
```
REACT_APP_API_BASE_URL=http://localhost:8000
```

## Frontend Integration

### Making API Calls
The frontend uses a service to communicate with the backend:

```javascript
// Example API call
const queryBackend = async (query, context = null) => {
  const response = await fetch('http://localhost:8000/query', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      query: query,
      context: context,
    }),
  });

  if (!response.ok) {
    throw new Error(`API request failed: ${response.statusText}`);
  }

  return await response.json();
};
```

### Text Selection Feature
The frontend should capture selected text and pass it as context:

```javascript
// Example text selection capture
const getSelectedText = () => {
  const selection = window.getSelection();
  return selection.toString().trim();
};
```

## Testing the Integration

### Manual Testing
1. Start both frontend and backend servers
2. Navigate to the book interface in your browser
3. Select some text from the book content
4. Use the chatbot interface to ask a question related to the selected text
5. Verify that the response is relevant and includes proper citations

### API Testing
Test the backend endpoint directly:
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main themes?",
    "context": "The author explores themes of identity and belonging..."
  }'
```

## Running with Docker Compose

For a complete integrated environment, you can use the provided docker-compose configuration:

```bash
# Make sure you have your API keys in a .env file at the project root
# Then start the entire system
docker-compose up -d

# This will start:
# - Backend API server on port 8000
# - Frontend on port 3000
# - Qdrant vector database on port 6333
```

## Troubleshooting

### Common Issues
1. **CORS Errors**: Ensure both servers are running and CORS is properly configured
2. **API Keys**: Verify all required API keys are set in environment variables
3. **Port Conflicts**: Check that ports 8000 (backend) and 3000 (frontend) are available
4. **Docker Issues**: Ensure Docker and Docker Compose are properly installed

### Debugging Tips
- Check backend logs for error messages
- Use browser developer tools to inspect network requests
- Verify API endpoint URLs match your running services
- For Docker deployments, check container logs with `docker logs <container-name>`