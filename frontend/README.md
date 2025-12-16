# Frontend Documentation: RAG Chatbot Integration

## Overview
This frontend integrates with a RAG (Retrieval-Augmented Generation) backend to provide AI-powered chat capabilities directly within the book interface. Users can select text from book content and ask questions about it, receiving contextual responses from an AI agent.

## Architecture
The frontend uses a service-oriented architecture with the following key components:

- **API Client**: Handles communication with the backend API
- **State Management**: Manages loading states, errors, and responses
- **Text Selection**: Captures selected text from the book interface
- **Connection Management**: Handles connection status and retry logic
- **Chatbot UI**: Provides the interface for user interaction

## Key Components

### API Client (`src/services/apiClient.js`)
Handles all communication with the backend API, including:
- Query submission with optional context
- Error handling and response parsing
- Request timeout and retry mechanisms

### Text Selection (`src/services/textSelection.js`)
Captures selected text from the book interface:
- Uses browser's `window.getSelection()` API
- Provides utilities to extract and format selected content
- Handles edge cases like empty selections

### Connection Manager (`src/services/connectionManager.js`)
Manages the connection state with the backend:
- Tracks connection status (connected/disconnected)
- Implements retry logic for failed requests
- Provides fallback mechanisms for backend unavailability

### Chatbot UI (`src/components/Chatbot/`)
React components that provide the user interface:
- Query input field
- Response display area
- Context visualization
- Loading and error states

## API Integration

### Query Endpoint
The frontend communicates with the backend's `/query` endpoint:

```javascript
// Example request
const response = await fetch('http://localhost:8000/query', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    query: userQuery,
    context: selectedText,  // Optional
    thread_id: conversationId  // Optional
  })
});
```

### Request Format
- `query` (string, required): The user's question
- `context` (string, optional): Selected text providing context
- `thread_id` (string, optional): Conversation identifier

### Response Format
- `response` (string): AI-generated answer
- `sources` (array): List of source references used
- `thread_id` (string): Conversation identifier
- `query_id` (string): Unique query identifier
- `timestamp` (string): Response timestamp

## Error Handling
The system implements comprehensive error handling:
- Network errors with retry mechanisms
- Backend unavailability with graceful degradation
- Invalid responses with user feedback
- Timeout handling for long-running requests

## Performance Considerations
- API responses are cached when appropriate
- Loading states provide immediate user feedback
- Connection metrics monitor performance
- Optimized for responses under 10 seconds

## Security
- All API communication uses HTTPS in production
- Input validation prevents injection attacks
- Secure handling of API keys (stored in environment)
- CORS configuration allows only trusted origins

## Testing
Unit tests are located in `frontend/tests/` and cover:
- API client functionality
- Text selection utilities
- State management
- UI component behavior

## Development Setup

### Prerequisites
- Node.js (v18 or higher)
- npm or yarn package manager

### Installation
```bash
# Install dependencies
npm install

# Start development server
npm run start
```

### Environment Variables
The frontend uses the following environment variables:
- `REACT_APP_API_BASE_URL`: Backend API base URL (defaults to http://localhost:8000)

## Deployment
The frontend can be built for production:
```bash
npm run build
```

## Troubleshooting

### Common Issues
1. **CORS Errors**: Verify backend CORS configuration allows frontend origin
2. **API Connection**: Check that backend is running and accessible
3. **Text Selection**: Ensure the book interface allows text selection

### Debugging
- Use browser developer tools to inspect network requests
- Check console for JavaScript errors
- Verify API endpoint configuration