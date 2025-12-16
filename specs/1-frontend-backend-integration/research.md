# Research: Frontend-Backend Integration

## Overview
This research document addresses the technical requirements for integrating the RAG backend with the frontend book interface, enabling users to query the chatbot directly from the published book.

## Research Tasks

### 1. Configure frontend API client for FastAPI

**Decision**: Use fetch API or axios for HTTP requests from frontend to FastAPI backend
**Rationale**: Both approaches are standard for web applications. Fetch API is built into browsers, while axios provides additional features like interceptors and request/response transformation.
**Recommendation**: Use fetch API for simplicity, with a custom wrapper for common functionality like error handling and request formatting.
**Alternatives considered**:
- Axios: More feature-rich but adds bundle size
- Custom HTTP client: More control but requires more implementation
- Native fetch: Lightweight and standard

### 2. Enable CORS and local networking

**Decision**: Configure FastAPI to allow CORS requests from the frontend domain
**Rationale**: CORS (Cross-Origin Resource Sharing) prevents browsers from making requests to different origins by default. For local development, we need to explicitly allow requests from the frontend (likely running on a different port).
**Implementation**: Use FastAPI's CORSMiddleware to allow requests from the frontend origin during development.
**Alternatives considered**:
- Proxy requests through the web server: More complex setup
- Disable CORS in browser: Not practical for users
- CORS configuration: Standard approach for development

### 3. Wire chatbot UI to backend endpoint

**Decision**: Create a chat interface component that communicates with the existing `/query` endpoint
**Rationale**: The backend already has a query endpoint that handles AI agent interactions. The frontend needs a UI component that can capture user input (with optional selected text context) and display responses.
**Implementation**: Create React components for the chat interface, including:
- Input field for queries
- Context display for selected text
- Response display area
- Loading states
- Error handling
**Alternatives considered**:
- Full chat interface: More complex but richer experience
- Simple form: Less interactive
- Modal-based interface: Contextual but may interrupt reading flow

### 4. Test query flow end-to-end

**Decision**: Implement comprehensive testing at multiple levels
**Rationale**: To ensure the integration works reliably, we need tests at API, integration, and end-to-end levels.
**Implementation**:
- API tests: Verify backend endpoints work correctly
- Integration tests: Verify frontend-backend communication
- E2E tests: Verify complete user flow from query to response
**Alternatives considered**:
- Unit tests only: Don't verify integration
- Manual testing: Not scalable or reliable
- Automated testing: Comprehensive but requires more setup

## Technical Specifications

### API Client Configuration
- Base URL: `http://localhost:8000` (default FastAPI port)
- Endpoints: `/query` for chatbot interactions
- Request format: JSON with query text and optional context
- Response format: JSON with response text and sources

### CORS Configuration
- Allow origins: `http://localhost:3000` (Docusaurus default) and `http://0.0.0.0:3000`
- Allow methods: GET, POST, OPTIONS
- Allow headers: Content-Type, Authorization (if needed)
- Credentials: May need to be configured based on authentication needs

### UI Component Design
- Position: Overlay or sidebar that doesn't obstruct reading
- Context: Ability to pass selected text as context
- History: Display conversation history
- Loading states: Visual feedback during API calls

## Dependencies and Setup

### Frontend Dependencies
- React (if not already present)
- Optional: axios or fetch wrapper utilities
- Optional: state management library (Redux, Zustand, etc.)

### Backend Dependencies
- FastAPI's CORSMiddleware (already present in existing backend)
- Proper configuration in settings

## Integration Points

### Frontend Integration
- Locate existing book interface components
- Add chatbot interface component
- Implement text selection capture
- Connect to API client

### Backend Integration
- Verify existing `/query` endpoint supports the required functionality
- Ensure proper CORS configuration
- Confirm response format compatibility with frontend needs

## Risk Assessment

### High Risk Items
- CORS configuration conflicts with security requirements
- Performance issues with API response times
- Text selection capture not working properly in Docusaurus

### Mitigation Strategies
- Thorough testing of CORS configuration
- Performance monitoring of API endpoints
- Alternative text selection methods if needed