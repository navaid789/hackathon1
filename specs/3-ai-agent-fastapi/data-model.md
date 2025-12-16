# Data Model: AI Agent with FastAPI

## Feature Context
Building a backend AI agent using OpenAI Agents SDK and FastAPI, integrating vector-based retrieval to enable question answering over book content.

## Key Entities

### 1. QueryRequest
**Description**: Input from users containing their questions about book content
**Fields**:
- query: string (required) - The user's question about book content
- thread_id: string (optional) - For maintaining conversation context
- metadata: object (optional) - Additional context for the query

**Validation Rules**:
- query must be 1-1000 characters
- query must not be empty or whitespace only
- thread_id must follow UUID format if provided

### 2. QueryResponse
**Description**: Response from the AI agent containing the answer and relevant information
**Fields**:
- response: string (required) - The agent's answer to the user's question
- sources: array of SourceReference (optional) - List of documents used to generate the response
- thread_id: string (required) - Thread identifier for conversation continuity
- timestamp: datetime (required) - When the response was generated

**Validation Rules**:
- response must be 1-10000 characters
- sources array must contain 0-10 references
- timestamp must be in ISO 8601 format

### 3. SourceReference
**Description**: Reference to a specific piece of content from the book that was used in the response
**Fields**:
- url: string (required) - URL where the content was originally sourced from
- section: string (required) - Section or chapter where the content appears
- text: string (required) - The actual text content that was referenced
- relevance_score: number (required) - Similarity score between 0-1 indicating relevance

**Validation Rules**:
- url must be a valid URL format
- section must be 1-200 characters
- text must be 1-5000 characters
- relevance_score must be between 0 and 1 (inclusive)

### 4. AgentThread
**Description**: Represents a conversation thread with the AI agent
**Fields**:
- thread_id: string (required) - Unique identifier for the conversation thread
- created_at: datetime (required) - When the thread was created
- last_accessed: datetime (required) - When the thread was last used
- messages: array of Message (required) - History of messages in the thread

**Validation Rules**:
- thread_id must follow UUID format
- created_at and last_accessed must be in ISO 8601 format
- messages array must contain 1-100 messages

### 5. Message
**Description**: A single message in a conversation thread
**Fields**:
- message_id: string (required) - Unique identifier for the message
- role: string (required) - Either "user" or "assistant"
- content: string (required) - The text content of the message
- timestamp: datetime (required) - When the message was created

**Validation Rules**:
- message_id must follow UUID format
- role must be either "user" or "assistant"
- content must be 1-10000 characters
- timestamp must be in ISO 8601 format

### 6. RetrievalResult
**Description**: Result from the Qdrant vector database retrieval
**Fields**:
- content_chunks: array of ContentChunk (required) - Retrieved relevant content
- query_embedding: array of number (required) - The embedding vector of the query
- search_metadata: object (optional) - Additional metadata about the search

**Validation Rules**:
- content_chunks array must contain 0-20 items
- query_embedding must be an array of 1024 numbers (for Cohere embeddings)
- Each content chunk must have valid text and metadata

### 7. ContentChunk
**Description**: A segment of book content stored in the vector database
**Fields**:
- text: string (required) - The actual text content
- url: string (required) - Source URL of the content
- section: string (required) - Section or chapter identifier
- embedding: array of number (required) - Vector embedding of the content
- metadata: object (optional) - Additional metadata about the content

**Validation Rules**:
- text must be 1-5000 characters
- url must be a valid URL format
- section must be 1-200 characters
- embedding must be an array of 1024 numbers (for Cohere embeddings)

## Relationships

```
QueryRequest --[processes to]--> AgentThread --[contains]--> Message
QueryResponse --[includes]--> SourceReference
AgentThread --[uses]--> RetrievalResult --[contains]--> ContentChunk
```

## State Transitions

### Query Processing States
1. **Received**: QueryRequest is received by the API
2. **Processing**: Agent is retrieving relevant content and formulating response
3. **Completed**: QueryResponse is generated and returned to user
4. **Error**: Processing failed due to system or external issues

## Data Validation Requirements

All data models must comply with the following:
- All string fields must be properly sanitized to prevent injection attacks
- All numeric fields must be validated for appropriate ranges
- All datetime fields must be in ISO 8601 format
- All identifiers must follow UUID format when specified
- All URLs must be valid URL format
- Content length limits must be enforced to prevent excessive resource usage