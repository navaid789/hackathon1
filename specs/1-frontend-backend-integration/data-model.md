# Data Model: Frontend-Backend Integration

## Overview
This document defines the data structures and relationships for the frontend-backend integration feature, focusing on the entities involved in the query and response flow.

## Key Entities

### Query Request
**Description**: Represents a user's query along with optional context from selected text

**Fields**:
- `query` (string, required): The user's question or query text
- `context` (string, optional): Selected text from the book that provides context
- `thread_id` (string, optional): Identifier for maintaining conversation context
- `metadata` (object, optional): Additional information about the query source

**Validation Rules**:
- `query` must not be empty
- `query` length should be between 1 and 2000 characters
- `context` length should not exceed 5000 characters when provided

### Agent Response
**Description**: The AI-generated response to a user query

**Fields**:
- `response` (string, required): The AI-generated answer text
- `sources` (array of SourceReference, optional): List of documents used to generate the response
- `thread_id` (string, required): Identifier for the conversation thread
- `timestamp` (datetime, required): When the response was generated
- `query_id` (string, required): Unique identifier for this query-response pair

**Validation Rules**:
- `response` must not be empty
- `sources` array should have maximum 10 items
- `timestamp` must be in ISO 8601 format

### SourceReference
**Description**: Reference to a specific document or text that was used in generating the response

**Fields**:
- `url` (string, required): URL or identifier of the source document
- `section` (string, optional): Specific section or heading within the document
- `text` (string, optional): Snippet of the referenced text
- `confidence` (number, optional): Confidence score of the source relevance (0-1)

**Validation Rules**:
- `url` must be a valid URL or document identifier
- `confidence` must be between 0 and 1 if provided

### Connection Configuration
**Description**: Settings that define how the frontend connects to the backend API

**Fields**:
- `api_base_url` (string, required): Base URL for the backend API
- `timeout` (number, optional): Request timeout in milliseconds (default: 30000)
- `retries` (number, optional): Number of retry attempts for failed requests (default: 3)

**Validation Rules**:
- `api_base_url` must be a valid URL
- `timeout` must be between 5000 and 60000 milliseconds
- `retries` must be between 0 and 5

## State Transitions

### Query State Machine
1. **IDLE**: Initial state when no query is in progress
2. **LOADING**: When request is being sent to backend
3. **SUCCESS**: When response is received successfully
4. **ERROR**: When an error occurs during the request

### Connection State Machine
1. **DISCONNECTED**: Initial state when API connection is not established
2. **CONNECTING**: When establishing connection to backend
3. **CONNECTED**: When successfully connected to backend
4. **FAILED**: When connection attempt fails

## Relationships

- A `Query Request` generates one `Agent Response`
- An `Agent Response` can include multiple `SourceReference` objects
- `Connection Configuration` is used by API client to make requests