# Data Model: Docusaurus RAG Pipeline

## Entities

### ExtractedContent
- **id**: string - Unique identifier for the content chunk
- **url**: string - Source URL of the content
- **section**: string - Section/heading of the content
- **title**: string - Title of the page containing the content
- **text**: string - Clean extracted text content
- **metadata**: dict - Additional metadata (timestamp, source, etc.)

### Embedding
- **id**: string - Unique identifier matching the content chunk
- **vector**: list[float] - Vector representation from Cohere embedding
- **content_id**: string - Reference to the source content
- **model**: string - Name of the embedding model used

### VectorRecord
- **id**: string - Unique identifier for the Qdrant record
- **vector**: list[float] - Embedding vector for similarity search
- **payload**: dict - Metadata including URL, section, title, and original text
- **collection**: string - Name of the Qdrant collection (e.g., "rag_embeddings")

## Relationships

- One `ExtractedContent` entity maps to one `Embedding` entity (1:1)
- One `Embedding` entity maps to one `VectorRecord` entity (1:1)
- Multiple `VectorRecord` entities belong to one collection (e.g., "rag_embeddings")

## Validation Rules

### ExtractedContent Validation
- URL must be a valid HTTP/HTTPS URL
- Text content must not be empty
- Section and title should be less than 500 characters
- Metadata must be a valid JSON object

### Embedding Validation
- Vector must have consistent dimensions based on the embedding model
- Content_id must reference an existing ExtractedContent entity
- Model name must be a valid Cohere model identifier

### VectorRecord Validation
- Vector dimensions must match the collection schema
- Payload must contain required metadata fields (url, text)
- ID must be unique within the collection

## State Transitions

### Content Processing Flow
1. `Raw HTML` → `ExtractedContent` (after text extraction and cleaning)
2. `ExtractedContent` → `Embedding` (after embedding generation)
3. `Embedding` → `VectorRecord` (after storage in Qdrant)
4. `VectorRecord` → `Queryable` (after successful storage and indexing)

## Schema Examples

### Qdrant Collection Schema
```json
{
  "name": "rag_embeddings",
  "vector_size": 1024,
  "distance": "Cosine",
  "payload_schema": {
    "url": "keyword",
    "section": "keyword",
    "title": "text",
    "text": "text",
    "timestamp": "integer"
  }
}
```

### Content Extraction Payload
```json
{
  "url": "https://hackathon1-five-taupe.vercel.app/docs/intro",
  "section": "Introduction",
  "title": "Getting Started",
  "text": "This is the content text extracted from the page...",
  "timestamp": 1702646400
}
```