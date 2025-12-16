---
name: create-validate-retrieval-pipeline-spec
model: claude-sonnet-4-5-20250929
description: Validate the retrieval pipeline by querying the vector database and ensuring extracted book content can be accurately retrieved for downstream RAG usage.
---

### Context
Validate the retrieval pipeline by querying the vector database and ensuring extracted book content can be accurately retrieved for downstream RAG usage.

### Target Audience
- AI engineers
- RAG system developers

### Objectives
- Retrieve relevant chunks from Qdrant using vector similarity search
- Verify metadata integrity and content relevance
- Test end-to-end retrieval from query to results

### Success Criteria
- Queries return semantically relevant book content
- Retrieved chunks include correct metadata (URL, section)
- Retrieval latency is acceptable
- No errors in the retrieval pipeline

### Constraints
- Vector database: Qdrant Cloud
- Embeddings: Cohere
- Use previously ingested data only
