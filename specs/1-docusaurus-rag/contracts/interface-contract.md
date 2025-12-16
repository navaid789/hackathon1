# Interface Contract: Docusaurus RAG Pipeline

## Function Signatures

### get_all_urls(base_url: str) -> List[str]
- **Purpose**: Collect all accessible URLs from the Docusaurus site
- **Input**: Base URL of the Docusaurus site to crawl
- **Output**: List of all discovered URLs
- **Errors**: Returns empty list if crawling fails
- **Side effects**: None

### extract_text_from_url(url: str) -> Dict
- **Purpose**: Extract clean text content from a given URL
- **Input**: URL to extract content from
- **Output**: Dictionary with keys: 'url', 'title', 'text', 'section'
- **Errors**: Returns None if extraction fails
- **Side effects**: None

### chunk_text(text: str, max_length: int = 1000) -> List[str]
- **Purpose**: Split text into smaller chunks for embedding
- **Input**: Text to chunk and maximum chunk length
- **Output**: List of text chunks
- **Errors**: Returns original text if chunking fails
- **Side effects**: None

### embed(text_chunks: List[str]) -> List[List[float]]
- **Purpose**: Generate embeddings for text chunks using Cohere
- **Input**: List of text chunks to embed
- **Output**: List of embedding vectors (lists of floats)
- **Errors**: Raises exception if Cohere API fails
- **Side effects**: Makes API calls to Cohere service

### create_collection(collection_name: str) -> bool
- **Purpose**: Create a Qdrant collection for storing embeddings
- **Input**: Name of the collection to create
- **Output**: Boolean indicating success
- **Errors**: Raises exception if collection creation fails
- **Side effects**: Creates collection in Qdrant Cloud

### save_chunk_to_qdrant(chunk_data: Dict, embedding: List[float]) -> bool
- **Purpose**: Save a text chunk with its embedding to Qdrant
- **Input**: Dictionary with chunk data and its embedding vector
- **Output**: Boolean indicating success
- **Errors**: Raises exception if storage fails
- **Side effects**: Adds vector record to Qdrant Cloud

## Configuration Contract

### Environment Variables
- `COHERE_API_KEY`: Required API key for Cohere service
- `QDRANT_URL`: Required URL endpoint for Qdrant Cloud instance
- `QDRANT_API_KEY`: Required API key for Qdrant Cloud
- `DOCUSAURUS_BASE_URL`: Required base URL of the Docusaurus site to process

## Data Contract

### Input Data Format
- URLs must be valid HTTP/HTTPS addresses
- Text content must be in UTF-8 encoding
- Chunk sizes must be within Cohere API limits (max ~2048 tokens)

### Output Data Format
- Embeddings are returned as lists of floats
- Qdrant records include metadata: URL, section, title, and original text
- Collection named "rag_embeddings" is created in Qdrant Cloud

## Performance Contract
- Processing time scales linearly with number of pages
- Embedding generation rate depends on Cohere API limits
- Qdrant storage rate depends on network and service limits