# Quickstart: Docusaurus RAG Pipeline

## Prerequisites

- Python 3.11 or higher
- `uv` package manager installed
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Create the backend directory**
   ```bash
   mkdir backend
   cd backend
   ```

3. **Initialize the project with uv**
   ```bash
   uv init
   ```

4. **Install dependencies**
   ```bash
   uv add requests beautifulsoup4 cohere qdrant-client python-dotenv
   ```

5. **Set up environment variables**
   Create a `.env` file in the backend directory with the following:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   DOCUSAURUS_BASE_URL=https://hackathon1-five-taupe.vercel.app/
   ```

## Usage

1. **Run the RAG pipeline**
   ```bash
   cd backend
   python main.py
   ```

2. **The pipeline will execute in this order:**
   - Collect all URLs from the Docusaurus site
   - Extract clean text from each URL
   - Chunk the text appropriately
   - Generate embeddings using Cohere
   - Create the "rag_embeddings" collection in Qdrant
   - Save each chunk to Qdrant
   - Validate with a sample similarity query

## Verification

After running the pipeline, you can verify:

1. Check that the "rag_embeddings" collection exists in your Qdrant Cloud instance
2. Verify that the collection contains the expected number of vectors
3. Test similarity search with a sample query to ensure content is properly indexed