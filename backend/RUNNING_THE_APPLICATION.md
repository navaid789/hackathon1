# Docusaurus RAG Pipeline - Setup and Execution Guide

## Overview
The Docusaurus RAG Pipeline extracts content from a deployed Docusaurus site, generates embeddings using Cohere, and stores them in Qdrant Cloud for RAG (Retrieval Augmented Generation) applications.

## Files Created
- `backend/main.py` - Main application with all required functionality
- `backend/pyproject.toml` - Project configuration
- `backend/requirements.txt` - Python dependencies
- `backend/.env.example` - Example environment variables file
- `backend/.env` - Environment variables file (you need to fill this)
- `backend/README.md` - Comprehensive documentation
- `backend/test_functions.py` - Test script for core functions

## Prerequisites
1. Python 3.11+
2. Cohere API key
3. Qdrant Cloud account and API key

## Setup Instructions

### 1. Install Dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables
1. Copy the example environment file:
```bash
cp .env.example .env
```

2. Edit the `.env` file and add your credentials:
```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# Docusaurus Site Configuration (default is already set)
DOCUSAURUS_BASE_URL=https://hackathon1-five-taupe.vercel.app/
```

## API Key Sources
- **Cohere API Key**: Sign up at [cohere.com](https://cohere.com) and get your API key from the dashboard
- **Qdrant Cloud**: Sign up at [qdrant.io](https://qdrant.io) for cloud hosting, or use open source version
  - Qdrant Cloud URL format: `https://<cluster-id>.<region>.gcp.cloud.qdrant.io:6333`
  - Get the URL and API key from your Qdrant Cloud dashboard

## Running the Application

### Basic Execution
```bash
cd backend
python main.py
```

### Command-line Options
```bash
python main.py --help
```

Common options:
- `--base-url`: Base URL of the Docusaurus site to process (overrides environment variable)
- `--collection-name`: Name of the Qdrant collection to use (default: rag_embeddings)
- `--chunk-size`: Maximum size of text chunks (default: 1000)
- `--top-k`: Number of results to return in validation search (default: 3)
- `--validation-query`: Query to use for validation search (default: "What is this documentation about?")

### Example Usage
```bash
python main.py --base-url "https://my-docs.example.com" --chunk-size 1500 --top-k 5
```

## Pipeline Flow
1. **URL Collection**: Automatically discovers pages via sitemap.xml or web crawling
2. **Content Extraction**: Extracts clean text content from Docusaurus pages while preserving metadata
3. **Text Chunking**: Splits content into appropriate sizes for embedding generation
4. **Embedding Generation**: Uses Cohere's multilingual embedding model
5. **Vector Storage**: Stores embeddings in Qdrant Cloud with metadata
6. **Validation**: Performs sample similarity search to validate the pipeline

## Testing Core Functions
To test the core functionality without external services:
```bash
python test_functions.py
```

## Qdrant Cloud Free Tier Limits
- Up to 100,000 vectors storage
- The script includes checks to monitor usage
- Will warn when approaching the 10% threshold of the limit

## Troubleshooting
- **Connection errors**: Verify your Qdrant URL and API key are correct
- **Cohere API errors**: Check that your Cohere API key is valid and you have sufficient credits
- **Rate limiting**: The script includes retry logic with exponential backoff
- **Empty results**: Ensure the Docusaurus site is accessible and has crawlable content

## Security Notes
- Never commit your `.env` file with actual API keys
- Use environment variables for all sensitive credentials
- The `.env` file is included in `.gitignore` by default