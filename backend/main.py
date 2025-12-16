"""
Docusaurus RAG Pipeline

This script extracts content from a deployed Docusaurus site,
generates embeddings using Cohere, and stores them in Qdrant Cloud.
"""

import os
import requests
import time
import logging
from typing import List, Dict, Optional, Tuple
from urllib.parse import urljoin, urlparse
from dataclasses import dataclass

import bs4
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


# Load environment variables
load_dotenv()


@dataclass
class Config:
    """Configuration class to manage API keys and URLs"""
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    docusaurus_base_url: str = "https://hackathon1-five-taupe.vercel.app/"

    @classmethod
    def from_env(cls) -> 'Config':
        """Create Config instance from environment variables"""
        return cls(
            cohere_api_key=os.getenv('COHERE_API_KEY'),
            qdrant_url=os.getenv('QDRANT_URL'),
            qdrant_api_key=os.getenv('QDRANT_API_KEY'),
            docusaurus_base_url=os.getenv('DOCUSAURUS_BASE_URL', 'https://hackathon1-five-taupe.vercel.app/')
        )


def setup_logging():
    """Set up logging configuration for the application"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('rag_pipeline.log'),
            logging.StreamHandler()
        ]
    )


def validate_config(config: Config) -> Tuple[bool, List[str]]:
    """Validate configuration parameters"""
    errors = []

    if not config.cohere_api_key:
        errors.append("COHERE_API_KEY is required")
    if not config.qdrant_url:
        errors.append("QDRANT_URL is required")
    if not config.qdrant_api_key:
        errors.append("QDRANT_API_KEY is required")

    return len(errors) == 0, errors


def initialize_qdrant_client(config: Config) -> QdrantClient:
    """Initialize Qdrant client with error handling"""
    try:
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            timeout=30
        )
        # Test connection
        client.get_collections()
        return client
    except Exception as e:
        raise ConnectionError(f"Failed to connect to Qdrant: {e}")


def validate_url(url: str) -> bool:
    """Validate if a URL is properly formatted"""
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except:
        return False


def retry_on_failure(max_retries: int = 3, delay: float = 1.0):
    """Decorator to retry a function on failure"""
    def decorator(func):
        def wrapper(*args, **kwargs):
            last_exception = None
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt < max_retries - 1:
                        time.sleep(delay * (2 ** attempt))  # Exponential backoff
                        continue
                    else:
                        raise last_exception
            return None
        return wrapper
    return decorator


@retry_on_failure(max_retries=3, delay=1.0)
def get_all_urls(base_url: str) -> List[str]:
    """
    Collect all accessible URLs from the Docusaurus site.
    First tries to parse sitemap.xml, then falls back to web crawling.
    """
    logger = logging.getLogger(__name__)

    if not validate_url(base_url):
        raise ValueError(f"Invalid base URL: {base_url}")

    urls = set()

    # Try to get URLs from sitemap first
    try:
        sitemap_url = urljoin(base_url, 'sitemap.xml')
        sitemap_urls = _get_urls_from_sitemap(sitemap_url)
        urls.update(sitemap_urls)
        logger.info(f"Found {len(sitemap_urls)} URLs from sitemap")
    except Exception as e:
        logger.warning(f"Sitemap parsing failed: {e}. Falling back to web crawling.")

    # If sitemap is not available or failed, crawl the site
    if not urls:
        crawled_urls = _crawl_site(base_url)
        urls.update(crawled_urls)
        logger.info(f"Found {len(crawled_urls)} URLs through crawling")

    return list(urls)


def check_qdrant_limits(collection_name: str = "rag_embeddings") -> Dict[str, int]:
    """
    Check Qdrant Cloud limits to ensure we're within Free Tier constraints.
    """
    logger = logging.getLogger(__name__)

    config = Config.from_env()
    client = initialize_qdrant_client(config)

    try:
        # Get collection info to check vector count
        collection_info = client.get_collection(collection_name)
        vector_count = collection_info.points_count

        # Qdrant Cloud Free Tier limits
        max_vectors = 100000  # Free tier allows up to 100K vectors
        remaining = max_vectors - vector_count

        limits = {
            "current_count": vector_count,
            "max_allowed": max_vectors,
            "remaining": remaining,
            "approaching_limit": remaining < max_vectors * 0.1  # Alert if within 10% of limit
        }

        if limits["approaching_limit"]:
            logger.warning(f"Approaching Qdrant Cloud Free Tier limit: {vector_count}/{max_vectors} vectors used")

        return limits
    except Exception as e:
        logger.error(f"Failed to check Qdrant limits: {e}")
        return {"error": str(e)}


def _get_urls_from_sitemap(sitemap_url: str) -> List[str]:
    """Parse sitemap.xml to extract URLs"""
    response = requests.get(sitemap_url)
    response.raise_for_status()

    soup = BeautifulSoup(response.content, 'xml')
    urls = []

    # Look for <url><loc> elements in the sitemap
    for url_elem in soup.find_all('loc'):
        urls.append(url_elem.text.strip())

    # Also look for sitemap references (sitemap index files)
    for sitemap_elem in soup.find_all('sitemap'):
        loc_elem = sitemap_elem.find('loc')
        if loc_elem:
            nested_sitemap_url = loc_elem.text.strip()
            try:
                nested_urls = _get_urls_from_sitemap(nested_sitemap_url)
                urls.extend(nested_urls)
            except:
                continue  # Skip if nested sitemap fails

    return urls


def _crawl_site(base_url: str) -> List[str]:
    """Crawl the site by following internal links"""
    logger = logging.getLogger(__name__)
    urls = set()
    to_visit = [base_url]
    visited = set()

    # Parse base URL to determine domain for internal link filtering
    base_domain = urlparse(base_url).netloc

    while to_visit:
        current_url = to_visit.pop(0)

        if current_url in visited:
            continue

        visited.add(current_url)

        try:
            response = requests.get(current_url, timeout=10)
            response.raise_for_status()

            # Add current URL to our list
            urls.add(current_url)

            # Parse for additional internal links
            soup = BeautifulSoup(response.content, 'html.parser')

            # Find all links
            for link in soup.find_all('a', href=True):
                href = link['href']

                # Convert relative URLs to absolute
                absolute_url = urljoin(current_url, href)

                # Only follow links from the same domain
                if urlparse(absolute_url).netloc == base_domain:
                    # Only add URLs that are likely to be pages (not external links, etc.)
                    if absolute_url.startswith(base_url) and absolute_url not in visited:
                        to_visit.append(absolute_url)

        except Exception as e:
            logger.warning(f"Failed to crawl {current_url}: {e}")
            continue

    return list(urls)


def extract_text_from_url(url: str) -> Optional[Dict]:
    """
    Extract clean text content from a given URL with metadata.
    Returns a dictionary with keys: 'url', 'title', 'text', 'section'
    """
    logger = logging.getLogger(__name__)

    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Extract title
        title = ''
        title_elem = soup.find('title')
        if title_elem:
            title = title_elem.get_text().strip()
        else:
            # Look for h1 as title alternative
            h1_elem = soup.find('h1')
            if h1_elem:
                title = h1_elem.get_text().strip()

        # Extract main content by looking for Docusaurus-specific selectors
        # Docusaurus typically uses these selectors for main content
        main_content = None

        # Try common Docusaurus content selectors
        selectors = [
            '[class*="docItemContainer"]',
            '[class*="markdown"]',
            '[role="main"]',
            'main',
            '[class*="container"]',
            '[class*="content"]',
            '.main-wrapper',
            '.theme-doc-markdown'
        ]

        for selector in selectors:
            main_content = soup.select_one(selector)
            if main_content:
                break

        # If no specific selector worked, use the body
        if not main_content:
            main_content = soup.find('body')

        if not main_content:
            return None

        # Remove navigation, headers, footers, and sidebar elements
        for element in main_content.find_all(['nav', 'header', 'footer', 'aside']):
            element.decompose()

        # Remove script and style elements
        for element in main_content(['script', 'style', 'noscript']):
            element.decompose()

        # Get clean text
        text = main_content.get_text(separator=' ', strip=True)

        # Clean up excessive whitespace
        import re
        text = re.sub(r'\s+', ' ', text)

        # Extract section information (could be from headings or URL structure)
        section = _extract_section_from_url(url, soup)

        return {
            'url': url,
            'title': title,
            'text': text,
            'section': section
        }

    except Exception as e:
        logger.error(f"Failed to extract text from {url}: {e}")
        return None


def _extract_section_from_url(url: str, soup: BeautifulSoup) -> str:
    """Extract section information from URL or page content"""
    # Try to get section from URL path
    parsed_url = urlparse(url)
    path_parts = parsed_url.path.strip('/').split('/')

    # If there are path parts, the second-to-last part is often the section
    if len(path_parts) > 1:
        return path_parts[-2] if path_parts[-1] else path_parts[-2]
    elif len(path_parts) == 1 and path_parts[0]:
        return path_parts[0]
    else:
        # Fallback: try to get from page headings
        h1 = soup.find('h1')
        h2 = soup.find('h2')
        if h1:
            return h1.get_text().strip()
        elif h2:
            return h2.get_text().strip()
        else:
            return "General"




def chunk_text(text: str, max_length: int = 1000) -> List[str]:
    """
    Split text into smaller chunks for embedding.
    Uses a simple approach to split by sentences while respecting max_length.
    """
    import re

    # Split by sentences
    sentences = re.split(r'[.!?]+\s+', text)

    chunks = []
    current_chunk = ""

    for sentence in sentences:
        # If adding the next sentence would exceed max length
        if len(current_chunk) + len(sentence) > max_length:
            # If current chunk is not empty, save it
            if current_chunk.strip():
                chunks.append(current_chunk.strip())

            # If sentence is longer than max_length, split it
            if len(sentence) > max_length:
                # Split long sentence into smaller parts
                for i in range(0, len(sentence), max_length):
                    chunks.append(sentence[i:i+max_length])
            else:
                current_chunk = sentence + ". "
        else:
            current_chunk += sentence + ". "

    # Add the last chunk if it exists
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def initialize_cohere_client() -> cohere.Client:
    """Initialize Cohere client with API key from environment"""
    cohere_api_key = os.getenv('COHERE_API_KEY')
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable is required")

    return cohere.Client(cohere_api_key)


def embed(text_chunks: List[str]) -> List[List[float]]:
    """
    Generate embeddings for text chunks using Cohere with enhanced error handling and rate limiting.
    """
    import time
    from cohere.errors import BadRequestError

    logger = logging.getLogger(__name__)

    # Initialize Cohere client
    co = initialize_cohere_client()

    try:
        # Process in batches to respect API limits (Cohere allows up to 96 texts per request)
        batch_size = 10  # Further reduced batch size to be more conservative with trial account
        all_embeddings = []

        for i in range(0, len(text_chunks), batch_size):
            batch = text_chunks[i:i + batch_size]

            # Add a small delay between batches to avoid rate limiting
            if i > 0:
                time.sleep(1.0)  # Increased delay to be more conservative with trial account

            # Retry logic for handling rate limits and other temporary issues
            max_retries = 3
            retry_count = 0

            while retry_count < max_retries:
                try:
                    # Using a more standard model name that's available in trial accounts
                    response = co.embed(
                        texts=batch,
                        model="embed-english-v3.0",  # Changed to English model which is typically more available in trials
                        input_type="search_document"  # Added input_type which is required for newer models
                    )

                    all_embeddings.extend(response.embeddings)
                    break  # Success, exit retry loop

                except BadRequestError as e:
                    logger.warning(f"Cohere API error (attempt {retry_count + 1}/{max_retries}): {e}")

                    # Log the response body for better debugging
                    if hasattr(e, 'body'):
                        logger.error(f"BadRequestError body: {e.body}")

                    # Check if it's a rate limit error
                    if "rate limit" in str(e).lower() or e.status_code == 429 or "limit" in str(e).lower():
                        wait_time = (2 ** retry_count) + 2  # Increased base wait time
                        logger.info(f"Rate limited. Waiting {wait_time}s before retry...")
                        time.sleep(wait_time)
                        retry_count += 1
                    else:
                        # For other BadRequestErrors, log details and raise
                        logger.error(f"BadRequestError with details: {e.body if hasattr(e, 'body') else str(e)}")
                        raise e
                except Exception as e:
                    logger.error(f"Unexpected error during embedding (attempt {retry_count + 1}/{max_retries}): {e}")
                    if retry_count >= max_retries - 1:
                        raise e
                    retry_count += 1
                    wait_time = (2 ** retry_count) + 2  # Increased base wait time
                    time.sleep(wait_time)

        return all_embeddings
    except Exception as e:
        logger.error(f"Failed to generate embeddings: {e}")
        raise




def create_collection(collection_name: str = "rag_embeddings") -> bool:
    """
    Create a Qdrant collection for storing embeddings.
    Defines the schema with required payload fields (url, section, title, text, timestamp).
    """
    logger = logging.getLogger(__name__)

    config = Config.from_env()
    client = initialize_qdrant_client(config)

    try:
        # Check if collection already exists
        existing_collections = client.get_collections()
        collection_exists = any(col.name == collection_name for col in existing_collections.collections)

        if collection_exists:
            logger.info(f"Collection '{collection_name}' already exists")
            return True

        # Create collection with appropriate vector size (Cohere's multilingual model returns 1024-dim vectors)
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
        )

        # Set up payload index for efficient filtering
        client.create_payload_index(
            collection_name=collection_name,
            field_name="url",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

        client.create_payload_index(
            collection_name=collection_name,
            field_name="section",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

        client.create_payload_index(
            collection_name=collection_name,
            field_name="title",
            field_schema=models.PayloadSchemaType.TEXT
        )

        logger.info(f"Created collection '{collection_name}' successfully with payload indexes")
        return True
    except Exception as e:
        logger.error(f"Failed to create collection '{collection_name}': {e}")
        return False


def save_chunk_to_qdrant(chunk_data: Dict, embedding: List[float], collection_name: str = "rag_embeddings") -> bool:
    """
    Save a text chunk with its embedding to Qdrant.
    """
    logger = logging.getLogger(__name__)

    config = Config.from_env()
    client = initialize_qdrant_client(config)

    try:
        # Prepare the payload with metadata
        payload = {
            "url": chunk_data.get('url', ''),
            "title": chunk_data.get('title', ''),
            "section": chunk_data.get('section', ''),
            "text": chunk_data.get('text', ''),
            "timestamp": chunk_data.get('timestamp', int(time.time()))
        }

        # Generate a unique ID for the record
        import hashlib
        content_id = hashlib.md5(f"{chunk_data.get('url', '')}_{chunk_data.get('text', '')}".encode()).hexdigest()

        # Upsert the record to Qdrant
        client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=content_id,
                    vector=embedding,
                    payload=payload
                )
            ]
        )

        logger.info(f"Saved chunk to Qdrant: {content_id}")
        return True
    except Exception as e:
        logger.error(f"Failed to save chunk to Qdrant: {e}")
        return False


def search_similar_content(query_text: str, top_k: int = 5, collection_name: str = "rag_embeddings") -> List[Dict]:
    """
    Perform similarity search in Qdrant to find relevant content.
    """
    logger = logging.getLogger(__name__)

    config = Config.from_env()
    client = initialize_qdrant_client(config)

    try:
        # Initialize Cohere client to embed the query
        co = initialize_cohere_client()

        # Embed the query text
        query_embedding = co.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"  # Using query input type for search queries
        ).embeddings[0]

        # Perform similarity search
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        # Format results
        results = []
        for result in search_results:
            results.append({
                'id': result.id,
                'score': result.score,
                'payload': result.payload
            })

        logger.info(f"Found {len(results)} similar content items")
        return results

    except Exception as e:
        logger.error(f"Failed to perform similarity search: {e}")
        return []


def main():
    """
    Main function to execute the complete RAG pipeline:
    1. Collect all URLs from the Docusaurus site
    2. Extract clean text from each URL
    3. Chunk the text appropriately
    4. Generate embeddings using Cohere
    5. Create the "rag_embeddings" collection in Qdrant
    6. Save each chunk to Qdrant
    7. Validate with a sample similarity query
    """
    import argparse

    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description='Docusaurus RAG Pipeline')
    parser.add_argument('--base-url', type=str, default=None,
                        help='Base URL of the Docusaurus site to process (default: from environment)')
    parser.add_argument('--collection-name', type=str, default='rag_embeddings',
                        help='Name of the Qdrant collection to use (default: rag_embeddings)')
    parser.add_argument('--chunk-size', type=int, default=1000,
                        help='Maximum size of text chunks (default: 1000)')
    parser.add_argument('--top-k', type=int, default=3,
                        help='Number of results to return in validation search (default: 3)')
    parser.add_argument('--validation-query', type=str, default='What is this documentation about?',
                        help='Query to use for validation search (default: "What is this documentation about?")')
    args = parser.parse_args()

    # Set up logging
    setup_logging()
    logger = logging.getLogger(__name__)

    logger.info("Starting Docusaurus RAG Pipeline...")

    # Initialize configuration
    config = Config.from_env()
    if args.base_url:
        config = Config(
            cohere_api_key=config.cohere_api_key,
            qdrant_url=config.qdrant_url,
            qdrant_api_key=config.qdrant_api_key,
            docusaurus_base_url=args.base_url
        )

    # Validate configuration
    is_valid, errors = validate_config(config)
    if not is_valid:
        raise ValueError(f"Configuration validation failed: {'; '.join(errors)}")

    logger.info(f"Processing Docusaurus site: {config.docusaurus_base_url}")

    # Check Qdrant Cloud limits before starting
    limits = check_qdrant_limits(args.collection_name)
    if "error" not in limits and limits.get("approaching_limit"):
        logger.warning(f"Qdrant limits check: {limits}")

    # Step 0: Create Qdrant collection
    logger.info("Step 0: Setting up Qdrant collection...")
    if not create_collection(args.collection_name):
        raise RuntimeError("Failed to create Qdrant collection")
    logger.info("Qdrant collection created successfully")

    # Execute pipeline steps
    # Step 1: Collect all URLs
    logger.info("Step 1: Collecting URLs...")
    urls = get_all_urls(config.docusaurus_base_url)
    logger.info(f"Collected {len(urls)} URLs")

    # Step 2: Extract content from each URL
    logger.info("Step 2: Extracting content...")
    extracted_contents = []
    for i, url in enumerate(urls):
        logger.info(f"Processing {i+1}/{len(urls)}: {url}")
        try:
            content = extract_text_from_url(url)
            if content:
                extracted_contents.append(content)
        except Exception as e:
            logger.error(f"Failed to extract content from {url}: {e}")

    logger.info(f"Extracted content from {len(extracted_contents)} pages")

    # Step 3: Process content and generate embeddings
    logger.info("Step 3: Generating embeddings...")
    total_chunks = 0
    for content in extracted_contents:
        text_chunks = chunk_text(content['text'], max_length=args.chunk_size)
        embeddings = embed(text_chunks)  # Generate embeddings for all chunks at once

        for chunk, embedding in zip(text_chunks, embeddings):
            # Save to Qdrant
            success = save_chunk_to_qdrant({
                'url': content['url'],
                'title': content['title'],
                'section': content['section'],
                'text': chunk,
                'timestamp': int(time.time())
            }, embedding, collection_name=args.collection_name)

            if success:
                total_chunks += 1

    logger.info(f"Successfully processed and stored {total_chunks} text chunks")

    # Step 4: Validate with sample similarity query
    logger.info("Step 4: Validating with sample similarity query...")
    search_results = search_similar_content(args.validation_query, top_k=args.top_k, collection_name=args.collection_name)

    if search_results:
        logger.info(f"Sample search found {len(search_results)} relevant results:")
        for i, result in enumerate(search_results):
            logger.info(f"  {i+1}. Score: {result['score']:.3f}, URL: {result['payload']['url'][:50]}...")
    else:
        logger.warning("No results found for validation query. Pipeline may need more content.")

    # Final Qdrant limits check
    final_limits = check_qdrant_limits(args.collection_name)
    if "error" not in final_limits:
        logger.info(f"Final Qdrant limits: {final_limits['current_count']} vectors stored")

    logger.info("Pipeline completed successfully!")


if __name__ == "__main__":
    main()