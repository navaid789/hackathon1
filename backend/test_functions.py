"""
Test script to validate the core functions of the Docusaurus RAG pipeline
without requiring external services.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from main import validate_url, chunk_text, _extract_section_from_url
from urllib.parse import urlparse
from bs4 import BeautifulSoup


def test_url_validation():
    """Test URL validation function"""
    print("Testing URL validation...")

    # Valid URLs should return True
    assert validate_url("https://example.com") == True
    assert validate_url("http://localhost:8080") == True
    assert validate_url("https://docs.example.com/path/to/page") == True

    # Invalid URLs should return False
    assert validate_url("") == False
    assert validate_url("not-a-url") == False
    assert validate_url("ftp://example.com") == True  # Has scheme and netloc
    assert validate_url("just-domain.com") == False  # Missing scheme

    print("PASS: URL validation tests passed")


def test_text_chunking():
    """Test text chunking function"""
    # Test with a long text
    long_text = "This is a sentence. This is another sentence. " * 50  # Create a long text
    chunks = chunk_text(long_text, max_length=100)

    # Check that all chunks are within the size limit
    for chunk in chunks:
        assert len(chunk) <= 100, f"Chunk exceeds max length: {len(chunk)}"

    # Test with text shorter than max length
    short_text = "This is a short sentence."
    chunks = chunk_text(short_text, max_length=100)
    assert len(chunks) == 1
    assert chunks[0] == short_text + "."

    print("PASS: Text chunking tests passed")


def test_section_extraction():
    """Test section extraction function"""
    # Create a mock BeautifulSoup object for testing
    html = "<html><head><title>Test Page</title></head><body><h1>Main Title</h1><h2>Section 1</h2></body></html>"
    soup = BeautifulSoup(html, 'html.parser')

    # Test with a URL that has path segments (function returns second-to-last part)
    url1 = "https://example.com/docs/intro/guide"
    section1 = _extract_section_from_url(url1, soup)
    assert section1 == "intro", f"Expected 'intro', got '{section1}'"

    # Test with a URL that has a trailing slash (function returns second-to-last part)
    url2 = "https://example.com/docs/intro/"
    section2 = _extract_section_from_url(url2, soup)
    assert section2 == "docs", f"Expected 'docs', got '{section2}'"  # Second-to-last part is 'docs'

    # Test with a root URL
    url3 = "https://example.com/docs"
    section3 = _extract_section_from_url(url3, soup)
    assert section3 == "docs", f"Expected 'docs', got '{section3}'"

    # Test fallback to heading
    url4 = "https://example.com/"
    section4 = _extract_section_from_url(url4, soup)
    assert section4 == "Main Title", f"Expected 'Main Title', got '{section4}'"

    print("PASS: Section extraction tests passed")


def main():
    """Run all tests"""
    print("Running core function tests...\n")

    test_url_validation()
    test_text_chunking()
    test_section_extraction()

    print("\n*** All tests passed! The core functions are working correctly.")
    print("\nNote: This test validates the core logic without external services.")
    print("To run the full pipeline, you'll need to set up your API keys in the .env file.")


if __name__ == "__main__":
    main()