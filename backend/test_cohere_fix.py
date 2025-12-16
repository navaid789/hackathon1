#!/usr/bin/env python3
"""
Test script to verify the Cohere API fixes are working properly.
"""

import os
import sys
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the backend directory to the path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from main import initialize_cohere_client, embed, chunk_text

def test_cohere_connection():
    """Test basic Cohere client initialization"""
    print("Testing Cohere client initialization...")
    try:
        co = initialize_cohere_client()
        print("+ Cohere client initialized successfully")
        return True
    except Exception as e:
        print(f"- Failed to initialize Cohere client: {e}")
        return False

def test_embedding_function():
    """Test the embedding function with sample text"""
    print("\nTesting embedding function...")
    try:
        sample_text = [
            "This is a test sentence for embedding.",
            "Here is another sentence to test with.",
            "A third sentence for good measure."
        ]

        print(f"Attempting to embed {len(sample_text)} text chunks...")
        embeddings = embed(sample_text)

        print(f"+ Successfully generated {len(embeddings)} embeddings")
        print(f"+ First embedding length: {len(embeddings[0]) if embeddings else 0}")
        return True
    except Exception as e:
        print(f"- Failed to generate embeddings: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_text_chunking():
    """Test the text chunking function"""
    print("\nTesting text chunking function...")
    try:
        long_text = "This is a sentence. " * 100  # Create a longer text
        chunks = chunk_text(long_text, max_length=50)

        print(f"+ Successfully chunked text into {len(chunks)} chunks")
        for i, chunk in enumerate(chunks[:3]):  # Show first 3 chunks
            print(f"  Chunk {i+1}: {len(chunk)} chars")
        return True
    except Exception as e:
        print(f"- Failed to chunk text: {e}")
        return False

def main():
    """Run all tests"""
    print("Running Cohere API fix verification tests...\n")

    tests = [
        ("Cohere Client Initialization", test_cohere_connection),
        ("Text Chunking", test_text_chunking),
        ("Embedding Generation", test_embedding_function),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n--- {test_name} ---")
        result = test_func()
        results.append((test_name, result))

    print(f"\n--- Summary ---")
    passed = sum(1 for _, result in results if result)
    total = len(results)

    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"{test_name}: {status}")

    print(f"\nOverall: {passed}/{total} tests passed")

    if passed == total:
        print("\n*** All tests passed! The Cohere API fixes are working correctly.")
    else:
        print("\n⚠️  Some tests failed. Please check the output above for details.")

    return passed == total

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)