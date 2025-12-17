"""
Configuration settings for the AI Agent with FastAPI application.
"""
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # OpenAI settings
    openai_api_key: str
    openai_model: str = "gpt-4-turbo"

    # Qdrant settings
    qdrant_api_key: str
    qdrant_url: str

    # Cohere settings
    cohere_api_key: str

    # Application settings
    app_name: str = "AI Agent with FastAPI"
    app_version: str = "1.0.0"
    debug: bool = False
    log_level: str = "INFO"

    # Agent settings
    agent_instructions: str = "You are an AI assistant that answers questions based on retrieved book content. Always cite your sources from the retrieved content."

    # Qdrant collection name
    qdrant_collection_name: str = "rag_embeddings"

    # Docusaurus site configuration (for RAG pipeline)
    docusaurus_base_url: str = "https://hackathon1-five-taupe.vercel.app/"

    class Config:
        env_file = ".env"


settings = Settings()