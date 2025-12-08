"""
Configuration settings for the RAG chatbot backend
"""
import os
from pydantic_settings import BaseSettings
from dotenv import load_dotenv

load_dotenv()

class Settings(BaseSettings):
    # API Keys
    groq_api_key: str = os.getenv("GROQ_API_KEY", "")
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    
    # Qdrant Configuration
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_textbook")
    
    # Database Configuration
    database_url: str = os.getenv("DATABASE_URL", "")
    
    # Server Configuration
    port: int = int(os.getenv("PORT", "8000"))
    host: str = os.getenv("HOST", "0.0.0.0")
    
    # Embedding Configuration
    embedding_model: str = "embed-english-v3.0"
    embedding_dimension: int = 1024
    
    # Chunking Configuration
    chunk_size: int = 1000
    chunk_overlap: int = 200
    
    class Config:
        env_file = ".env"

settings = Settings()
