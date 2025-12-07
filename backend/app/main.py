"""
FastAPI application for RAG chatbot backend
"""
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from app.agent import RAGAgent
from app.vector_db import VectorDB
from app.data_extractor import DataExtractor
from app.config import settings
import uvicorn

app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="RAG chatbot API for Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
agent = RAGAgent()

# Request/Response Models
class QuestionRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None

class QuestionResponse(BaseModel):
    answer: str
    context_used: Optional[str] = None
    model: str

class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    messages: List[ChatMessage]

class ChatResponse(BaseModel):
    response: str

class IndexRequest(BaseModel):
    urls: Optional[List[str]] = None
    sitemap_url: Optional[str] = None

class SearchRequest(BaseModel):
    query: str
    limit: int = 5

class SearchResponse(BaseModel):
    results: List[Dict[str, Any]]

# Routes
@app.get("/")
async def root():
    return {
        "message": "Physical AI Textbook RAG API",
        "version": "1.0.0",
        "docs": "/docs"
    }

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

@app.post("/ask", response_model=QuestionResponse)
async def ask_question(request: QuestionRequest):
    """
    Answer a question using RAG.
    Supports optional selected_text for context-aware answers.
    """
    try:
        result = agent.answer_question(
            question=request.question,
            selected_text=request.selected_text
        )
        
        if result.get("error"):
            raise HTTPException(status_code=500, detail=result["answer"])
        
        return QuestionResponse(
            answer=result["answer"],
            context_used=result.get("context_used"),
            model=result["model"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Multi-turn conversation endpoint
    """
    try:
        messages = [msg.dict() for msg in request.messages]
        response = agent.chat(messages)
        return ChatResponse(response=response)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/search", response_model=SearchResponse)
async def search(request: SearchRequest):
    """
    Search for relevant content in the vector database
    """
    try:
        results = agent.vector_db.search(request.query, limit=request.limit)
        return SearchResponse(results=results)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/index")
async def index_documents(request: IndexRequest):
    """
    Index documents from URLs or sitemap into the vector database
    """
    try:
        vector_db = VectorDB()
        
        # Create collection if it doesn't exist
        vector_db.create_collection()
        
        urls = []
        if request.sitemap_url:
            urls = DataExtractor.get_urls_from_sitemap(request.sitemap_url)
        elif request.urls:
            urls = request.urls
        else:
            raise HTTPException(
                status_code=400,
                detail="Either urls or sitemap_url must be provided"
            )
        
        if not urls:
            raise HTTPException(status_code=400, detail="No URLs found to index")
        
        # Process documents
        documents = DataExtractor.process_documents(urls)
        
        if not documents:
            raise HTTPException(status_code=400, detail="No content extracted from URLs")
        
        # Add to vector database
        vector_db.add_documents(documents)
        
        return {
            "message": "Documents indexed successfully",
            "urls_processed": len(urls),
            "chunks_created": len(documents)
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.delete("/collection")
async def delete_collection():
    """
    Delete the vector database collection
    """
    try:
        vector_db = VectorDB()
        vector_db.delete_collection()
        return {"message": "Collection deleted successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host=settings.host,
        port=settings.port,
        reload=True
    )
