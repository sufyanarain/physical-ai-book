"""
FastAPI application for RAG chatbot backend
"""
from fastapi import FastAPI, HTTPException, Depends, Header
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, EmailStr
from typing import List, Optional, Dict, Any
from groq import Groq
from app.agent import RAGAgent
from app.vector_db import VectorDB
from app.data_extractor import DataExtractor
from app.config import settings
from app.database import db
from app.auth import (
    get_password_hash, 
    authenticate_user, 
    create_access_token,
    get_current_user
)
import uvicorn
import logging

logger = logging.getLogger(__name__)

# Initialize Groq client for personalization
groq_client = Groq(api_key=settings.groq_api_key)

app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="RAG chatbot API for Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# CORS middleware
# Allow specific origins for better security and reliability
allowed_origins = [
    "http://localhost:3000",  # Local development
    "http://localhost:5173",  # Vite dev server
    "http://127.0.0.1:3000",
    "http://127.0.0.1:5173",
    "https://sufyanarain.github.io",  # GitHub Pages
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=False,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["*"],
    max_age=3600,  # Cache preflight requests for 1 hour
)

# Lazy initialization - initialize on first use to prevent startup crashes
agent = None

def get_agent():
    """Get or initialize the RAG agent"""
    global agent
    if agent is None:
        try:
            agent = RAGAgent()
        except Exception as e:
            raise HTTPException(
                status_code=503,
                detail=f"Failed to initialize RAG agent: {str(e)}. Please check environment variables (GROQ_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)."
            )
    return agent

# Initialize database on startup
@app.on_event("startup")
async def startup_event():
    """Initialize database connection on startup"""
    try:
        db.initialize()
    except Exception as e:
        print(f"Warning: Database initialization failed: {e}")
        print("Continuing without database features...")

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

class TranslateRequest(BaseModel):
    content: str
    target_language: str = "urdu"

class TranslateResponse(BaseModel):
    original: str
    translated: str
    target_language: str

class UserPreferenceRequest(BaseModel):
    session_id: str
    language_preference: Optional[str] = None
    personalization_level: Optional[str] = None
    background: Optional[str] = None

class UserPreferenceResponse(BaseModel):
    session_id: str
    language_preference: str
    personalization_level: str
    background: Optional[str] = None

class SignupRequest(BaseModel):
    email: EmailStr
    name: str
    password: str
    software_experience: str = 'beginner'
    hardware_experience: str = 'beginner'
    learning_goals: Optional[str] = None

class LoginRequest(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: Dict[str, Any]

class UserResponse(BaseModel):
    id: int
    email: str
    name: str
    software_experience: str
    hardware_experience: str
    learning_goals: Optional[str]

class PersonalizeRequest(BaseModel):
    content: str
    software_experience: str = 'beginner'
    hardware_experience: str = 'beginner'
    learning_goals: Optional[str] = None

class PersonalizeResponse(BaseModel):
    original_content: str
    personalized_content: str
    user_level: str

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

# Authentication endpoints
@app.post("/auth/signup", response_model=AuthResponse)
async def signup(request: SignupRequest):
    """
    Create a new user account with background information
    """
    try:
        # Hash the password
        password_hash = get_password_hash(request.password)
        
        # Create user
        user = db.create_user(
            email=request.email,
            name=request.name,
            password_hash=password_hash,
            software_exp=request.software_experience,
            hardware_exp=request.hardware_experience,
            learning_goals=request.learning_goals
        )
        
        if not user:
            raise HTTPException(status_code=400, detail="Email already registered")
        
        # Create access token
        access_token = create_access_token(data={"sub": str(user.id)})
        
        return AuthResponse(
            access_token=access_token,
            user={
                "id": user.id,
                "email": user.email,
                "name": user.name,
                "software_experience": user.software_experience,
                "hardware_experience": user.hardware_experience,
                "learning_goals": user.learning_goals
            }
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Signup error: {str(e)}")

@app.post("/auth/login", response_model=AuthResponse)
async def login(request: LoginRequest):
    """
    Login with email and password
    """
    try:
        user = authenticate_user(request.email, request.password)
        
        if not user:
            raise HTTPException(status_code=401, detail="Invalid email or password")
        
        # Create access token
        access_token = create_access_token(data={"sub": str(user.id)})
        
        return AuthResponse(
            access_token=access_token,
            user={
                "id": user.id,
                "email": user.email,
                "name": user.name,
                "software_experience": user.software_experience,
                "hardware_experience": user.hardware_experience,
                "learning_goals": user.learning_goals
            }
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Login error: {str(e)}")

@app.get("/auth/me", response_model=UserResponse)
async def get_me(authorization: Optional[str] = Header(None)):
    """
    Get current user information
    """
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Not authenticated")
    
    token = authorization.replace("Bearer ", "")
    user = get_current_user(token)
    
    if not user:
        raise HTTPException(status_code=401, detail="Invalid or expired token")
    
    return UserResponse(
        id=user.id,
        email=user.email,
        name=user.name,
        software_experience=user.software_experience,
        hardware_experience=user.hardware_experience,
        learning_goals=user.learning_goals
    )

@app.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest, authorization: Optional[str] = Header(None)):
    """
    Personalize content based on user's experience level and learning goals
    """
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Not authenticated")
    
    token = authorization.replace("Bearer ", "")
    user = get_current_user(token)
    
    if not user:
        raise HTTPException(status_code=401, detail="Invalid or expired token")
    
    try:
        # Use user's actual experience levels
        software_exp = user.software_experience or request.software_experience
        hardware_exp = user.hardware_experience or request.hardware_experience
        learning_goals = user.learning_goals or request.learning_goals or ""
        
        # Determine overall experience level for display
        exp_levels = {"beginner": 0, "intermediate": 1, "advanced": 2}
        avg_level = (exp_levels.get(software_exp, 0) + exp_levels.get(hardware_exp, 0)) / 2
        if avg_level < 0.5:
            overall_level = "beginner"
        elif avg_level < 1.5:
            overall_level = "intermediate"
        else:
            overall_level = "advanced"
        
        # Create personalization prompt
        system_prompt = f"""You are an expert educator personalizing educational content about Physical AI and Humanoid Robotics.

User Profile:
- Software Experience: {software_exp}
- Hardware Experience: {hardware_exp}
- Learning Goals: {learning_goals}

Your task is to adapt the following content to match the user's experience level:

For BEGINNERS:
- Add more explanations of basic concepts
- Include analogies and real-world examples
- Break down complex topics into smaller steps
- Define technical terms
- Add "Why this matters" sections

For INTERMEDIATE learners:
- Assume basic knowledge
- Focus on practical applications
- Include best practices and common pitfalls
- Add references to related concepts
- Include more technical details

For ADVANCED learners:
- Assume strong foundational knowledge
- Dive deep into implementation details
- Include optimization techniques
- Reference research papers and advanced topics
- Discuss trade-offs and design decisions

Maintain the original structure and code examples, but adjust explanations and add context appropriate for the user's level.
Preserve all formatting, code blocks, and markdown syntax."""
        
        user_prompt = f"""Personalize this content for a {overall_level} level learner:

{request.content}

Remember to:
1. Keep all code examples intact
2. Maintain markdown formatting
3. Adjust explanation depth to match user level
4. Add relevant context based on their learning goals
5. Keep the same general structure"""

        # Call Groq API for personalization
        response = groq_client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=4000,
        )
        
        personalized_content = response.choices[0].message.content
        
        return PersonalizeResponse(
            original_content=request.content[:200] + "..." if len(request.content) > 200 else request.content,
            personalized_content=personalized_content,
            user_level=overall_level
        )
    
    except Exception as e:
        logger.error(f"Personalization error: {e}")
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")

@app.post("/ask", response_model=QuestionResponse)
async def ask_question(request: QuestionRequest):
    """
    Answer a question using RAG.
    Supports optional selected_text for context-aware answers.
    """
    try:
        result = get_agent().answer_question(
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
        response = get_agent().chat(messages)
        return ChatResponse(response=response)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/search", response_model=SearchResponse)
async def search(request: SearchRequest):
    """
    Search for relevant content in the vector database
    """
    try:
        results = get_agent().vector_db.search(request.query, limit=request.limit)
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

@app.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate content to target language using Groq LLM
    """
    try:
        from groq import Groq
        from app.config import settings

        client = Groq(api_key=settings.groq_api_key)

        # Use LLM for high-quality translation
        response = client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=[
                {
                    "role": "system",
                    "content": f"""You are an expert translator specializing in technical documentation.
Translate the following technical content about Physical AI and Robotics to {request.target_language}.

Guidelines:
1. Maintain technical accuracy
2. Keep code blocks, syntax, and technical terms intact
3. Translate explanatory text naturally
4. Preserve markdown formatting
5. Keep proper nouns and technical acronyms in English (ROS 2, NVIDIA Isaac, etc.)
6. Ensure the translation is culturally appropriate and technically precise

Return ONLY the translated text without any preamble or explanation."""
                },
                {
                    "role": "user",
                    "content": request.content
                }
            ],
            temperature=0.3,
            max_tokens=4000
        )

        translated_text = response.choices[0].message.content

        return TranslateResponse(
            original=request.content,
            translated=translated_text,
            target_language=request.target_language
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation error: {str(e)}")

@app.post("/preferences", response_model=UserPreferenceResponse)
async def save_preferences(request: UserPreferenceRequest):
    """
    Save user preferences to database
    """
    try:
        pref = db.save_user_preference(
            session_id=request.session_id,
            language=request.language_preference,
            personalization=request.personalization_level,
            background=request.background
        )
        
        if not pref:
            raise HTTPException(status_code=500, detail="Failed to save preferences")
        
        return UserPreferenceResponse(
            session_id=pref.session_id,
            language_preference=pref.language_preference,
            personalization_level=pref.personalization_level,
            background=pref.background
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error saving preferences: {str(e)}")

@app.get("/preferences/{session_id}", response_model=UserPreferenceResponse)
async def get_preferences(session_id: str):
    """
    Get user preferences from database
    """
    try:
        pref = db.get_user_preference(session_id)
        
        if not pref:
            # Return defaults if no preferences found
            return UserPreferenceResponse(
                session_id=session_id,
                language_preference="english",
                personalization_level="intermediate",
                background=None
            )
        
        return UserPreferenceResponse(
            session_id=pref.session_id,
            language_preference=pref.language_preference,
            personalization_level=pref.personalization_level,
            background=pref.background
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting preferences: {str(e)}")

if __name__ == "__main__":
    import os
    port = int(os.getenv("PORT", "8000"))
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=port,
        timeout_keep_alive=75
    )
