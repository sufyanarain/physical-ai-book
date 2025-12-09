"""
Neon Postgres database helper for user data and preferences
"""
from sqlalchemy import create_engine, Column, String, Integer, DateTime, Text, Boolean
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
from app.config import settings
import logging

logger = logging.getLogger(__name__)

Base = declarative_base()

class User(Base):
    """User authentication and profile"""
    __tablename__ = 'users'
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    email = Column(String(255), unique=True, nullable=False, index=True)
    name = Column(String(255), nullable=False)
    password_hash = Column(String(255), nullable=False)
    is_active = Column(Boolean, default=True)
    
    # Background information
    background_type = Column(String(50), default='software')  # 'software' or 'hardware'
    learning_goals = Column(Text, nullable=True)
    
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class UserPreference(Base):
    """Store user preferences for personalization and translation"""
    __tablename__ = 'user_preferences'
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    user_id = Column(Integer, nullable=True)  # Links to User table (optional for anonymous)
    session_id = Column(String(255), unique=True, nullable=False, index=True)
    language_preference = Column(String(50), default='english')
    personalization_level = Column(String(50), default='intermediate')
    background = Column(Text, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class ChatHistory(Base):
    """Store chat history for analytics (optional)"""
    __tablename__ = 'chat_history'
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    session_id = Column(String(255), nullable=False, index=True)
    question = Column(Text, nullable=False)
    answer = Column(Text, nullable=False)
    context_used = Column(Text, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)

class Database:
    def __init__(self):
        self.engine = None
        self.SessionLocal = None
        self.initialized = False
        
    def initialize(self):
        """Initialize database connection"""
        if self.initialized:
            return
        
        try:
            # Use Neon database URL if available, otherwise fall back to DATABASE_URL
            db_url = settings.neon_database_url or settings.database_url
            
            if not db_url:
                logger.warning("No database URL configured. Database features disabled.")
                return
            
            # Create engine with connection pooling
            self.engine = create_engine(
                db_url,
                pool_size=5,
                max_overflow=10,
                pool_pre_ping=True,  # Verify connections before using
                pool_recycle=3600    # Recycle connections after 1 hour
            )
            
            # Create session factory
            self.SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=self.engine)
            
            # Create tables if they don't exist
            Base.metadata.create_all(bind=self.engine)
            
            self.initialized = True
            logger.info("Database initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize database: {e}")
            self.initialized = False
    
    def get_session(self):
        """Get a database session"""
        if not self.initialized:
            self.initialize()
        
        if not self.SessionLocal:
            return None
        
        return self.SessionLocal()
    
    def save_user_preference(self, session_id: str, language: str = None, 
                            personalization: str = None, background: str = None):
        """Save or update user preferences"""
        session = self.get_session()
        if not session:
            return None
        
        try:
            # Check if preference exists
            pref = session.query(UserPreference).filter_by(session_id=session_id).first()
            
            if pref:
                # Update existing
                if language:
                    pref.language_preference = language
                if personalization:
                    pref.personalization_level = personalization
                if background:
                    pref.background = background
                pref.updated_at = datetime.utcnow()
            else:
                # Create new
                pref = UserPreference(
                    session_id=session_id,
                    language_preference=language or 'english',
                    personalization_level=personalization or 'intermediate',
                    background=background
                )
                session.add(pref)
            
            session.commit()
            session.refresh(pref)
            return pref
        
        except Exception as e:
            logger.error(f"Error saving user preference: {e}")
            session.rollback()
            return None
        finally:
            session.close()
    
    def get_user_preference(self, session_id: str):
        """Get user preferences"""
        session = self.get_session()
        if not session:
            return None
        
        try:
            pref = session.query(UserPreference).filter_by(session_id=session_id).first()
            return pref
        except Exception as e:
            logger.error(f"Error getting user preference: {e}")
            return None
        finally:
            session.close()
    
    def save_chat_history(self, session_id: str, question: str, answer: str, context: str = None):
        """Save chat history (optional)"""
        session = self.get_session()
        if not session:
            return None
        
        try:
            chat = ChatHistory(
                session_id=session_id,
                question=question,
                answer=answer,
                context_used=context
            )
            session.add(chat)
            session.commit()
            return chat
        except Exception as e:
            logger.error(f"Error saving chat history: {e}")
            session.rollback()
            return None
        finally:
            session.close()

    def create_user(self, email: str, name: str, password_hash: str, 
                   background_type: str = 'software',
                   learning_goals: str = None):
        """Create a new user"""
        session = self.get_session()
        if not session:
            return None
        
        try:
            # Check if user exists
            existing = session.query(User).filter_by(email=email).first()
            if existing:
                return None  # User already exists
            
            user = User(
                email=email,
                name=name,
                password_hash=password_hash,
                background_type=background_type,
                learning_goals=learning_goals
            )
            session.add(user)
            session.commit()
            session.refresh(user)
            return user
        except Exception as e:
            logger.error(f"Error creating user: {e}")
            session.rollback()
            return None
        finally:
            session.close()
    
    def get_user_by_email(self, email: str):
        """Get user by email"""
        session = self.get_session()
        if not session:
            return None
        
        try:
            user = session.query(User).filter_by(email=email).first()
            return user
        except Exception as e:
            logger.error(f"Error getting user: {e}")
            return None
        finally:
            session.close()
    
    def get_user_by_id(self, user_id: int):
        """Get user by ID"""
        session = self.get_session()
        if not session:
            return None
        
        try:
            user = session.query(User).filter_by(id=user_id).first()
            return user
        except Exception as e:
            logger.error(f"Error getting user: {e}")
            return None
        finally:
            session.close()
    
    def update_user(self, user_id: int, **kwargs):
        """Update user information"""
        session = self.get_session()
        if not session:
            return None
        
        try:
            user = session.query(User).filter_by(id=user_id).first()
            if not user:
                return None
            
            for key, value in kwargs.items():
                if hasattr(user, key) and value is not None:
                    setattr(user, key, value)
            
            user.updated_at = datetime.utcnow()
            session.commit()
            session.refresh(user)
            return user
        except Exception as e:
            logger.error(f"Error updating user: {e}")
            session.rollback()
            return None
        finally:
            session.close()

# Global database instance
db = Database()
