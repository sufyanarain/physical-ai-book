"""
AI Agent module using Groq API
"""
from groq import Groq
from typing import List, Dict, Any
from app.config import settings
from app.vector_db import VectorDB

class RAGAgent:
    """RAG Agent using Groq for question answering"""
    
    def __init__(self):
        self.client = Groq(api_key=settings.groq_api_key)
        self.model = "llama-3.3-70b-versatile"
        self.vector_db = VectorDB()
    
    def retrieve_context(self, query: str, limit: int = 5) -> str:
        """Retrieve relevant context from vector database"""
        results = self.vector_db.search(query, limit=limit)
        
        if not results:
            return "No relevant context found."
        
        context_parts = []
        for i, result in enumerate(results, 1):
            context_parts.append(
                f"[Source {i} - {result['title']}]\n{result['content']}\n"
            )
        
        return "\n".join(context_parts)
    
    def answer_question(self, question: str, selected_text: str = None) -> Dict[str, Any]:
        """Answer a question using RAG"""
        
        # If user has selected specific text, use that as primary context
        if selected_text and len(selected_text.strip()) > 20:
            context = f"User Selected Text:\n{selected_text}\n\n"
            context += f"Additional Context:\n{self.retrieve_context(question, limit=3)}"
        else:
            context = self.retrieve_context(question, limit=5)
        
        system_prompt = """You are an expert AI assistant specialized in Physical AI and Humanoid Robotics.
Your role is to answer questions about the textbook content clearly and accurately.

Guidelines:
1. Base your answers primarily on the provided context
2. If the context doesn't contain enough information, say so clearly
3. Provide detailed technical explanations when appropriate
4. Use examples from the textbook when relevant
5. Be concise but comprehensive
6. If user selected specific text, prioritize that in your answer"""

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {question}"}
                ],
                temperature=0.7,
                max_tokens=1000
            )
            
            answer = response.choices[0].message.content
            
            return {
                "answer": answer,
                "context_used": context,
                "model": self.model
            }
        
        except Exception as e:
            return {
                "answer": f"Error generating response: {str(e)}",
                "context_used": context,
                "model": self.model,
                "error": True
            }
    
    def chat(self, messages: List[Dict[str, str]]) -> str:
        """Multi-turn conversation support"""
        try:
            # Get the latest user message for context retrieval
            latest_user_msg = next(
                (m["content"] for m in reversed(messages) if m["role"] == "user"),
                ""
            )
            
            context = self.retrieve_context(latest_user_msg, limit=3)
            
            system_msg = {
                "role": "system",
                "content": f"""You are an expert AI assistant for Physical AI and Humanoid Robotics.
                
Relevant Context:
{context}

Use this context to inform your responses."""
            }
            
            full_messages = [system_msg] + messages
            
            response = self.client.chat.completions.create(
                model=self.model,
                messages=full_messages,
                temperature=0.7,
                max_tokens=1000
            )
            
            return response.choices[0].message.content
        
        except Exception as e:
            return f"Error in chat: {str(e)}"
