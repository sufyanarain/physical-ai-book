"""
Vector Database module using Qdrant and sentence-transformers (free)
"""
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from sentence_transformers import SentenceTransformer
from typing import List, Dict, Any
import hashlib
from app.config import settings

class VectorDB:
    """Vector database operations using Qdrant and local embeddings"""
    
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = "physical_ai_textbook"
        # Use free local embedding model
        self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
        self.vector_size = 384  # all-MiniLM-L6-v2 produces 384-dimensional vectors
    
    def create_collection(self):
        """Create collection if it doesn't exist"""
        try:
            self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists")
        except:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE
                )
            )
            print(f"Collection '{self.collection_name}' created successfully")
    
    def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Get embeddings for texts using sentence-transformers"""
        embeddings = self.embedding_model.encode(texts, convert_to_numpy=True)
        return embeddings.tolist()
    
    def add_documents(self, documents: List[Dict[str, str]]):
        """Add documents to the vector database"""
        texts = [doc["content"] for doc in documents]
        embeddings = self.get_embeddings(texts)
        
        if not embeddings:
            print("Failed to generate embeddings")
            return
        
        points = []
        for i, (doc, embedding) in enumerate(zip(documents, embeddings)):
            point_id = hashlib.md5(doc["content"].encode()).hexdigest()
            points.append(
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "title": doc.get("title", ""),
                        "content": doc["content"],
                        "source": doc.get("source", "")
                    }
                )
            )
        
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        print(f"Added {len(points)} documents to the collection")
    
    def search(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """Search for similar documents"""
        try:
            query_embedding = self.get_embeddings([query])[0]
            
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )
            
            return [
                {
                    "title": hit.payload.get("title", ""),
                    "content": hit.payload.get("content", ""),
                    "score": hit.score
                }
                for hit in results
            ]
        except Exception as e:
            print(f"Search error: {e}")
            return []
