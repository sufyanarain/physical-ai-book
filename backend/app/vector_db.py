"""
Vector Database module using Qdrant and Cohere embeddings (free API)
"""
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import cohere
from typing import List, Dict, Any
import hashlib
from app.config import settings

class VectorDB:
    """Vector database operations using Qdrant and Cohere embeddings"""
    
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = "physical_ai_textbook"
        # Use Cohere's free embedding API
        self.cohere_client = cohere.Client(settings.cohere_api_key)
        self.vector_size = 1024  # Cohere embed-english-light-v3.0 produces 1024-dimensional vectors
    
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
        """Get embeddings for texts using Cohere API"""
        response = self.cohere_client.embed(
            texts=texts,
            model="embed-english-light-v3.0",
            input_type="search_document"
        )
        return response.embeddings
    
    def add_documents(self, documents: List[Dict[str, str]], batch_size: int = 50):
        """Add documents to the vector database in batches"""
        total_docs = len(documents)

        for batch_start in range(0, total_docs, batch_size):
            batch_end = min(batch_start + batch_size, total_docs)
            batch_docs = documents[batch_start:batch_end]

            texts = [doc["content"] for doc in batch_docs]
            embeddings = self.get_embeddings(texts)

            if not embeddings:
                print(f"Failed to generate embeddings for batch {batch_start}-{batch_end}")
                continue

            points = []
            for i, (doc, embedding) in enumerate(zip(batch_docs, embeddings)):
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
            print(f"Added batch {batch_start+1}-{batch_end} ({len(points)} documents)")
    
    def search(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """Search for similar documents"""
        try:
            # Use search_query input type for queries
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-light-v3.0",
                input_type="search_query"
            )
            query_embedding = response.embeddings[0]
            
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
