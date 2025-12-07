# Physical AI Textbook RAG Chatbot Backend

Backend API for the Physical AI & Humanoid Robotics textbook with RAG (Retrieval Augmented Generation) capabilities.

## Features

- ✅ **RAG-based Q&A**: Answer questions using retrieved context from the textbook
- ✅ **Selected Text Support**: Answer questions based on user-selected text
- ✅ **Multi-turn Conversations**: Support for chat-based interactions
- ✅ **Vector Search**: Semantic search across textbook content
- ✅ **Document Indexing**: Automatic indexing from sitemap.xml

## Tech Stack

- **FastAPI**: Modern web framework
- **OpenAI API**: GPT-4 for answer generation
- **Cohere**: Embedding generation (embed-english-v3.0)
- **Qdrant Cloud**: Vector database (free tier)
- **Trafilatura**: Web content extraction
- **Neon Postgres**: Database (optional, for user data)

## Setup Instructions

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment

Copy `.env.example` to `.env` and fill in your API keys:

```bash
cp .env.example .env
```

Required keys:
- **OPENAI_API_KEY**: Get from https://platform.openai.com/api-keys
- **COHERE_API_KEY**: Get from https://dashboard.cohere.com/api-keys
- **QDRANT_URL**: Your Qdrant Cloud cluster URL
- **QDRANT_API_KEY**: Your Qdrant API key

### 3. Index Your Textbook Content

After deploying your Docusaurus site, run:

```bash
python populate_db.py
```

This will:
1. Fetch all URLs from your sitemap
2. Extract and chunk the content
3. Generate embeddings
4. Store in Qdrant

### 4. Run the Server

```bash
# Development mode
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# Production mode
uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
```

## API Endpoints

### Health Check
```http
GET /health
```

### Ask a Question
```http
POST /ask
Content-Type: application/json

{
  "question": "What is Physical AI?",
  "selected_text": "optional user-selected text for context"
}
```

### Chat (Multi-turn)
```http
POST /chat
Content-Type: application/json

{
  "messages": [
    {"role": "user", "content": "What is ROS 2?"},
    {"role": "assistant", "content": "ROS 2 is..."},
    {"role": "user", "content": "How does it differ from ROS 1?"}
  ]
}
```

### Search
```http
POST /search
Content-Type: application/json

{
  "query": "humanoid robot",
  "limit": 5
}
```

### Index Documents
```http
POST /index
Content-Type: application/json

{
  "sitemap_url": "https://yoursite.com/sitemap.xml"
}
```

## Free Tier Resources

### Qdrant Cloud (Vector DB)
1. Go to https://cloud.qdrant.io/
2. Create free account
3. Create a cluster (1GB free)
4. Get API key from cluster settings

### Cohere (Embeddings)
1. Go to https://dashboard.cohere.com/
2. Sign up for free
3. Get API key (1000 requests/month free)

### OpenAI (GPT)
1. Go to https://platform.openai.com/
2. Add payment method (pay-as-you-go)
3. Get API key

## Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py           # FastAPI app
│   ├── config.py         # Configuration
│   ├── agent.py          # RAG agent
│   ├── vector_db.py      # Qdrant operations
│   └── data_extractor.py # Content extraction
├── populate_db.py        # Indexing script
├── requirements.txt
├── .env.example
└── README.md
```

## Deployment

### Deploy on Render.com (Free)

1. Create `render.yaml`:
```yaml
services:
  - type: web
    name: physical-ai-api
    runtime: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn app.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: OPENAI_API_KEY
        sync: false
      - key: COHERE_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
```

2. Connect your GitHub repo
3. Add environment variables
4. Deploy!

## Testing

Test the API with curl:

```bash
# Health check
curl http://localhost:8000/health

# Ask a question
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'

# Search
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{"query": "ROS 2", "limit": 3}'
```

## Troubleshooting

### Qdrant Connection Issues
- Verify your Qdrant URL and API key
- Check if your cluster is running
- Ensure your IP is not blocked

### Embedding Errors
- Verify Cohere API key
- Check rate limits (1000/month on free tier)
- Try smaller batch sizes

### OpenAI Errors
- Verify API key and billing
- Check rate limits
- Use gpt-4o-mini for lower costs

## License

MIT
