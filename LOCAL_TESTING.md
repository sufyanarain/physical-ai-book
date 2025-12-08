# Local Testing Guide

This guide explains how to run both the backend and frontend locally to test the Urdu translation feature.

## Prerequisites

- Python 3.12+ installed
- Node.js 18+ installed
- Git Bash or PowerShell

## Step 1: Start Backend Server (Port 8000)

Open a new terminal and run:

```bash
cd physical-ai-book/backend

# Start the FastAPI server
python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

You should see:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete.
```

**Test the backend:**
- Open browser: http://localhost:8000
- Check API docs: http://localhost:8000/docs

## Step 2: Start Frontend Dev Server (Port 3000)

Open a **new terminal** (keep backend running) and run:

```bash
cd physical-ai-book/website

# Install dependencies if not done yet
npm install

# Start dev server
npm start
```

You should see:
```
[SUCCESS] Serving at http://localhost:3000
```

## Step 3: Test the Features

### Test RAG Chatbot
1. Go to http://localhost:3000/physical-ai-book/
2. Click the floating chat button (bottom right)
3. Ask: "What is VLA?"
4. Should get response about Vision-Language-Action

### Test Urdu Translation
1. Go to any doc page, e.g., http://localhost:3000/physical-ai-book/docs/module-4/vla-intro
2. You should see "🌐 Translate to Urdu" button at the top
3. Click it and wait for translation
4. Content should change to Urdu (right-to-left text)
5. Click "✓ Show Original" to toggle back

### Test Text Selection Feature
1. Select any text on the page
2. Chatbot opens automatically
3. Ask a question about the selected text
4. Should get context-aware response

## Troubleshooting

### Backend Issues

**Error: "ModuleNotFoundError"**
```bash
cd physical-ai-book/backend
pip install -r requirements.txt
```

**Error: "Port 8000 already in use"**
```bash
# Kill the process on port 8000
# Windows:
netstat -ano | findstr :8000
taskkill /PID <PID> /F

# Linux/Mac:
lsof -ti:8000 | xargs kill
```

**Error: "GROQ_API_KEY not found"**
Make sure you have `.env` file in `backend/` directory with:
```
GROQ_API_KEY=your_key_here
COHERE_API_KEY=your_key_here
QDRANT_URL=your_url_here
QDRANT_API_KEY=your_key_here
```

### Frontend Issues

**Error: "CORS policy"**
Backend is configured to allow localhost:3000. If you see CORS errors, check that backend is running on port 8000.

**Translation not working**
1. Check browser console (F12) for errors
2. Verify backend is running: http://localhost:8000/health
3. Test translation endpoint directly: http://localhost:8000/docs → Try /translate endpoint

**Chatbot not connecting**
1. Check backend logs for errors
2. Verify database is populated:
   ```bash
   cd physical-ai-book/backend
   python populate_db.py
   ```

## Quick Test Commands

### Backend Health Check
```bash
curl http://localhost:8000/health
```

### Test Translation API
```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{"content": "Hello, this is a test.", "target_language": "urdu"}'
```

### Test Chat API
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"messages": [{"role": "user", "content": "What is Physical AI?"}]}'
```

## Production Deployment

Once testing is complete:

1. **Push changes:**
   ```bash
   git add .
   git commit -m "Add Urdu translation feature"
   git push
   ```

2. **Railway will auto-deploy** the backend

3. **Deploy frontend to GitHub Pages:**
   ```bash
   cd physical-ai-book/website
   npm run build
   npm run deploy
   ```

## Environment Variables

### Development (Local)
Backend uses `.env` file in `backend/` directory.

### Production (Railway)
Environment variables are set in Railway dashboard:
- `GROQ_API_KEY`
- `COHERE_API_KEY`
- `QDRANT_URL`
- `QDRANT_API_KEY`

Frontend automatically switches to Railway backend in production build.
