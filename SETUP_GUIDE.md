# 🚀 Complete Setup Guide

Step-by-step instructions to get the Physical AI Textbook running.

## Prerequisites Check

Open PowerShell and verify:

```powershell
# Check Node.js (need 18+)
node --version

# Check Python (need 3.11+)
python --version

# Check Git
git --version
```

If missing, install from:
- Node.js: https://nodejs.org/
- Python: https://python.org/
- Git: https://git-scm.com/

## Step 1: Get Free API Keys

### 1.1 OpenAI API Key

1. Go to https://platform.openai.com/
2. Sign up / Log in
3. Click your profile → "View API keys"
4. Click "Create new secret key"
5. Copy the key (starts with `sk-`)

### 1.2 Cohere API Key (Free)

1. Go to https://dashboard.cohere.com/
2. Sign up with email
3. Click "API Keys" in sidebar
4. Copy the "Trial Key"

### 1.3 Qdrant Cloud (Free)

1. Go to https://cloud.qdrant.io/
2. Sign up (GitHub recommended)
3. Click "Create Cluster"
4. Choose "Free" tier (1GB)
5. Name it (e.g., "physical-ai")
6. Click "Create"
7. Once created, click the cluster
8. Copy the **API Key** and **Cluster URL**

## Step 2: Clone & Setup Project

```powershell
# Navigate to your projects folder
cd "C:\Users\YourName\Documents"

# Clone this repository
git clone https://github.com/yourusername/physical-ai-textbook.git
cd physical-ai-textbook
```

## Step 3: Setup Backend

```powershell
# Navigate to backend
cd backend

# Create virtual environment (recommended)
python -m venv venv
.\venv\Scripts\Activate

# Install dependencies
pip install -r requirements.txt

# Create .env file
copy .env.example .env
```

Edit `.env` file with your API keys:

```env
OPENAI_API_KEY=sk-your-actual-key-here
COHERE_API_KEY=your-cohere-key-here
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=physical_ai_textbook
```

**Test the backend:**

```powershell
# Start server
uvicorn app.main:app --reload

# In another terminal, test:
curl http://localhost:8000/health
```

You should see: `{"status":"healthy"}`

## Step 4: Setup Frontend

```powershell
# Open NEW terminal
cd website

# Install dependencies
npm install

# Start dev server
npm start
```

Browser will open at http://localhost:3000

## Step 5: Deploy Frontend to GitHub Pages

### 5.1 Create GitHub Repository

1. Go to https://github.com/new
2. Name: `physical-ai-textbook`
3. Make it **Public**
4. Click "Create repository"

### 5.2 Configure Docusaurus

Edit `website/docusaurus.config.ts`:

```typescript
url: 'https://YOUR-GITHUB-USERNAME.github.io',
baseUrl: '/physical-ai-textbook/',
organizationName: 'YOUR-GITHUB-USERNAME',
projectName: 'physical-ai-textbook',
```

### 5.3 Push to GitHub

```powershell
# In project root
git remote add origin https://github.com/YOUR-USERNAME/physical-ai-textbook.git
git add .
git commit -m "Initial commit"
git branch -M main
git push -u origin main
```

### 5.4 Enable GitHub Pages

1. Go to your repo → **Settings**
2. Click **Pages** (left sidebar)
3. Source: Deploy from a branch
4. Branch: `gh-pages` → `/ (root)`
5. Click **Save**

After a few minutes, your site will be live at:
`https://YOUR-USERNAME.github.io/physical-ai-textbook/`

## Step 6: Index Content to Vector Database

After deployment:

```powershell
cd backend
python populate_db.py
```

When prompted, enter:
```
https://YOUR-USERNAME.github.io/physical-ai-textbook/sitemap.xml
```

Wait for indexing to complete (1-2 minutes).

## Step 7: Deploy Backend (Render.com - Free)

### 7.1 Prepare for Deployment

Create `backend/render.yaml`:

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

Push changes:
```powershell
git add .
git commit -m "Add Render deployment config"
git push
```

### 7.2 Deploy on Render

1. Go to https://render.com/
2. Sign up with GitHub
3. Click **"New +" → "Web Service"**
4. Connect your repository
5. Name: `physical-ai-api`
6. Root Directory: `backend`
7. Build Command: `pip install -r requirements.txt`
8. Start Command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
9. Click **"Create Web Service"**

### 7.3 Add Environment Variables

In Render dashboard:
1. Click your service
2. Go to **"Environment"** tab
3. Add:
   - `OPENAI_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
4. Click **"Save Changes"**

Your API will deploy! URL: `https://physical-ai-api-xxx.onrender.com`

### 7.4 Update Frontend to Use Deployed API

Edit `website/src/components/RAGChatbot.tsx`:

```typescript
const BACKEND_URL = 'https://physical-ai-api-xxx.onrender.com';
```

Rebuild and redeploy:
```powershell
cd website
npm run build
git add .
git commit -m "Update backend URL"
git push
```

## Step 8: Final Testing

1. Visit your deployed site
2. Click the chatbot button (💬)
3. Ask: "What is Physical AI?"
4. Select text on the page
5. Ask about the selected text

## 🎥 Create Demo Video

### Using Screen Recording

**Windows:**
1. Press `Win + G` to open Game Bar
2. Click Record button
3. Navigate and demo features
4. Stop recording (90 seconds max!)

**Upload:**
- YouTube (unlisted)
- Google Drive (public link)
- Loom (free)

### Using NotebookLM (Recommended)

1. Go to https://notebooklm.google.com/
2. Upload your README.md
3. Click "Generate Audio Overview"
4. Download the audio
5. Create slides with key screenshots
6. Combine with Camtasia/OBS

## 📝 Hackathon Submission

Submit at: https://forms.gle/CQsSEGM3GeCrL43c8

Required:
1. ✅ GitHub Repo URL
2. ✅ Deployed Site URL
3. ✅ Demo Video URL (<90 seconds)
4. ✅ WhatsApp Number

## 🐛 Troubleshooting

### Backend won't start
```powershell
# Check if port 8000 is in use
netstat -ano | findstr :8000

# Kill the process
taskkill /PID <PID> /F
```

### Frontend build fails
```powershell
# Clear cache
rm -rf node_modules
rm package-lock.json
npm install
```

### Chatbot not responding
1. Check backend is running
2. Check CORS settings in `main.py`
3. Open browser console (F12) for errors
4. Verify API keys in `.env`

### Qdrant errors
1. Verify cluster is running (check Qdrant Cloud dashboard)
2. Ensure API key is correct
3. Check if collection exists

## 🎉 Success Checklist

- [ ] Backend running locally
- [ ] Frontend running locally
- [ ] Chatbot answering questions
- [ ] Selected text feature working
- [ ] Site deployed to GitHub Pages
- [ ] Backend deployed to Render
- [ ] Content indexed in Qdrant
- [ ] Demo video created
- [ ] Submitted to hackathon form

## 💡 Tips for Demo

1. **Start with homepage**: Show clean design
2. **Navigate to content**: Open a module
3. **Use chatbot**: Ask simple question
4. **Select text**: Highlight passage, ask about it
5. **Show response**: Emphasize RAG retrieval
6. **End with CTA**: "Ready to learn robotics!"

Time: 15 seconds per step = 90 seconds total

## 🚀 Next Steps After Hackathon

1. Add authentication with Better-Auth
2. Implement content personalization
3. Add Urdu translation
4. Create Claude subagents
5. Add more interactive examples
6. Build community forum

## 📞 Need Help?

- GitHub Issues: [Create Issue](https://github.com/yourusername/physical-ai-textbook/issues)
- Discord: [Panaversity Community](#)
- Email: support@example.com

---

**Good luck with the hackathon! 🎯**
