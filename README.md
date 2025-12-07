# Physical AI & Humanoid Robotics Textbook

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comprehensive, interactive textbook on Physical AI and Humanoid Robotics with an integrated RAG chatbot for Q&A.

## 🌟 Features

- ✅ **Comprehensive Content**: 4 modules covering ROS 2, Simulation, NVIDIA Isaac, and VLA
- ✅ **RAG Chatbot**: AI assistant that answers questions based on textbook content
- ✅ **Selected Text Support**: Ask questions about specific passages
- ✅ **Beautiful UI**: Built with Docusaurus for excellent reading experience
- ✅ **Code Examples**: Practical Python code throughout
- ✅ **Capstone Project**: Build an autonomous humanoid robot

## 📚 Course Modules

### Module 1: The Robotic Nervous System (ROS 2)
- ROS 2 architecture and fundamentals
- Nodes, topics, services, actions
- Custom messages and URDF
- TF2 coordinate transforms

### Module 2: The Digital Twin (Gazebo & Unity)
- Physics simulation with Gazebo
- Sensor modeling (LiDAR, cameras, IMU)
- Unity integration for visualization
- Sim-to-real transfer techniques

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- Isaac Sim for photorealistic simulation
- Isaac ROS for hardware-accelerated perception
- VSLAM and Nav2 navigation
- Synthetic data generation

### Module 4: Vision-Language-Action (VLA)
- Voice-to-action with OpenAI Whisper
- LLM-based cognitive planning
- Multimodal robot control
- **Capstone Project**: Autonomous humanoid

## 🚀 Quick Start

### Prerequisites

- **Node.js**: 18+ ([Download](https://nodejs.org/))
- **Python**: 3.11+ ([Download](https://python.org/))
- **Git**: ([Download](https://git-scm.com/))

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Setup Backend (RAG Chatbot)

```bash
cd backend

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env and add your API keys:
# - OPENAI_API_KEY
# - COHERE_API_KEY
# - QDRANT_URL
# - QDRANT_API_KEY

# Run the server
uvicorn app.main:app --reload
```

The backend will run at `http://localhost:8000`

### 3. Setup Frontend (Docusaurus)

```bash
cd ../website

# Install dependencies
npm install

# Start development server
npm start
```

The website will open at `http://localhost:3000`

### 4. Index Your Content

After deploying the site, populate the vector database:

```bash
cd ../backend
python populate_db.py
```

Enter your deployed site's sitemap URL (e.g., `https://yourusername.github.io/physical-ai-textbook/sitemap.xml`)

## 🔧 Configuration

### Backend API Keys

You need free API keys from:

1. **OpenAI** ([Get Key](https://platform.openai.com/api-keys))
   - Used for GPT-4 based answers
   - Pay-as-you-go pricing

2. **Cohere** ([Get Key](https://dashboard.cohere.com/api-keys))
   - Used for embeddings (embed-english-v3.0)
   - 1000 requests/month free

3. **Qdrant Cloud** ([Get Cluster](https://cloud.qdrant.io/))
   - Vector database for RAG
   - 1GB free tier

### Frontend Configuration

Edit `website/docusaurus.config.ts`:

```typescript
url: 'https://yourusername.github.io',
baseUrl: '/physical-ai-textbook/',
organizationName: 'yourusername',
projectName: 'physical-ai-textbook',
```

Edit `website/src/components/RAGChatbot.tsx`:

```typescript
const BACKEND_URL = 'https://your-backend-url.com'; // Or use environment variable
```

## 📦 Deployment

### Deploy Frontend to GitHub Pages

```bash
cd website

# Build
npm run build

# Deploy (configure in package.json or use GitHub Actions)
GIT_USER=yourusername npm run deploy
```

### Deploy Backend to Render.com (Free)

1. Create `render.yaml` in backend/:

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

2. Push to GitHub
3. Connect Render.com to your repo
4. Add environment variables
5. Deploy!

## 🧪 Testing

### Test Backend API

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
  -d '{"query": "ROS 2 nodes", "limit": 3}'
```

### Test Frontend

1. Navigate to http://localhost:3000
2. Click the chatbot button (💬)
3. Ask: "What is Physical AI?"
4. Select text on the page and ask about it

## 📂 Project Structure

```
physical-ai-textbook/
├── backend/
│   ├── app/
│   │   ├── main.py          # FastAPI application
│   │   ├── agent.py         # RAG agent with OpenAI
│   │   ├── vector_db.py     # Qdrant operations
│   │   ├── data_extractor.py # Content extraction
│   │   └── config.py        # Configuration
│   ├── populate_db.py       # Indexing script
│   ├── requirements.txt
│   ├── .env.example
│   └── README.md
├── website/
│   ├── docs/
│   │   ├── intro.md
│   │   ├── module-1/        # ROS 2 content
│   │   ├── module-2/        # Simulation content
│   │   ├── module-3/        # Isaac content
│   │   └── module-4/        # VLA + Capstone
│   ├── src/
│   │   ├── components/
│   │   │   └── RAGChatbot.tsx  # Chatbot component
│   │   ├── css/
│   │   │   └── custom.css
│   │   └── pages/
│   │       └── index.tsx    # Homepage
│   ├── docusaurus.config.ts
│   ├── sidebars.ts
│   ├── package.json
│   └── tsconfig.json
└── README.md
```

## 🎯 Hackathon Submission

This project was built for the Panaversity Hackathon. It fulfills all requirements:

### Base Requirements (100 points)
- ✅ Docusaurus-based textbook
- ✅ Deployed to GitHub Pages
- ✅ RAG chatbot with FastAPI backend
- ✅ OpenAI Agents SDK integration
- ✅ Qdrant Cloud vector database
- ✅ Selected text support

### Bonus Features (Optional)
- 🔄 Spec-Kit Plus integration (50 bonus points)
- 🔄 Better-Auth signup/signin (50 bonus points)
- 🔄 Content personalization (50 bonus points)
- 🔄 Urdu translation (50 bonus points)

## 🎬 Demo Video

[Link to 90-second demo video](#)

Key highlights:
1. Homepage and navigation
2. Reading textbook content
3. Asking chatbot questions
4. Selected text feature
5. Backend API demonstration

## 🛠️ Development

### Add New Content

1. Create markdown files in `website/docs/`
2. Update `website/sidebars.ts`
3. Rebuild and redeploy
4. Run `populate_db.py` to index new content

### Customize Chatbot

Edit `website/src/components/RAGChatbot.tsx` to:
- Change UI styling
- Add conversation history
- Implement voice input
- Add file upload support

### Extend Backend

Add new endpoints in `backend/app/main.py`:
- Document upload
- Custom embeddings
- Multi-language support
- User analytics

## 📖 Resources

- [Docusaurus Documentation](https://docusaurus.io/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [OpenAI API Reference](https://platform.openai.com/docs/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [NVIDIA Isaac Documentation](https://docs.omniverse.nvidia.com/isaacsim/)

## 👥 Contributors

- [Your Name](https://github.com/yourusername)

## 📄 License

MIT License - see [LICENSE](LICENSE) file for details

## 🙏 Acknowledgments

- **Panaversity** for organizing the hackathon
- **Zia Khan**, **Rehan Aziz**, **Junaid Ahmed**, **Wania Iqbal** - Panaversity founders
- **GitHub** for Spec-Kit inspiration
- **NVIDIA** for Isaac platform documentation
- **Open Source Community** for amazing tools

## 📞 Support

For questions or issues:
- Open a [GitHub Issue](https://github.com/yourusername/physical-ai-textbook/issues)
- Email: your.email@example.com
- Twitter: [@yourhandle](https://twitter.com/yourhandle)

---

**Built with ❤️ for the Panaversity Hackathon**

*Submission Date: November 30, 2025*
