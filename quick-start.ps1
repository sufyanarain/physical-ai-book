# Quick Start Script for Physical AI Textbook
# Run this after cloning the repository

Write-Host "🚀 Physical AI Textbook - Quick Start" -ForegroundColor Green
Write-Host "=====================================" -ForegroundColor Green
Write-Host ""

# Check prerequisites
Write-Host "📋 Checking prerequisites..." -ForegroundColor Yellow

$nodeVersion = node --version 2>$null
$pythonVersion = python --version 2>$null
$gitVersion = git --version 2>$null

if (-not $nodeVersion) {
    Write-Host "❌ Node.js not found. Install from https://nodejs.org/" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Node.js: $nodeVersion" -ForegroundColor Green

if (-not $pythonVersion) {
    Write-Host "❌ Python not found. Install from https://python.org/" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Python: $pythonVersion" -ForegroundColor Green

if (-not $gitVersion) {
    Write-Host "❌ Git not found. Install from https://git-scm.com/" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Git: $gitVersion" -ForegroundColor Green

Write-Host ""
Write-Host "🔧 Setting up backend..." -ForegroundColor Yellow
Set-Location backend

if (-not (Test-Path ".env")) {
    Copy-Item ".env.example" ".env"
    Write-Host "⚠️  Created .env file. Please edit it with your API keys!" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "You need:" -ForegroundColor Cyan
    Write-Host "  1. OpenAI API Key: https://platform.openai.com/api-keys" -ForegroundColor Cyan
    Write-Host "  2. Cohere API Key: https://dashboard.cohere.com/api-keys" -ForegroundColor Cyan
    Write-Host "  3. Qdrant URL + Key: https://cloud.qdrant.io/" -ForegroundColor Cyan
    Write-Host ""
    
    $continue = Read-Host "Have you added your API keys to .env? (y/n)"
    if ($continue -ne "y") {
        Write-Host "❌ Please add API keys to backend\.env and run this script again." -ForegroundColor Red
        exit 1
    }
}

Write-Host "📦 Installing Python dependencies..." -ForegroundColor Yellow
pip install -q -r requirements.txt

if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Failed to install backend dependencies" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Backend dependencies installed" -ForegroundColor Green

Write-Host ""
Write-Host "🌐 Setting up frontend..." -ForegroundColor Yellow
Set-Location ../website

Write-Host "📦 Installing Node dependencies (this may take a few minutes)..." -ForegroundColor Yellow
npm install

if ($LASTEXITCODE -ne 0) {
    Write-Host "❌ Failed to install frontend dependencies" -ForegroundColor Red
    exit 1
}
Write-Host "✅ Frontend dependencies installed" -ForegroundColor Green

Write-Host ""
Write-Host "✅ Setup complete!" -ForegroundColor Green
Write-Host ""
Write-Host "📚 Next steps:" -ForegroundColor Yellow
Write-Host ""
Write-Host "1. Start the backend:" -ForegroundColor Cyan
Write-Host "   cd backend" -ForegroundColor White
Write-Host "   uvicorn app.main:app --reload" -ForegroundColor White
Write-Host ""
Write-Host "2. In a NEW terminal, start the frontend:" -ForegroundColor Cyan
Write-Host "   cd website" -ForegroundColor White
Write-Host "   npm start" -ForegroundColor White
Write-Host ""
Write-Host "3. Open your browser to http://localhost:3000" -ForegroundColor Cyan
Write-Host ""
Write-Host "4. Click the chatbot button (💬) and ask a question!" -ForegroundColor Cyan
Write-Host ""
Write-Host "📖 For full deployment instructions, see SETUP_GUIDE.md" -ForegroundColor Yellow
Write-Host ""
Write-Host "🎉 Happy learning!" -ForegroundColor Green
