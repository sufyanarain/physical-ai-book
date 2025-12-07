# Setup script for Docusaurus - Run this manually

Write-Host "🚀 Setting up Docusaurus for Physical AI Textbook..." -ForegroundColor Green

# Navigate to project root
Set-Location "C:\Users\Diya Interactive\Documents\hack-vs\physical-ai-book"

# Create Docusaurus site
npx create-docusaurus@latest website classic --typescript

Write-Host "✅ Docusaurus initialized!" -ForegroundColor Green
Write-Host "📝 Next steps:" -ForegroundColor Yellow
Write-Host "   1. cd website" -ForegroundColor Cyan
Write-Host "   2. npm start" -ForegroundColor Cyan
