"""
Data extraction and processing module
Extracts content from URLs and chunks it for embedding
"""
import requests
from bs4 import BeautifulSoup
import trafilatura
from typing import List, Dict
import re
from app.config import settings

class DataExtractor:
    """Extract and process text content from URLs"""
    
    @staticmethod
    def get_urls_from_sitemap(sitemap_url: str) -> List[str]:
        """Extract URLs from sitemap.xml"""
        try:
            response = requests.get(sitemap_url, timeout=10)
            response.raise_for_status()
            soup = BeautifulSoup(response.content, 'xml')
            urls = [loc.text for loc in soup.find_all('loc')]
            return urls
        except Exception as e:
            print(f"Error fetching sitemap: {e}")
            return []
    
    @staticmethod
    def extract_text_from_url(url: str) -> Dict[str, str]:
        """Extract clean text content from a URL"""
        try:
            downloaded = trafilatura.fetch_url(url)
            if downloaded:
                text = trafilatura.extract(
                    downloaded,
                    include_comments=False,
                    include_tables=True,
                    no_fallback=False
                )
                if text:
                    return {
                        "url": url,
                        "content": text,
                        "title": DataExtractor._extract_title(downloaded)
                    }
        except Exception as e:
            print(f"Error extracting from {url}: {e}")
        return {"url": url, "content": "", "title": ""}
    
    @staticmethod
    def _extract_title(html_content: str) -> str:
        """Extract title from HTML"""
        try:
            soup = BeautifulSoup(html_content, 'html.parser')
            title_tag = soup.find('title')
            if title_tag:
                return title_tag.get_text(strip=True)
        except:
            pass
        return "Untitled"
    
    @staticmethod
    def chunk_text(text: str, chunk_size: int = None, overlap: int = None) -> List[str]:
        """Split text into overlapping chunks"""
        if not chunk_size:
            chunk_size = settings.chunk_size
        if not overlap:
            overlap = settings.chunk_overlap
            
        chunks = []
        start = 0
        text_length = len(text)
        
        while start < text_length:
            end = start + chunk_size
            chunk = text[start:end]
            
            # Try to end at a sentence boundary
            if end < text_length:
                last_period = chunk.rfind('.')
                last_newline = chunk.rfind('\n')
                split_point = max(last_period, last_newline)
                if split_point > chunk_size * 0.5:
                    chunk = text[start:start + split_point + 1]
                    end = start + split_point + 1
            
            chunks.append(chunk.strip())
            start = end - overlap
            
        return [c for c in chunks if len(c) > 50]  # Filter out very small chunks
    
    @staticmethod
    def process_documents(urls: List[str]) -> List[Dict[str, str]]:
        """Process multiple URLs and extract chunked content"""
        all_chunks = []
        
        for url in urls:
            print(f"Processing: {url}")
            doc = DataExtractor.extract_text_from_url(url)
            
            if doc["content"]:
                chunks = DataExtractor.chunk_text(doc["content"])
                for i, chunk in enumerate(chunks):
                    all_chunks.append({
                        "url": url,
                        "title": doc["title"],
                        "content": chunk,
                        "chunk_id": f"{url}#chunk-{i}"
                    })
        
        return all_chunks
