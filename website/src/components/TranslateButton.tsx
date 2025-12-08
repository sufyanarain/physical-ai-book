import React, { useState, useEffect } from 'react';
import '@site/src/css/custom.css';

// Use local backend in development, Railway in production
const BACKEND_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://physical-ai-backend-production-b62f.up.railway.app';

interface TranslateButtonProps {
  content: string;
  onTranslate?: (translatedContent: string) => void;
}

export default function TranslateButton({ content, onTranslate }: TranslateButtonProps): React.ReactElement {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [error, setError] = useState('');

  // Check if we have cached translation in sessionStorage
  useEffect(() => {
    // Create a simple hash from content for cache key (avoiding btoa with Unicode)
    const createHash = (str: string) => {
      let hash = 0;
      for (let i = 0; i < str.length; i++) {
        const char = str.charCodeAt(i);
        hash = ((hash << 5) - hash) + char;
        hash = hash & hash; // Convert to 32bit integer
      }
      return hash.toString(36);
    };
    
    const cacheKey = `translation_${createHash(content.substring(0, 100))}`;
    const cached = sessionStorage.getItem(cacheKey);
    if (cached) {
      try {
        const { translated, timestamp } = JSON.parse(cached);
        // Cache valid for 1 hour
        if (Date.now() - timestamp < 3600000) {
          setTranslatedContent(translated);
        }
      } catch (e) {
        console.error('Cache parse error:', e);
      }
    }
  }, [content]);

  const handleTranslate = async () => {
    if (isTranslated) {
      // Toggle back to original
      setIsTranslated(false);
      if (onTranslate) {
        onTranslate(''); // Clear translation
      }
      return;
    }

    // Use cached translation if available
    if (translatedContent) {
      setIsTranslated(true);
      if (onTranslate) {
        onTranslate(translatedContent);
      }
      return;
    }

    setIsTranslating(true);
    setError('');

    try {
      const response = await fetch(`${BACKEND_URL}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: content,
          target_language: 'urdu'
        })
      });

      if (!response.ok) {
        throw new Error('Translation failed');
      }

      const data = await response.json();
      const translated = data.translated;
      setTranslatedContent(translated);
      setIsTranslated(true);

      // Cache the translation using hash instead of btoa
      const createHash = (str: string) => {
        let hash = 0;
        for (let i = 0; i < str.length; i++) {
          const char = str.charCodeAt(i);
          hash = ((hash << 5) - hash) + char;
          hash = hash & hash;
        }
        return hash.toString(36);
      };
      
      const cacheKey = `translation_${createHash(content.substring(0, 100))}`;
      sessionStorage.setItem(cacheKey, JSON.stringify({
        translated,
        timestamp: Date.now()
      }));

      if (onTranslate) {
        onTranslate(translated);
      }
    } catch (err) {
      setError('Failed to translate content. Please try again.');
      console.error('Translation error:', err);
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <div style={{ marginBottom: '1rem' }}>
      <button
        onClick={handleTranslate}
        disabled={isTranslating}
        style={{
          padding: '8px 16px',
          backgroundColor: isTranslated ? '#28a745' : '#007bff',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: isTranslating ? 'not-allowed' : 'pointer',
          fontSize: '14px',
          fontWeight: '500',
          display: 'inline-flex',
          alignItems: 'center',
          gap: '8px',
          transition: 'background-color 0.2s'
        }}
        onMouseEnter={(e) => {
          if (!isTranslating) {
            e.currentTarget.style.backgroundColor = isTranslated ? '#218838' : '#0056b3';
          }
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.backgroundColor = isTranslated ? '#28a745' : '#007bff';
        }}
      >
        {isTranslating ? (
          <>
            <span className="spinner" style={{
              width: '14px',
              height: '14px',
              border: '2px solid rgba(255, 255, 255, 0.3)',
              borderTopColor: 'white',
              borderRadius: '50%',
              animation: 'spin 0.6s linear infinite'
            }}></span>
            Translating...
          </>
        ) : isTranslated ? (
          <>
            ✓ Show Original
          </>
        ) : (
          <>
            🌐 Translate to Urdu
          </>
        )}
      </button>

      {error && (
        <div style={{
          marginTop: '8px',
          padding: '8px 12px',
          backgroundColor: '#f8d7da',
          color: '#721c24',
          borderRadius: '4px',
          fontSize: '14px'
        }}>
          {error}
        </div>
      )}

      <style>{`
        @keyframes spin {
          to { transform: rotate(360deg); }
        }
      `}</style>
    </div>
  );
}
