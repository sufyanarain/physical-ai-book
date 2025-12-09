import React, { useState } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import '@site/src/css/custom.css';

interface TranslateButtonProps {
  content?: string;
  onTranslate?: (translatedContent: string) => void;
}

export default function TranslateButton({ content, onTranslate }: TranslateButtonProps): React.ReactElement {
  const history = useHistory();
  const location = useLocation();
  const [isUrdu, setIsUrdu] = useState(false);

  // Check if currently on Urdu docs
  React.useEffect(() => {
    setIsUrdu(location.pathname.includes('/docs-urdu/'));
  }, [location.pathname]);

  const handleTranslate = () => {
    const currentPath = location.pathname;

    if (isUrdu) {
      // Currently on Urdu → Switch back to original (English or personalized)
      // Check if user is logged in to determine which version to show
      const userStr = localStorage.getItem('user');
      let targetPath = currentPath.replace('/docs-urdu/', '/docs/');

      if (userStr) {
        try {
          const user = JSON.parse(userStr);
          const backgroundType = user.background_type;

          if (backgroundType) {
            // User is logged in → redirect to their personalized docs
            targetPath = currentPath.replace('/docs-urdu/', `/docs-${backgroundType}/`);
          }
        } catch (e) {
          console.error('Error parsing user data:', e);
        }
      }

      history.push(targetPath + location.search + location.hash);
    } else {
      // Currently on English/personalized → Switch to Urdu
      let targetPath = currentPath;

      // Handle different doc paths
      if (currentPath.includes('/docs-software/')) {
        targetPath = currentPath.replace('/docs-software/', '/docs-urdu/');
      } else if (currentPath.includes('/docs-hardware/')) {
        targetPath = currentPath.replace('/docs-hardware/', '/docs-urdu/');
      } else if (currentPath.includes('/docs/')) {
        targetPath = currentPath.replace('/docs/', '/docs-urdu/');
      }

      history.push(targetPath + location.search + location.hash);
    }
  };

  return (
    <div style={{ marginBottom: '1rem' }}>
      <button
        onClick={handleTranslate}
        style={{
          padding: '8px 16px',
          backgroundColor: isUrdu ? '#28a745' : '#007bff',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: 'pointer',
          fontSize: '14px',
          fontWeight: '500',
          display: 'inline-flex',
          alignItems: 'center',
          gap: '8px',
          transition: 'background-color 0.2s'
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.backgroundColor = isUrdu ? '#218838' : '#0056b3';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.backgroundColor = isUrdu ? '#28a745' : '#007bff';
        }}
      >
        {isUrdu ? (
          <>
            ✓ Show Original
          </>
        ) : (
          <>
            🌐 Translate to Urdu
          </>
        )}
      </button>

      <style>{`
        @keyframes spin {
          to { transform: rotate(360deg); }
        }
      `}</style>
    </div>
  );
}
