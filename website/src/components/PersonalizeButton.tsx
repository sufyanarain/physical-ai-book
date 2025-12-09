import React, { useState, useEffect } from 'react';
import { useLocation, useHistory } from '@docusaurus/router';
import styles from './PersonalizeButton.module.css';

interface User {
  email: string;
  name: string;
  background_type?: 'software' | 'hardware';
}

export default function PersonalizeButton(): JSX.Element | null {
  const [user, setUser] = useState<User | null>(null);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const location = useLocation();
  const history = useHistory();

  useEffect(() => {
    // Get user from localStorage
    const userStr = localStorage.getItem('user');
    if (userStr) {
      try {
        const userData = JSON.parse(userStr);
        setUser(userData);
        
        // Check if we're already on a personalized page (English or Urdu)
        if (location.pathname.includes('/docs-software/') || 
            location.pathname.includes('/docs-hardware/') ||
            location.pathname.includes('/docs-urdu-software/') ||
            location.pathname.includes('/docs-urdu-hardware/')) {
          setIsPersonalized(true);
        } else {
          setIsPersonalized(false);
        }
      } catch (error) {
        console.error('Error parsing user data:', error);
      }
    }
  }, [location]);

  const handlePersonalize = () => {
    if (!user || !user.background_type) {
      alert('Please log in and set your background type to use personalization.');
      return;
    }

    // Determine current doc path
    let currentPath = location.pathname;
    
    // If already on personalized page, go back to default or Urdu
    if (isPersonalized) {
      // Check if we came from Urdu docs
      const isUrduPersonalized = currentPath.includes('/docs-urdu-software/') || currentPath.includes('/docs-urdu-hardware/');
      
      if (isUrduPersonalized) {
        // Go back to Urdu docs
        currentPath = currentPath
          .replace('/docs-urdu-software/', '/docs-urdu/')
          .replace('/docs-urdu-hardware/', '/docs-urdu/');
      } else {
        // Go back to default docs
        currentPath = currentPath
          .replace('/docs-software/', '/docs/')
          .replace('/docs-hardware/', '/docs/');
      }
      history.push(currentPath + location.search + location.hash);
      return;
    }

    // Extract the page path and navigate to personalized version
    if (currentPath.includes('/physical-ai-book/docs-urdu/')) {
      const pagePath = currentPath.replace('/physical-ai-book/docs-urdu/', '');
      const newPath = `/physical-ai-book/docs-urdu-${user.background_type}/${pagePath}`;
      history.push(newPath + location.search + location.hash);
    } else if (currentPath.includes('/physical-ai-book/docs/')) {
      const pagePath = currentPath.replace('/physical-ai-book/docs/', '');
      const newPath = `/physical-ai-book/docs-${user.background_type}/${pagePath}`;
      history.push(newPath + location.search + location.hash);
    } else if (currentPath.includes('/docs-urdu/')) {
      // Handle local development paths for Urdu
      const pagePath = currentPath.replace('/docs-urdu/', '');
      const newPath = `/docs-urdu-${user.background_type}/${pagePath}`;
      history.push(newPath + location.search + location.hash);
    } else if (currentPath.includes('/docs/')) {
      // Handle local development paths
      const pagePath = currentPath.replace('/docs/', '');
      const newPath = `/docs-${user.background_type}/${pagePath}`;
      history.push(newPath + location.search + location.hash);
    }
  };

  // Don't show button if user is not logged in or doesn't have background type
  if (!user || !user.background_type) {
    return null;
  }

  const backgroundLabel = user.background_type === 'software' ? '💻 Software' : '🔧 Hardware';

  return (
    <div className={styles.personalizeContainer}>
      <button
        className={`${styles.personalizeButton} ${isPersonalized ? styles.active : ''}`}
        onClick={handlePersonalize}
        title={isPersonalized ? 'View original version' : `Personalize for ${backgroundLabel} background`}
      >
        {isPersonalized ? (
          <>
            <span className={styles.icon}>✓</span>
            <span>Personalized ({backgroundLabel})</span>
          </>
        ) : (
          <>
            <span className={styles.icon}>🎯</span>
            <span>Personalize for {backgroundLabel}</span>
          </>
        )}
      </button>
      {isPersonalized && (
        <div className={styles.personalizeHint}>
          This content is adapted for your {backgroundLabel} background
        </div>
      )}
    </div>
  );
}
