import React, { useEffect } from 'react';
import { useLocation, useHistory } from '@docusaurus/router';
import RAGChatbot from '../components/RAGChatbot';

export default function Root({children}: {children: React.ReactNode}): React.ReactElement {
  const location = useLocation();
  const history = useHistory();

  useEffect(() => {
    // Get user from localStorage
    const userStr = localStorage.getItem('user');
    if (!userStr) return;

    try {
      const user = JSON.parse(userStr);
      const backgroundType = user.background_type; // 'software' or 'hardware'

      if (!backgroundType) return;

      // Check if user is on default docs path
      if (location.pathname.startsWith('/physical-ai-book/docs/')) {
        // Extract the page path after /docs/
        const pagePath = location.pathname.replace('/physical-ai-book/docs/', '');
        
        // Redirect to background-specific docs
        const newPath = `/physical-ai-book/docs-${backgroundType}/${pagePath}`;
        
        // Only redirect if we're not already on the correct path
        if (!location.pathname.startsWith(`/physical-ai-book/docs-${backgroundType}/`)) {
          history.replace(newPath + location.search + location.hash);
        }
      }
    } catch (error) {
      console.error('Error parsing user data:', error);
    }
  }, [location, history]);

  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
}
