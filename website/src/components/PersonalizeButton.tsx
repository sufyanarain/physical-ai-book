import React, { useState, useEffect } from 'react';

const BACKEND_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://physical-ai-backend-production-b62f.up.railway.app';

interface PersonalizeButtonProps {
  content: string;
  onPersonalize: (personalizedContent: string) => void;
}

export default function PersonalizeButton({ content, onPersonalize }: PersonalizeButtonProps): React.ReactElement | null {
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [user, setUser] = useState<any>(null);
  const [error, setError] = useState<string>('');

  useEffect(() => {
    // Check if user is logged in
    const storedUser = localStorage.getItem('user');
    if (storedUser) {
      try {
        setUser(JSON.parse(storedUser));
      } catch (e) {
        console.error('Error parsing user data:', e);
      }
    }
  }, []);

  // Only show button if user is logged in
  if (!user) {
    return null;
  }

  const handlePersonalize = async () => {
    if (!content || content.length < 50) {
      setError('Content too short to personalize');
      return;
    }

    setIsLoading(true);
    setError('');

    try {
      const token = localStorage.getItem('auth_token');
      if (!token) {
        setError('Please sign in to personalize content');
        return;
      }

      const response = await fetch(`${BACKEND_URL}/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          content: content,
          software_experience: user.software_experience || 'beginner',
          hardware_experience: user.hardware_experience || 'beginner',
          learning_goals: user.learning_goals || '',
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to personalize content');
      }

      const data = await response.json();
      setIsPersonalized(true);
      onPersonalize(data.personalized_content);
    } catch (err: any) {
      setError(err.message || 'Personalization failed');
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    setIsPersonalized(false);
    onPersonalize('');
  };

  return (
    <div className="personalize-button-container">
      {!isPersonalized ? (
        <button
          onClick={handlePersonalize}
          disabled={isLoading}
          className="personalize-button"
        >
          {isLoading ? (
            <>
              <span className="spinner"></span>
              Personalizing for {user.name}...
            </>
          ) : (
            <>
              🎯 Personalize Content for Me
            </>
          )}
        </button>
      ) : (
        <div className="personalize-info">
          <span className="personalize-badge">
            ✓ Personalized for your {user.software_experience} level
          </span>
          <button onClick={handleReset} className="reset-button">
            Reset to Original
          </button>
        </div>
      )}
      {error && <div className="personalize-error">{error}</div>}
    </div>
  );
}
