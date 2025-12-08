import React, { useState, useEffect } from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import AuthModal from '@site/src/components/AuthModal';
import type { Props } from '@theme/NavbarItem';

export default function NavbarItemWrapper(props: Props): JSX.Element {
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [user, setUser] = useState<any>(null);

  useEffect(() => {
    // Check if user is logged in
    const storedUser = localStorage.getItem('user');
    if (storedUser) {
      try {
        setUser(JSON.parse(storedUser));
      } catch (e) {
        localStorage.removeItem('user');
        localStorage.removeItem('auth_token');
      }
    }
  }, []);

  const handleLogout = () => {
    localStorage.removeItem('user');
    localStorage.removeItem('auth_token');
    setUser(null);
  };

  const handleAuthSuccess = (userData: any, token: string) => {
    setUser(userData);
    setShowAuthModal(false);
  };

  // Add auth button to navbar
  if (props.label === 'GitHub') {
    return (
      <>
        {user ? (
          <div style={{ display: 'flex', alignItems: 'center', gap: '12px', marginRight: '8px' }}>
            <span style={{ fontSize: '14px', color: 'var(--ifm-navbar-link-color)' }}>
              {user.name}
            </span>
            <button
              onClick={handleLogout}
              style={{
                padding: '6px 12px',
                background: 'rgba(102, 126, 234, 0.1)',
                color: '#667eea',
                border: 'none',
                borderRadius: '6px',
                cursor: 'pointer',
                fontSize: '13px',
                fontWeight: '600'
              }}
            >
              Logout
            </button>
          </div>
        ) : (
          <button
            onClick={() => setShowAuthModal(true)}
            className="user-profile-btn"
            style={{ marginRight: '12px' }}
          >
            👤 Sign In
          </button>
        )}
        {showAuthModal && (
          <AuthModal
            onClose={() => setShowAuthModal(false)}
            onSuccess={handleAuthSuccess}
          />
        )}
        <NavbarItem {...props} />
      </>
    );
  }

  return <NavbarItem {...props} />;
}
