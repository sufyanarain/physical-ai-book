import React, { useState, useEffect } from 'react';
import NavbarItem from '@theme-original/NavbarItem';
import AuthModal from '@site/src/components/AuthModal';
import { useHistory, useLocation } from '@docusaurus/router';
import type { Props } from '@theme/NavbarItem';

export default function NavbarItemWrapper(props: Props): JSX.Element {
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [user, setUser] = useState<any>(null);
  const [isUrdu, setIsUrdu] = useState(false);
  const history = useHistory();
  const location = useLocation();

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

    // Check if on Urdu docs
    setIsUrdu(location.pathname.includes('/docs-urdu/'));
  }, [location.pathname]);

  const handleLogout = () => {
    localStorage.removeItem('user');
    localStorage.removeItem('auth_token');
    setUser(null);
    // Redirect to home page after logout
    if (location.pathname.includes('/docs-software/') || location.pathname.includes('/docs-hardware/')) {
      history.push('/physical-ai-book/');
    }
  };

  const handleAuthSuccess = (userData: any, token: string) => {
    setUser(userData);
    setShowAuthModal(false);

    // Redirect to personalized docs after login/signup
    const backgroundType = userData.background_type; // 'software' or 'hardware'

    if (backgroundType) {
      // If user is on docs page, redirect to their personalized version
      if (location.pathname.startsWith('/physical-ai-book/docs/')) {
        const pagePath = location.pathname.replace('/physical-ai-book/docs/', '');
        const newPath = `/physical-ai-book/docs-${backgroundType}/${pagePath}`;
        history.push(newPath + location.search + location.hash);
      } else if (!location.pathname.includes('/docs-software/') && !location.pathname.includes('/docs-hardware/')) {
        // If user is not on a docs page, redirect to intro of their personalized docs
        history.push(`/physical-ai-book/docs-${backgroundType}/intro`);
      }
    }
  };

  const handleTranslate = () => {
    const currentPath = location.pathname;

    if (isUrdu) {
      // Switch back to original (check if user is logged in)
      const userStr = localStorage.getItem('user');
      let targetPath = currentPath.replace('/docs-urdu/', '/docs/');

      if (userStr) {
        try {
          const user = JSON.parse(userStr);
          const backgroundType = user.background_type;
          if (backgroundType) {
            targetPath = currentPath.replace('/docs-urdu/', `/docs-${backgroundType}/`);
          }
        } catch (e) {
          console.error('Error parsing user data:', e);
        }
      }
      history.push(targetPath + location.search + location.hash);
    } else {
      // Switch to Urdu
      let targetPath = currentPath;
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

  // Check if we're on a docs page to show translate button
  const isDocsPage = location.pathname.includes('/docs/') ||
                     location.pathname.includes('/docs-software/') ||
                     location.pathname.includes('/docs-hardware/') ||
                     location.pathname.includes('/docs-urdu/');

  // Add translate and auth buttons to navbar
  if (props.label === 'GitHub') {
    return (
      <>
        {/* Translate Button - only show on docs pages */}
        {isDocsPage && (
          <button
            onClick={handleTranslate}
            style={{
              padding: '6px 14px',
              backgroundColor: isUrdu ? '#28a745' : '#2e8555',
              color: 'white',
              border: 'none',
              borderRadius: '6px',
              cursor: 'pointer',
              fontSize: '13px',
              fontWeight: '600',
              marginRight: '12px',
              display: 'inline-flex',
              alignItems: 'center',
              gap: '6px',
              transition: 'all 0.2s ease',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.backgroundColor = isUrdu ? '#218838' : '#246944';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.backgroundColor = isUrdu ? '#28a745' : '#2e8555';
            }}
          >
            {isUrdu ? '✓ اصل دکھائیں' : '🌐 اردو'}
          </button>
        )}

        {/* User Auth Section */}
        {user ? (
          <div style={{
            display: 'flex',
            alignItems: 'center',
            gap: '10px',
            marginRight: '8px',
            padding: '4px 12px',
            background: 'var(--ifm-color-emphasis-100)',
            borderRadius: '8px',
            border: '1px solid var(--ifm-color-emphasis-300)'
          }}>
            <div style={{
              display: 'flex',
              alignItems: 'center',
              gap: '8px'
            }}>
              <div style={{
                width: '32px',
                height: '32px',
                borderRadius: '50%',
                background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: 'white',
                fontWeight: '700',
                fontSize: '14px'
              }}>
                {user.name?.charAt(0).toUpperCase() || 'U'}
              </div>
              <div style={{ display: 'flex', flexDirection: 'column', gap: '2px' }}>
                <span style={{
                  fontSize: '13px',
                  fontWeight: '600',
                  color: 'var(--ifm-navbar-link-color)',
                  lineHeight: '1'
                }}>
                  {user.name}
                </span>
                <span style={{
                  fontSize: '11px',
                  color: 'var(--ifm-color-emphasis-700)',
                  lineHeight: '1'
                }}>
                  {user.background_type === 'software' ? '💻 Software' : '🔧 Hardware'}
                </span>
              </div>
            </div>
            <button
              onClick={handleLogout}
              style={{
                padding: '5px 10px',
                background: 'transparent',
                color: 'var(--ifm-color-emphasis-800)',
                border: '1px solid var(--ifm-color-emphasis-400)',
                borderRadius: '5px',
                cursor: 'pointer',
                fontSize: '12px',
                fontWeight: '600',
                transition: 'all 0.2s ease'
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.background = 'var(--ifm-color-emphasis-200)';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.background = 'transparent';
              }}
            >
              Logout
            </button>
          </div>
        ) : (
          <button
            onClick={() => setShowAuthModal(true)}
            style={{
              padding: '6px 14px',
              background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
              color: 'white',
              border: 'none',
              borderRadius: '6px',
              cursor: 'pointer',
              fontSize: '13px',
              fontWeight: '600',
              marginRight: '12px',
              display: 'inline-flex',
              alignItems: 'center',
              gap: '6px',
              transition: 'all 0.2s ease'
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.transform = 'translateY(-1px)';
              e.currentTarget.style.boxShadow = '0 4px 12px rgba(102, 126, 234, 0.4)';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.transform = 'translateY(0)';
              e.currentTarget.style.boxShadow = 'none';
            }}
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
