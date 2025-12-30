import React, { useState } from 'react';
import { AuthProvider, useAuth } from '../../hooks/useAuth';
import ChatbotWidget from '../../components/ChatbotWidget';
import { AuthModal } from '../../components/AuthModal';
import { PersonalizationPanel } from '../../components/PersonalizationPanel';
import { UrduTranslationPanel } from '../../components/UrduTranslationPanel';
import LayoutOriginal from '@theme-original/Layout';
import '../Root.module.css';

function LayoutContent({ children }) {
  const { user, signout } = useAuth();
  const [showAuth, setShowAuth] = useState(false);
  const [showPersonalization, setShowPersonalization] = useState(false);
  const [showTranslation, setShowTranslation] = useState(false);

  const handleAuthSuccess = () => {
    setShowAuth(false);
  };

  const handleLogout = async () => {
    await signout();
  };

  return (
    <div className="root-container">
      {/* Custom Navbar */}
      <nav className="top-navbar">
        <div className="navbar-content">
          <div className="navbar-left">
            <h1>📚 Physical AI & Robotics</h1>
          </div>

          <div className="navbar-right">
            {user ? (
              <div className="user-section">
                <span className="user-info">👤 {user.email}</span>
                <button className="nav-button personalization-btn" onClick={() => setShowPersonalization(true)}>
                  ⚙️ Preferences
                </button>
                <button className="nav-button translation-btn" onClick={() => setShowTranslation(true)}>
                  🌐 ترجمہ
                </button>
                <button className="nav-button logout-btn" onClick={handleLogout}>
                  🚪 Logout
                </button>
              </div>
            ) : (
              <button className="nav-button login-btn" onClick={() => setShowAuth(true)}>
                🔐 Sign In
              </button>
            )}
          </div>
        </div>
      </nav>

      {/* Main Content */}
      <main className="main-content">
        <LayoutOriginal>
          {children}
        </LayoutOriginal>
      </main>

      {/* Auth Modal */}
      {showAuth && <AuthModal onAuthSuccess={handleAuthSuccess} onClose={() => setShowAuth(false)} />}

      {/* Personalization Panel */}
      {showPersonalization && user && (
        <div className="panel-overlay">
          <div className="panel-container">
            <button className="panel-close" onClick={() => setShowPersonalization(false)}>✕</button>
            <PersonalizationPanel onClose={() => setShowPersonalization(false)} />
          </div>
        </div>
      )}

      {/* Translation Panel */}
      {showTranslation && user && (
        <div className="panel-overlay">
          <div className="panel-container translation-panel">
            <UrduTranslationPanel onClose={() => setShowTranslation(false)} />
          </div>
        </div>
      )}

      {/* Chatbot */}
      <ChatbotWidget />
    </div>
  );
}

export default function Layout(props) {
  return (
    <AuthProvider>
      <LayoutContent {...props} />
    </AuthProvider>
  );
}
