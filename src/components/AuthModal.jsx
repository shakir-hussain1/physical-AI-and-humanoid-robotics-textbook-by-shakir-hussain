import React, { useState, useEffect } from 'react';
import { LoginForm } from './LoginForm';
import { SignupForm } from './SignupForm';
import styles from './AuthModal.module.css';

/**
 * AuthModal component that handles both login and signup flows
 * Can be used as a modal overlay or standalone page component
 */
export function AuthModal({ onClose, onAuthSuccess, defaultMode = 'login' }) {
  const [mode, setMode] = useState(defaultMode);

  const handleSuccess = () => {
    if (onAuthSuccess) {
      onAuthSuccess();
    }
    if (onClose) {
      onClose();
    }
  };

  const handleSwitchMode = (newMode) => {
    setMode(newMode);
  };

  return (
    <div className={styles.modal}>
      <div className={styles.modalContent}>
        {onClose && (
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close authentication modal"
          >
            &times;
          </button>
        )}

        {mode === 'login' ? (
          <LoginForm
            onSuccess={handleSuccess}
            onSwitchToSignup={() => handleSwitchMode('signup')}
          />
        ) : (
          <SignupForm
            onSuccess={handleSuccess}
            onSwitchToLogin={() => handleSwitchMode('login')}
          />
        )}
      </div>
    </div>
  );
}
