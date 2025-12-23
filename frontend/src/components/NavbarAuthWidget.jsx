import React, { useState } from 'react';
import { useAuth } from '../context/AuthContext';
import AuthForm from './AuthForm';
import styles from './NavbarAuthWidget.module.css';

export default function NavbarAuthWidget() {
  const [showModal, setShowModal] = useState(false);
  const { user, isAuthenticated, logout, loading } = useAuth();

  if (loading) {
    return <div className={styles.loadingSpinner}></div>;
  }

  if (isAuthenticated && user) {
    return (
      <div className={styles.userContainer}>
        <div className={styles.userInfo}>
          <div className={styles.avatar}>
            {user.email?.charAt(0).toUpperCase()}
          </div>
          <div className={styles.userDetails}>
            <span className={styles.username}>{user.email?.split('@')[0]}</span>
            {user.software_background && (
              <span className={styles.level}>{user.software_background}</span>
            )}
          </div>
        </div>
        <button
          className={styles.logoutBtn}
          onClick={logout}
          title="Logout"
        >
          ↗
        </button>
      </div>
    );
  }

  return (
    <>
      <button
        className={styles.signInBtn}
        onClick={() => setShowModal(true)}
      >
        Sign In
      </button>

      {showModal && (
        <div
          className={styles.modalBackdrop}
          onClick={() => setShowModal(false)}
        >
          <div
            className={styles.modalContent}
            onClick={e => e.stopPropagation()}
          >
            <button
              className={styles.closeBtn}
              onClick={() => setShowModal(false)}
              title="Close"
            >
              ✕
            </button>
            <AuthForm
              onAuthSuccess={(user) => {
                console.log('User authenticated:', user);
                setShowModal(false);
              }}
            />
          </div>
        </div>
      )}
    </>
  );
}
