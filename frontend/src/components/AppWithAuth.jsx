import React from 'react';
import { useAuth } from '../context/AuthContext';
import AuthForm from './AuthForm';
import ChatWidget from './ChatWidget';
import UserMenu from './UserMenu';
import styles from './AppWithAuth.module.css';

export default function AppWithAuth() {
  const { isAuthenticated, loading, user, login } = useAuth();

  if (loading) {
    return (
      <div className={styles.loadingContainer}>
        <div className={styles.spinner}></div>
        <p>Loading...</p>
      </div>
    );
  }

  if (!isAuthenticated) {
    return <AuthForm onAuthSuccess={login} />;
  }

  return (
    <div className={styles.appContainer}>
      <UserMenu user={user} />
      <ChatWidget />
    </div>
  );
}
