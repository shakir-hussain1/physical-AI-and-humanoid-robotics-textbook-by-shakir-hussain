import React from 'react';
import { AuthProvider } from '../context/AuthContext';
import ChatWidget from '../components/ChatWidget';
import NavbarAuthWidget from '../components/NavbarAuthWidget';
import styles from './Root.module.css';

export default function Root({ children }) {
  return (
    <AuthProvider>
      <>
        <div className={styles.navbarAuthOverlay}>
          <NavbarAuthWidget />
        </div>
        {children}
        <ChatWidget />
      </>
    </AuthProvider>
  );
}