// Wrapper hook for backward compatibility - uses AuthContext
import { useContext } from 'react';
import { AuthContext } from '../context/AuthContext';

export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
}

// Re-export AuthProvider from context for backward compatibility
export { AuthProvider } from '../context/AuthContext';
