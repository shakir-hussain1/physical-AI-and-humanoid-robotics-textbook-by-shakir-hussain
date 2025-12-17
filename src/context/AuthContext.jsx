import React, { createContext, useState, useEffect } from 'react';

export const AuthContext = createContext();

export function AuthProvider({ children }) {
  const [user, setUser] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [loading, setLoading] = useState(true);
  const [tokens, setTokens] = useState({});
  const [personalizationData, setPersonalizationData] = useState(null);

  // Initialize from localStorage
  useEffect(() => {
    const savedUser = localStorage.getItem('user');
    const accessToken = localStorage.getItem('access_token');

    if (savedUser && accessToken) {
      try {
        const userData = JSON.parse(savedUser);
        setUser(userData);
        setIsAuthenticated(true);
        setTokens({
          access_token: accessToken,
          refresh_token: localStorage.getItem('refresh_token')
        });
        fetchPersonalizationData(accessToken);
      } catch (err) {
        console.error('Error loading auth state:', err);
        logout();
      }
    }
    setLoading(false);
  }, []);

  const fetchPersonalizationData = async (accessToken) => {
    try {
      const response = await fetch('http://localhost:8000/auth/personalization', {
        headers: {
          'Authorization': `Bearer ${accessToken}`
        }
      });

      if (response.ok) {
        const data = await response.json();
        setPersonalizationData(data.data);
      }
    } catch (err) {
      console.error('Error fetching personalization data:', err);
    }
  };

  const login = (userData, tokens) => {
    setUser(userData);
    setIsAuthenticated(true);
    setTokens(tokens);
    localStorage.setItem('user', JSON.stringify(userData));
    localStorage.setItem('access_token', tokens.access_token);
    localStorage.setItem('refresh_token', tokens.refresh_token);
    fetchPersonalizationData(tokens.access_token);
  };

  const logout = () => {
    setUser(null);
    setIsAuthenticated(false);
    setTokens({});
    setPersonalizationData(null);
    localStorage.removeItem('user');
    localStorage.removeItem('access_token');
    localStorage.removeItem('refresh_token');
  };

  return (
    <AuthContext.Provider value={{
      user,
      isAuthenticated,
      loading,
      tokens,
      personalizationData,
      login,
      logout
    }}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = React.useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
}
