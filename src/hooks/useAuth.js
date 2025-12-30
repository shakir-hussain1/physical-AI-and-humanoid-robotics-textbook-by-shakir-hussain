import { useState, useEffect, useCallback, useContext, createContext } from 'react';

// Create Auth Context
const AuthContext = createContext(null);

// Auth Hook
export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
}

// Auth Provider Component
export function AuthProvider({ children }) {
  const [user, setUser] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  const API_BASE_URL = 'http://localhost:8001/api';

  // Check if user is already logged in (token in localStorage)
  useEffect(() => {
    const checkAuth = async () => {
      try {
        const accessToken = localStorage.getItem('accessToken');
        const refreshToken = localStorage.getItem('refreshToken');
        const storedUser = localStorage.getItem('user');

        if (accessToken && storedUser) {
          setUser(JSON.parse(storedUser));
          setIsAuthenticated(true);
        } else if (refreshToken) {
          // Try to refresh token if refresh token exists
          await refreshAccessToken(refreshToken);
        }
      } catch (err) {
        console.error('Auth check failed:', err);
        localStorage.removeItem('accessToken');
        localStorage.removeItem('refreshToken');
        localStorage.removeItem('user');
      } finally {
        setIsLoading(false);
      }
    };

    checkAuth();
  }, []);

  // Sign up user
  const signup = useCallback(async (formData) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        const errData = await response.json();
        throw new Error(errData.detail || 'Signup failed');
      }

      const data = await response.json();

      // Store tokens and user info
      localStorage.setItem('accessToken', data.tokens.access_token);
      localStorage.setItem('refreshToken', data.tokens.refresh_token);
      localStorage.setItem('user', JSON.stringify(data.user));

      setUser(data.user);
      setIsAuthenticated(true);

      return data;
    } catch (err) {
      const errorMessage = err.message || 'Signup failed';
      setError(errorMessage);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, [API_BASE_URL]);

  // Sign in user
  const signin = useCallback(async (email, password) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (!response.ok) {
        const errData = await response.json();
        throw new Error(errData.detail || 'Login failed');
      }

      const data = await response.json();

      // Store tokens and user info
      localStorage.setItem('accessToken', data.tokens.access_token);
      localStorage.setItem('refreshToken', data.tokens.refresh_token);
      localStorage.setItem('user', JSON.stringify(data.user));

      setUser(data.user);
      setIsAuthenticated(true);

      return data;
    } catch (err) {
      const errorMessage = err.message || 'Login failed';
      setError(errorMessage);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, [API_BASE_URL]);

  // Refresh access token
  const refreshAccessToken = useCallback(async (refreshToken) => {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/refresh`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ refresh_token: refreshToken }),
      });

      if (!response.ok) {
        throw new Error('Token refresh failed');
      }

      const data = await response.json();

      localStorage.setItem('accessToken', data.access_token);
      localStorage.setItem('refreshToken', data.refresh_token);

      return data.access_token;
    } catch (err) {
      console.error('Token refresh failed:', err);
      // Clear auth data on refresh failure
      localStorage.removeItem('accessToken');
      localStorage.removeItem('refreshToken');
      localStorage.removeItem('user');
      setIsAuthenticated(false);
      setUser(null);
      throw err;
    }
  }, [API_BASE_URL]);

  // Get current user profile
  const getProfile = useCallback(async () => {
    try {
      const accessToken = localStorage.getItem('accessToken');
      if (!accessToken) {
        throw new Error('No access token');
      }

      const response = await fetch(`${API_BASE_URL}/auth/me`, {
        headers: {
          'Authorization': `Bearer ${accessToken}`,
        },
      });

      if (!response.ok) {
        throw new Error('Failed to fetch profile');
      }

      return await response.json();
    } catch (err) {
      console.error('Get profile failed:', err);
      throw err;
    }
  }, [API_BASE_URL]);

  // Update user profile
  const updateProfile = useCallback(async (profileData) => {
    setIsLoading(true);
    setError(null);

    try {
      const accessToken = localStorage.getItem('accessToken');
      if (!accessToken) {
        throw new Error('No access token');
      }

      const response = await fetch(`${API_BASE_URL}/auth/me/profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${accessToken}`,
        },
        body: JSON.stringify(profileData),
      });

      if (!response.ok) {
        const errData = await response.json();
        throw new Error(errData.detail || 'Profile update failed');
      }

      const data = await response.json();

      // Update stored user info
      const updatedUser = { ...user, ...data };
      localStorage.setItem('user', JSON.stringify(updatedUser));
      setUser(updatedUser);

      return data;
    } catch (err) {
      const errorMessage = err.message || 'Profile update failed';
      setError(errorMessage);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, [user, API_BASE_URL]);

  // Sign out user
  const signout = useCallback(async () => {
    try {
      const accessToken = localStorage.getItem('accessToken');
      if (accessToken) {
        // Call logout endpoint
        await fetch(`${API_BASE_URL}/auth/logout`, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${accessToken}`,
          },
        }).catch(() => {
          // Ignore errors on logout
        });
      }
    } finally {
      // Clear stored data
      localStorage.removeItem('accessToken');
      localStorage.removeItem('refreshToken');
      localStorage.removeItem('user');
      setUser(null);
      setIsAuthenticated(false);
      setError(null);
    }
  }, [API_BASE_URL]);

  // Get authorization header with token
  const getAuthHeader = useCallback(() => {
    const token = localStorage.getItem('accessToken');
    if (!token) {
      return {};
    }
    return {
      'Authorization': `Bearer ${token}`,
    };
  }, []);

  const value = {
    user,
    isAuthenticated,
    isLoading,
    error,
    signup,
    signin,
    signout,
    getProfile,
    updateProfile,
    refreshAccessToken,
    getAuthHeader,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
}
