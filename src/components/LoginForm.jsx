import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';
import styles from './AuthForms.module.css';

export function LoginForm({ onSuccess, onSwitchToSignup }) {
  const { signin, isLoading, error: authError } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [showPassword, setShowPassword] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    // Validation
    if (!email.trim()) {
      setError('Email is required');
      return;
    }

    if (!password) {
      setError('Password is required');
      return;
    }

    if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email)) {
      setError('Please enter a valid email');
      return;
    }

    try {
      await signin(email, password);
      if (onSuccess) {
        onSuccess();
      }
    } catch (err) {
      setError(authError || err.message || 'Login failed');
    }
  };

  return (
    <div className={styles.formContainer}>
      <div className={styles.formHeader}>
        <h2>Sign In</h2>
        <p>Welcome back! Please sign in to your account</p>
      </div>

      {error && <div className={styles.errorAlert}>{error}</div>}
      {authError && <div className={styles.errorAlert}>{authError}</div>}

      <form onSubmit={handleSubmit} className={styles.form}>
        <div className={styles.formGroup}>
          <label htmlFor="email" className={styles.label}>
            Email Address
          </label>
          <input
            id="email"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="you@example.com"
            className={styles.input}
            disabled={isLoading}
            required
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password" className={styles.label}>
            Password
          </label>
          <div className={styles.passwordInput}>
            <input
              id="password"
              type={showPassword ? 'text' : 'password'}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="••••••••"
              className={styles.input}
              disabled={isLoading}
              required
            />
            <button
              type="button"
              className={styles.togglePassword}
              onClick={() => setShowPassword(!showPassword)}
              disabled={isLoading}
            >
              {showPassword ? 'Hide' : 'Show'}
            </button>
          </div>
        </div>

        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>

      <div className={styles.formFooter}>
        <p>
          Don't have an account?{' '}
          <button
            type="button"
            onClick={onSwitchToSignup}
            className={styles.switchLink}
            disabled={isLoading}
          >
            Sign Up
          </button>
        </p>
      </div>

      <div className={styles.testCredentials}>
        <p className={styles.testLabel}>Demo Credentials (for testing):</p>
        <code>email: test@example.com</code>
        <code>password: Test@1234</code>
      </div>
    </div>
  );
}
