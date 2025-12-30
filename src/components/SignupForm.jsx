import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';
import { BackgroundQuestionnaire } from './BackgroundQuestionnaire';
import styles from './AuthForms.module.css';

export function SignupForm({ onSuccess, onSwitchToLogin }) {
  const { signup, isLoading, error: authError } = useAuth();
  const [step, setStep] = useState('basic'); // 'basic' or 'background'
  const [basicData, setBasicData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    firstName: '',
    lastName: '',
  });
  const [error, setError] = useState('');
  const [showPassword, setShowPassword] = useState(false);

  const handleBasicChange = (e) => {
    const { name, value } = e.target;
    setBasicData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const validateBasicForm = () => {
    setError('');

    if (!basicData.email.trim()) {
      setError('Email is required');
      return false;
    }

    if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(basicData.email)) {
      setError('Please enter a valid email');
      return false;
    }

    if (!basicData.password) {
      setError('Password is required');
      return false;
    }

    if (basicData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return false;
    }

    if (!/[A-Z]/.test(basicData.password)) {
      setError('Password must contain at least one uppercase letter');
      return false;
    }

    if (!/[0-9]/.test(basicData.password)) {
      setError('Password must contain at least one number');
      return false;
    }

    if (basicData.password !== basicData.confirmPassword) {
      setError('Passwords do not match');
      return false;
    }

    return true;
  };

  const handleBasicSubmit = (e) => {
    e.preventDefault();
    if (validateBasicForm()) {
      setStep('background');
    }
  };

  const handleBackgroundSubmit = async (backgroundData) => {
    try {
      setError('');

      const formData = {
        email: basicData.email,
        password: basicData.password,
        first_name: basicData.firstName || undefined,
        last_name: basicData.lastName || undefined,
        software_background: backgroundData.softwareBackground || undefined,
        hardware_background: backgroundData.hardwareBackground || undefined,
        programming_languages: backgroundData.programmingLanguages || undefined,
        robotics_experience: backgroundData.roboticsExperience || undefined,
        ai_ml_experience: backgroundData.aiMlExperience || undefined,
      };

      // Remove undefined values
      Object.keys(formData).forEach(
        (key) => formData[key] === undefined && delete formData[key]
      );

      await signup(formData);

      if (onSuccess) {
        onSuccess();
      }
    } catch (err) {
      setError(authError || err.message || 'Signup failed');
    }
  };

  const handleBackToBascic = () => {
    setStep('basic');
  };

  if (step === 'background') {
    return (
      <BackgroundQuestionnaire
        onSubmit={handleBackgroundSubmit}
        onBack={handleBackToBascic}
        isLoading={isLoading}
      />
    );
  }

  return (
    <div className={styles.formContainer}>
      <div className={styles.formHeader}>
        <h2>Create Account</h2>
        <p>Join us to get started. Step 1 of 2</p>
      </div>

      {error && <div className={styles.errorAlert}>{error}</div>}
      {authError && <div className={styles.errorAlert}>{authError}</div>}

      <form onSubmit={handleBasicSubmit} className={styles.form}>
        <div className={styles.formRow}>
          <div className={styles.formGroup}>
            <label htmlFor="firstName" className={styles.label}>
              First Name (Optional)
            </label>
            <input
              id="firstName"
              type="text"
              name="firstName"
              value={basicData.firstName}
              onChange={handleBasicChange}
              placeholder="John"
              className={styles.input}
              disabled={isLoading}
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="lastName" className={styles.label}>
              Last Name (Optional)
            </label>
            <input
              id="lastName"
              type="text"
              name="lastName"
              value={basicData.lastName}
              onChange={handleBasicChange}
              placeholder="Doe"
              className={styles.input}
              disabled={isLoading}
            />
          </div>
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="email" className={styles.label}>
            Email Address
          </label>
          <input
            id="email"
            type="email"
            name="email"
            value={basicData.email}
            onChange={handleBasicChange}
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
              name="password"
              value={basicData.password}
              onChange={handleBasicChange}
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
          <div className={styles.passwordHint}>
            Min 8 chars, 1 uppercase, 1 number
          </div>
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="confirmPassword" className={styles.label}>
            Confirm Password
          </label>
          <input
            id="confirmPassword"
            type={showPassword ? 'text' : 'password'}
            name="confirmPassword"
            value={basicData.confirmPassword}
            onChange={handleBasicChange}
            placeholder="••••••••"
            className={styles.input}
            disabled={isLoading}
            required
          />
        </div>

        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? 'Creating Account...' : 'Next: Tell Us About Yourself'}
        </button>
      </form>

      <div className={styles.formFooter}>
        <p>
          Already have an account?{' '}
          <button
            type="button"
            onClick={onSwitchToLogin}
            className={styles.switchLink}
            disabled={isLoading}
          >
            Sign In
          </button>
        </p>
      </div>
    </div>
  );
}
