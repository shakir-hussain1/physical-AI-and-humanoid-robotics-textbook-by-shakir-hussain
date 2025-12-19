import React, { useState } from 'react';
import styles from './AuthForm.module.css';

const getAPIUrl = () => {
  try {
    if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL) {
      return process.env.REACT_APP_API_URL;
    }
  } catch (e) {
    // process not available
  }

  if (typeof window !== 'undefined' && window.location.hostname === 'localhost') {
    return 'http://localhost:8000';
  }

  // GitHub Pages or production - use Hugging Face Spaces backend
  // Replace USERNAME with your Hugging Face username
  return 'https://shakir-rag-chatbot-backend.hf.space';
};

const API_URL = getAPIUrl();

export default function AuthForm({ onAuthSuccess }) {
  const [authMode, setAuthMode] = useState('signin'); // signin, signup, background
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  // Signin form state
  const [signInData, setSignInData] = useState({
    email: '',
    password: ''
  });

  // Signup form state
  const [signUpData, setSignUpData] = useState({
    email: '',
    username: '',
    password: '',
    confirm_password: '',
    full_name: ''
  });

  // Background questionnaire state
  const [backgroundData, setBackgroundData] = useState({
    software_background: 'beginner',
    hardware_background: 'beginner',
    programming_languages: [],
    interest_areas: [],
    profession: '',
    organization: '',
    years_of_experience: '',
    preferred_learning_style: 'text',
    learning_pace: 'moderate'
  });

  const programmingLanguages = ['Python', 'C++', 'C', 'Java', 'JavaScript', 'ROS', 'MATLAB', 'Rust'];
  const interestAreas = ['ROS', 'Kinematics', 'Control', 'Perception', 'Humanoid', 'AI/ML', 'Computer Vision', 'Sensor Fusion'];

  const handleSignIn = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const res = await fetch(`${API_URL}/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(signInData)
      });

      if (!res.ok) {
        const errorData = await res.json();
        throw new Error(errorData.detail || 'Signin failed');
      }

      const data = await res.json();
      localStorage.setItem('access_token', data.access_token);
      localStorage.setItem('refresh_token', data.refresh_token);
      localStorage.setItem('user', JSON.stringify(data.user));

      onAuthSuccess(data.user);
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  const handleSignUp = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const res = await fetch(`${API_URL}/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(signUpData)
      });

      if (!res.ok) {
        const errorData = await res.json();
        throw new Error(errorData.detail || 'Signup failed');
      }

      const data = await res.json();
      setSuccess('Account created! Please fill your background information.');
      setAuthMode('background');
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  const handleBackgroundSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      // First sign in to get token
      const signInRes = await fetch(`${API_URL}/auth/signin`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email: signUpData.email,
          password: signUpData.password
        })
      });

      if (!signInRes.ok) {
        throw new Error('Failed to get access token');
      }

      const tokens = await signInRes.json();
      localStorage.setItem('access_token', tokens.access_token);
      localStorage.setItem('refresh_token', tokens.refresh_token);

      // Save background profile
      const profileRes = await fetch(`${API_URL}/auth/profile/background`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${tokens.access_token}`
        },
        body: JSON.stringify(backgroundData)
      });

      if (!profileRes.ok) {
        throw new Error('Failed to save profile');
      }

      const profileData = await profileRes.json();
      localStorage.setItem('user', JSON.stringify(profileData.user));

      setSuccess('Profile complete! Redirecting...');
      setTimeout(() => onAuthSuccess(profileData.user), 1000);
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  const toggleLanguage = (lang) => {
    setBackgroundData(prev => ({
      ...prev,
      programming_languages: prev.programming_languages.includes(lang)
        ? prev.programming_languages.filter(l => l !== lang)
        : [...prev.programming_languages, lang]
    }));
  };

  const toggleInterest = (area) => {
    setBackgroundData(prev => ({
      ...prev,
      interest_areas: prev.interest_areas.includes(area)
        ? prev.interest_areas.filter(a => a !== area)
        : [...prev.interest_areas, area]
    }));
  };

  if (authMode === 'signin') {
    return (
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.title}>Sign In</h1>
          <p className={styles.subtitle}>Access your personalized chatbot</p>

          {error && <div className={styles.error}>{error}</div>}
          {success && <div className={styles.success}>{success}</div>}

          <form onSubmit={handleSignIn}>
            <div className={styles.formGroup}>
              <label>Email</label>
              <input
                type="email"
                value={signInData.email}
                onChange={(e) => setSignInData({...signInData, email: e.target.value})}
                placeholder="your@email.com"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label>Password</label>
              <input
                type="password"
                value={signInData.password}
                onChange={(e) => setSignInData({...signInData, password: e.target.value})}
                placeholder="••••••••"
                required
              />
            </div>

            <button type="submit" className={styles.submitBtn} disabled={loading}>
              {loading ? 'Signing in...' : 'Sign In'}
            </button>
          </form>

          <p className={styles.toggle}>
            Don't have an account?{' '}
            <button type="button" onClick={() => setAuthMode('signup')} className={styles.toggleBtn}>
              Sign Up
            </button>
          </p>
        </div>
      </div>
    );
  }

  if (authMode === 'signup') {
    return (
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.title}>Create Account</h1>
          <p className={styles.subtitle}>Join our learning community</p>

          {error && <div className={styles.error}>{error}</div>}

          <form onSubmit={handleSignUp}>
            <div className={styles.formGroup}>
              <label>Full Name</label>
              <input
                type="text"
                value={signUpData.full_name}
                onChange={(e) => setSignUpData({...signUpData, full_name: e.target.value})}
                placeholder="John Doe"
              />
            </div>

            <div className={styles.formGroup}>
              <label>Email</label>
              <input
                type="email"
                value={signUpData.email}
                onChange={(e) => setSignUpData({...signUpData, email: e.target.value})}
                placeholder="your@email.com"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label>Username</label>
              <input
                type="text"
                value={signUpData.username}
                onChange={(e) => setSignUpData({...signUpData, username: e.target.value})}
                placeholder="johndoe"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label>Password</label>
              <input
                type="password"
                value={signUpData.password}
                onChange={(e) => setSignUpData({...signUpData, password: e.target.value})}
                placeholder="Min 8 chars, 1 uppercase, 1 number"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label>Confirm Password</label>
              <input
                type="password"
                value={signUpData.confirm_password}
                onChange={(e) => setSignUpData({...signUpData, confirm_password: e.target.value})}
                placeholder="••••••••"
                required
              />
            </div>

            <button type="submit" className={styles.submitBtn} disabled={loading}>
              {loading ? 'Creating account...' : 'Sign Up'}
            </button>
          </form>

          <p className={styles.toggle}>
            Already have an account?{' '}
            <button type="button" onClick={() => setAuthMode('signin')} className={styles.toggleBtn}>
              Sign In
            </button>
          </p>
        </div>
      </div>
    );
  }

  if (authMode === 'background') {
    return (
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.title}>Your Background</h1>
          <p className={styles.subtitle}>Help us personalize your learning experience</p>

          {error && <div className={styles.error}>{error}</div>}
          {success && <div className={styles.success}>{success}</div>}

          <form onSubmit={handleBackgroundSubmit} className={styles.backgroundForm}>
            <div className={styles.section}>
              <h3>Experience Level</h3>

              <div className={styles.formGroup}>
                <label>Software Background</label>
                <select
                  value={backgroundData.software_background}
                  onChange={(e) => setBackgroundData({...backgroundData, software_background: e.target.value})}
                >
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                  <option value="expert">Expert</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label>Hardware/Robotics Background</label>
                <select
                  value={backgroundData.hardware_background}
                  onChange={(e) => setBackgroundData({...backgroundData, hardware_background: e.target.value})}
                >
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                  <option value="expert">Expert</option>
                </select>
              </div>
            </div>

            <div className={styles.section}>
              <h3>Programming Languages (Select all that apply)</h3>
              <div className={styles.checkboxGrid}>
                {programmingLanguages.map(lang => (
                  <label key={lang} className={styles.checkbox}>
                    <input
                      type="checkbox"
                      checked={backgroundData.programming_languages.includes(lang)}
                      onChange={() => toggleLanguage(lang)}
                    />
                    {lang}
                  </label>
                ))}
              </div>
            </div>

            <div className={styles.section}>
              <h3>Interest Areas (Select all that apply)</h3>
              <div className={styles.checkboxGrid}>
                {interestAreas.map(area => (
                  <label key={area} className={styles.checkbox}>
                    <input
                      type="checkbox"
                      checked={backgroundData.interest_areas.includes(area)}
                      onChange={() => toggleInterest(area)}
                    />
                    {area}
                  </label>
                ))}
              </div>
            </div>

            <div className={styles.section}>
              <h3>Professional Background</h3>

              <div className={styles.formGroup}>
                <label>Profession</label>
                <input
                  type="text"
                  value={backgroundData.profession}
                  onChange={(e) => setBackgroundData({...backgroundData, profession: e.target.value})}
                  placeholder="e.g., Student, Engineer, Researcher"
                />
              </div>

              <div className={styles.formGroup}>
                <label>Organization</label>
                <input
                  type="text"
                  value={backgroundData.organization}
                  onChange={(e) => setBackgroundData({...backgroundData, organization: e.target.value})}
                  placeholder="e.g., University, Company"
                />
              </div>

              <div className={styles.formGroup}>
                <label>Years of Experience</label>
                <input
                  type="number"
                  value={backgroundData.years_of_experience}
                  onChange={(e) => setBackgroundData({...backgroundData, years_of_experience: parseInt(e.target.value) || ''})}
                  placeholder="0"
                  min="0"
                  max="60"
                />
              </div>
            </div>

            <div className={styles.section}>
              <h3>Learning Preferences</h3>

              <div className={styles.formGroup}>
                <label>Preferred Learning Style</label>
                <select
                  value={backgroundData.preferred_learning_style}
                  onChange={(e) => setBackgroundData({...backgroundData, preferred_learning_style: e.target.value})}
                >
                  <option value="visual">Visual (Diagrams, Videos)</option>
                  <option value="text">Text-based (Reading)</option>
                  <option value="interactive">Interactive (Exercises)</option>
                  <option value="hands-on">Hands-on (Coding)</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label>Learning Pace</label>
                <select
                  value={backgroundData.learning_pace}
                  onChange={(e) => setBackgroundData({...backgroundData, learning_pace: e.target.value})}
                >
                  <option value="slow">Slow (Detailed explanations)</option>
                  <option value="moderate">Moderate (Balanced)</option>
                  <option value="fast">Fast (Quick overview)</option>
                </select>
              </div>
            </div>

            <button type="submit" className={styles.submitBtn} disabled={loading}>
              {loading ? 'Saving profile...' : 'Complete Profile'}
            </button>
          </form>
        </div>
      </div>
    );
  }
}
