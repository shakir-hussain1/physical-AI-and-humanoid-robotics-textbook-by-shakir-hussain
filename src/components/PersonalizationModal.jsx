import React, { useState, useEffect } from 'react';
import { resetChapterPersonalization } from '../services/personalizationApi';
import { useAuth } from '../context/AuthContext';
import styles from './PersonalizationModal.module.css';

export default function PersonalizationModal({
  chapterId,
  chapterTitle,
  currentPreferences,
  onSave,
  onClose,
  isLoading
}) {
  const { tokens } = useAuth();
  const [preferences, setPreferences] = useState({
    difficulty_level: 'intermediate',
    content_style: 'balanced',
    example_density: 'moderate',
    learning_pace: 'standard',
    custom_preferences: {}
  });
  const [error, setError] = useState('');
  const [resetting, setResetting] = useState(false);

  // Load current preferences if available
  useEffect(() => {
    if (currentPreferences) {
      setPreferences({
        difficulty_level: currentPreferences.difficulty_level || 'intermediate',
        content_style: currentPreferences.content_style || 'balanced',
        example_density: currentPreferences.example_density || 'moderate',
        learning_pace: currentPreferences.learning_pace || 'standard',
        custom_preferences: currentPreferences.custom_preferences || {}
      });
    }
  }, [currentPreferences]);

  const handleDifficultyChange = (level) => {
    setPreferences(prev => ({ ...prev, difficulty_level: level }));
    setError('');
  };

  const handleStyleChange = (style) => {
    setPreferences(prev => ({ ...prev, content_style: style }));
    setError('');
  };

  const handleDensityChange = (density) => {
    setPreferences(prev => ({ ...prev, example_density: density }));
    setError('');
  };

  const handlePaceChange = (pace) => {
    setPreferences(prev => ({ ...prev, learning_pace: pace }));
    setError('');
  };

  const handleSave = async () => {
    try {
      setError('');
      console.log('[Modal] Saving preferences:', { preferences, chapterId });
      await onSave(preferences);
    } catch (err) {
      const errorMsg = err.message || 'Failed to save preferences. Please try again.';
      console.error('[Modal] Save error:', errorMsg);
      setError(errorMsg);
    }
  };

  const handleReset = async () => {
    if (!window.confirm('Reset this chapter to default settings?')) {
      return;
    }

    try {
      setResetting(true);
      setError('');
      await resetChapterPersonalization(chapterId, tokens.access_token);
      onClose();
      window.location.reload();
    } catch (err) {
      setError('Failed to reset preferences. Please try again.');
      console.error('Reset error:', err);
    } finally {
      setResetting(false);
    }
  };

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.modal} onClick={e => e.stopPropagation()}>
        <div className={styles.header}>
          <h2 className={styles.title}>Personalize Chapter</h2>
          <button
            className={styles.closeBtn}
            onClick={onClose}
            disabled={isLoading || resetting}
            aria-label="Close"
          >
            ✕
          </button>
        </div>

        <div className={styles.content}>
          <p className={styles.subtitle}>{chapterTitle}</p>

          {error && <div className={styles.error}>{error}</div>}

          {/* Difficulty Level */}
          <div className={styles.section}>
            <h3 className={styles.sectionTitle}>Difficulty Level</h3>
            <p className={styles.sectionDescription}>
              How detailed should the explanations be?
            </p>
            <div className={styles.options}>
              {[
                { value: 'beginner', label: 'Beginner', desc: 'Simple terms, basic concepts' },
                { value: 'intermediate', label: 'Intermediate', desc: 'Moderate depth, some details' },
                { value: 'advanced', label: 'Advanced', desc: 'Technical terms, implementation' },
                { value: 'expert', label: 'Expert', desc: 'Research-level insights' }
              ].map(option => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="difficulty"
                    value={option.value}
                    checked={preferences.difficulty_level === option.value}
                    onChange={() => handleDifficultyChange(option.value)}
                    disabled={isLoading || resetting}
                  />
                  <span className={styles.radioText}>
                    <strong>{option.label}</strong>
                    <span className={styles.radioDesc}>{option.desc}</span>
                  </span>
                </label>
              ))}
            </div>
          </div>

          {/* Content Style */}
          <div className={styles.section}>
            <h3 className={styles.sectionTitle}>Content Style</h3>
            <p className={styles.sectionDescription}>
              How should the content be presented?
            </p>
            <div className={styles.options}>
              {[
                { value: 'text', label: 'Text-Heavy', desc: 'Detailed written explanations' },
                { value: 'visual', label: 'Visual', desc: 'More diagrams and illustrations' },
                { value: 'code', label: 'Code-Heavy', desc: 'More code examples' },
                { value: 'balanced', label: 'Balanced', desc: 'Mix of all types' }
              ].map(option => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="style"
                    value={option.value}
                    checked={preferences.content_style === option.value}
                    onChange={() => handleStyleChange(option.value)}
                    disabled={isLoading || resetting}
                  />
                  <span className={styles.radioText}>
                    <strong>{option.label}</strong>
                    <span className={styles.radioDesc}>{option.desc}</span>
                  </span>
                </label>
              ))}
            </div>
          </div>

          {/* Example Density */}
          <div className={styles.section}>
            <h3 className={styles.sectionTitle}>Example Density</h3>
            <p className={styles.sectionDescription}>
              How many examples would you like?
            </p>
            <div className={styles.options}>
              {[
                { value: 'minimal', label: 'Minimal', desc: '1-2 examples per concept' },
                { value: 'moderate', label: 'Moderate', desc: '3-5 examples per concept' },
                { value: 'rich', label: 'Rich', desc: '6+ examples per concept' }
              ].map(option => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="density"
                    value={option.value}
                    checked={preferences.example_density === option.value}
                    onChange={() => handleDensityChange(option.value)}
                    disabled={isLoading || resetting}
                  />
                  <span className={styles.radioText}>
                    <strong>{option.label}</strong>
                    <span className={styles.radioDesc}>{option.desc}</span>
                  </span>
                </label>
              ))}
            </div>
          </div>

          {/* Learning Pace */}
          <div className={styles.section}>
            <h3 className={styles.sectionTitle}>Learning Pace</h3>
            <p className={styles.sectionDescription}>
              How fast should the content progress?
            </p>
            <div className={styles.options}>
              {[
                { value: 'concise', label: 'Concise', desc: 'Quick overviews, minimal detail' },
                { value: 'standard', label: 'Standard', desc: 'Balanced pacing' },
                { value: 'detailed', label: 'Detailed', desc: 'Comprehensive explanations' }
              ].map(option => (
                <label key={option.value} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="pace"
                    value={option.value}
                    checked={preferences.learning_pace === option.value}
                    onChange={() => handlePaceChange(option.value)}
                    disabled={isLoading || resetting}
                  />
                  <span className={styles.radioText}>
                    <strong>{option.label}</strong>
                    <span className={styles.radioDesc}>{option.desc}</span>
                  </span>
                </label>
              ))}
            </div>
          </div>
        </div>

        <div className={styles.footer}>
          <button
            className={`${styles.btn} ${styles.resetBtn}`}
            onClick={handleReset}
            disabled={isLoading || resetting}
          >
            {resetting ? 'Resetting...' : 'Reset to Default'}
          </button>
          <div className={styles.actionBtns}>
            <button
              className={`${styles.btn} ${styles.cancelBtn}`}
              onClick={onClose}
              disabled={isLoading || resetting}
            >
              Cancel
            </button>
            <button
              className={`${styles.btn} ${styles.saveBtn}`}
              onClick={handleSave}
              disabled={isLoading || resetting}
            >
              {isLoading ? 'Saving...' : 'Save & Reload'}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
