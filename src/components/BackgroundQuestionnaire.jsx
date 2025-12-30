import React, { useState } from 'react';
import styles from './AuthForms.module.css';

export function BackgroundQuestionnaire({ onSubmit, onBack, isLoading }) {
  const [backgroundData, setBackgroundData] = useState({
    softwareBackground: '',
    hardwareBackground: '',
    programmingLanguages: [],
    roboticsExperience: '',
    aiMlExperience: '',
  });

  const softwareOptions = ['beginner', 'intermediate', 'advanced', 'expert'];
  const hardwareOptions = ['beginner', 'intermediate', 'advanced', 'expert'];
  const programmingLanguageOptions = ['Python', 'JavaScript', 'C++', 'Java', 'ROS', 'Other'];
  const experienceOptions = ['none', 'some', 'intermediate', 'advanced'];

  const handleLevelChange = (field, value) => {
    setBackgroundData((prev) => ({
      ...prev,
      [field]: prev[field] === value ? '' : value,
    }));
  };

  const handleLanguageChange = (language) => {
    setBackgroundData((prev) => ({
      ...prev,
      programmingLanguages: prev.programmingLanguages.includes(language)
        ? prev.programmingLanguages.filter((l) => l !== language)
        : [...prev.programmingLanguages, language],
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    onSubmit(backgroundData);
  };

  return (
    <div className={styles.formContainer}>
      <div className={styles.formHeader}>
        <h2>Tell Us About Your Background</h2>
        <p>This helps us personalize your learning experience. Step 2 of 2</p>
      </div>

      <form onSubmit={handleSubmit} className={styles.form}>
        {/* Software Background */}
        <div className={styles.formGroup}>
          <label className={styles.label}>Software Development Background</label>
          <div className={styles.optionGrid}>
            {softwareOptions.map((option) => (
              <button
                key={option}
                type="button"
                className={`${styles.optionButton} ${
                  backgroundData.softwareBackground === option ? styles.selected : ''
                }`}
                onClick={() => handleLevelChange('softwareBackground', option)}
                disabled={isLoading}
              >
                {option.charAt(0).toUpperCase() + option.slice(1)}
              </button>
            ))}
          </div>
        </div>

        {/* Hardware Background */}
        <div className={styles.formGroup}>
          <label className={styles.label}>Hardware/Electronics Background</label>
          <div className={styles.optionGrid}>
            {hardwareOptions.map((option) => (
              <button
                key={option}
                type="button"
                className={`${styles.optionButton} ${
                  backgroundData.hardwareBackground === option ? styles.selected : ''
                }`}
                onClick={() => handleLevelChange('hardwareBackground', option)}
                disabled={isLoading}
              >
                {option.charAt(0).toUpperCase() + option.slice(1)}
              </button>
            ))}
          </div>
        </div>

        {/* Programming Languages */}
        <div className={styles.formGroup}>
          <label className={styles.label}>Programming Languages You Know</label>
          <div className={styles.checkboxGrid}>
            {programmingLanguageOptions.map((language) => (
              <label key={language} className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={backgroundData.programmingLanguages.includes(language)}
                  onChange={() => handleLanguageChange(language)}
                  disabled={isLoading}
                  className={styles.checkbox}
                />
                <span>{language}</span>
              </label>
            ))}
          </div>
        </div>

        {/* Robotics Experience */}
        <div className={styles.formGroup}>
          <label className={styles.label}>Robotics Experience</label>
          <div className={styles.optionGrid}>
            {experienceOptions.map((option) => (
              <button
                key={option}
                type="button"
                className={`${styles.optionButton} ${
                  backgroundData.roboticsExperience === option ? styles.selected : ''
                }`}
                onClick={() => handleLevelChange('roboticsExperience', option)}
                disabled={isLoading}
              >
                {option.charAt(0).toUpperCase() + option.slice(1)}
              </button>
            ))}
          </div>
        </div>

        {/* AI/ML Experience */}
        <div className={styles.formGroup}>
          <label className={styles.label}>AI/Machine Learning Experience</label>
          <div className={styles.optionGrid}>
            {experienceOptions.map((option) => (
              <button
                key={option}
                type="button"
                className={`${styles.optionButton} ${
                  backgroundData.aiMlExperience === option ? styles.selected : ''
                }`}
                onClick={() => handleLevelChange('aiMlExperience', option)}
                disabled={isLoading}
              >
                {option.charAt(0).toUpperCase() + option.slice(1)}
              </button>
            ))}
          </div>
        </div>

        <div className={styles.formActions}>
          <button
            type="button"
            className={styles.secondaryButton}
            onClick={onBack}
            disabled={isLoading}
          >
            Back
          </button>
          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? 'Creating Account...' : 'Complete Sign Up'}
          </button>
        </div>
      </form>

      <div className={styles.infoBox}>
        <p className={styles.infoText}>
          This information helps us customize the content difficulty and focus areas based on your experience level.
        </p>
      </div>
    </div>
  );
}
