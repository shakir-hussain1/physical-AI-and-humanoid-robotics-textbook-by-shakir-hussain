import React, { useState, useEffect } from 'react';
import { useTranslation } from '../hooks/useTranslation';
import styles from './TranslationButton.module.css';

/**
 * Translation button component for toggling Urdu translation
 */
export function TranslationButton({ chapterId, chapterContent, onLanguageChange }) {
  const {
    currentLanguage,
    isLoading,
    error,
    getTranslation,
    translateContent,
    getTranslationStatus,
    toggleLanguage,
    getRTLConfig,
  } = useTranslation();

  const [translationStatus, setTranslationStatus] = useState(null);
  const [rtlConfig, setRTLConfig] = useState(null);
  const [showError, setShowError] = useState(false);

  // Load translation status on mount
  useEffect(() => {
    checkTranslationStatus();
    loadRTLConfig();
  }, [chapterId]);

  const checkTranslationStatus = async () => {
    if (!chapterId) return;

    try {
      const status = await getTranslationStatus(chapterId);
      setTranslationStatus(status);
    } catch (err) {
      console.error('Status check failed:', err);
    }
  };

  const loadRTLConfig = async () => {
    try {
      const config = await getRTLConfig('ur');
      setRTLConfig(config);
    } catch (err) {
      console.error('RTL config failed:', err);
    }
  };

  const handleTranslate = async () => {
    try {
      setShowError(false);

      if (!chapterContent || chapterContent.trim().length === 0) {
        setShowError(true);
        return;
      }

      await translateContent(chapterId, chapterContent, 'chapter');
      checkTranslationStatus();
    } catch (err) {
      setShowError(true);
      console.error('Translation failed:', err);
    }
  };

  const handleLanguageToggle = async () => {
    try {
      setShowError(false);
      const success = await toggleLanguage(chapterId);

      if (success) {
        if (onLanguageChange) {
          onLanguageChange(currentLanguage === 'en' ? 'ur' : 'en');
        }
      } else {
        setShowError(true);
      }
    } catch (err) {
      setShowError(true);
      console.error('Language toggle failed:', err);
    }
  };

  const isTranslated =
    translationStatus && translationStatus.available;

  return (
    <div className={styles.container}>
      {showError && error && (
        <div className={styles.errorMessage}>
          {error}
        </div>
      )}

      <div className={styles.buttonGroup}>
        {/* Translate Button */}
        {!isTranslated && (
          <button
            className={styles.translateButton}
            onClick={handleTranslate}
            disabled={isLoading}
            title="Translate chapter to Urdu"
          >
            {isLoading ? 'Translating...' : 'اردو میں ترجمہ کریں'}
          </button>
        )}

        {/* Language Toggle Button */}
        {isTranslated && (
          <button
            className={`${styles.toggleButton} ${
              currentLanguage === 'ur' ? styles.urdu : styles.english
            }`}
            onClick={handleLanguageToggle}
            disabled={isLoading}
            title={`Switch to ${currentLanguage === 'en' ? 'Urdu' : 'English'}`}
          >
            {currentLanguage === 'en' ? (
              <>
                <span className={styles.flag}>🇵🇰</span>
                <span>اردو</span>
              </>
            ) : (
              <>
                <span className={styles.flag}>🇬🇧</span>
                <span>English</span>
              </>
            )}
          </button>
        )}
      </div>

      {/* Status Indicator */}
      {translationStatus && (
        <div className={styles.statusIndicator}>
          {translationStatus.available ? (
            <div className={styles.statusBadge + ' ' + styles.success}>
              <span className={styles.dot}></span>
              Urdu available
            </div>
          ) : (
            <div className={styles.statusBadge + ' ' + styles.pending}>
              <span className={styles.dot}></span>
              Not translated
            </div>
          )}

          {translationStatus.accuracy_score && (
            <span className={styles.accuracyScore}>
              Accuracy: {translationStatus.accuracy_score.toFixed(0)}%
            </span>
          )}
        </div>
      )}

      {/* RTL Config Info (debug) */}
      {rtlConfig && (
        <style>{`
          body[data-language="ur"] {
            direction: ${rtlConfig.direction};
            text-align: ${rtlConfig.text_align};
            font-family: ${rtlConfig.font_family};
          }
        `}</style>
      )}
    </div>
  );
}

/**
 * Language Toggle Switch Component
 */
export function LanguageToggle({ currentLanguage, onToggle, isLoading }) {
  return (
    <button
      className={`${styles.toggleSwitch} ${
        currentLanguage === 'ur' ? styles.urdu : styles.english
      }`}
      onClick={onToggle}
      disabled={isLoading}
    >
      <span className={styles.toggleLabel}>
        {currentLanguage === 'en' ? 'عربی' : 'English'}
      </span>
    </button>
  );
}

/**
 * Translation Progress Component
 */
export function TranslationProgress({
  isTranslating,
  progress = 0,
  status = '',
}) {
  if (!isTranslating && progress === 0) return null;

  return (
    <div className={styles.progressContainer}>
      <div className={styles.progressBar}>
        <div
          className={styles.progressFill}
          style={{ width: `${progress}%` }}
        ></div>
      </div>
      <p className={styles.progressText}>
        {status || 'Translating...'}
      </p>
    </div>
  );
}
