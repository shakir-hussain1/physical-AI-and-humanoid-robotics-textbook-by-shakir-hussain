import React, { useState, useEffect } from 'react';
import { useAuth } from '../context/AuthContext';
import { getChapterPersonalization, saveChapterPersonalization } from '../services/personalizationApi';
import PersonalizationModal from './PersonalizationModal';
import styles from './ChapterPersonalizeButton.module.css';

export default function ChapterPersonalizeButton({ chapterId, chapterTitle }) {
  const { isAuthenticated, tokens } = useAuth();
  const [showModal, setShowModal] = useState(false);
  const [currentPreferences, setCurrentPreferences] = useState(null);
  const [loading, setLoading] = useState(false);
  const [hasPreferences, setHasPreferences] = useState(false);

  // Load current preferences on mount
  useEffect(() => {
    if (isAuthenticated && tokens.access_token && chapterId) {
      loadPreferences();
    }
  }, [isAuthenticated, tokens.access_token, chapterId]);

  const loadPreferences = async () => {
    try {
      setLoading(true);
      const prefs = await getChapterPersonalization(chapterId, tokens.access_token);
      if (prefs) {
        setCurrentPreferences(prefs);
        setHasPreferences(true);
      }
    } catch (err) {
      console.error('Failed to load preferences:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleSavePreferences = async (newPreferences) => {
    try {
      setLoading(true);
      console.log('[Button] Saving preferences for chapter:', chapterId);
      console.log('[Button] Settings:', newPreferences);
      await saveChapterPersonalization(chapterId, newPreferences, tokens.access_token);
      console.log('[Button] Save successful, reloading page...');
      setCurrentPreferences(newPreferences);
      setHasPreferences(true);
      setShowModal(false);
      // Reload page to apply new styles
      window.location.reload();
    } catch (err) {
      console.error('[Button] Failed to save preferences:', err);
      console.error('[Button] Error details:', {
        message: err.message,
        chapterId: chapterId,
        token: tokens.access_token ? 'present' : 'missing'
      });
      // Re-throw the error so modal can handle it
      throw err;
    } finally {
      setLoading(false);
    }
  };

  // Don't render if not authenticated
  if (!isAuthenticated) {
    return null;
  }

  return (
    <>
      <div className={styles.buttonContainer}>
        <button
          className={`${styles.personalizeBtn} ${hasPreferences ? styles.hasPreferences : ''}`}
          onClick={() => setShowModal(true)}
          disabled={loading}
          title={hasPreferences ? 'Personalization applied' : 'Customize this chapter'}
        >
          <span className={styles.icon}>⚙️</span>
          <span className={styles.text}>
            {loading ? 'Loading...' : hasPreferences ? 'Personalized' : 'Personalize'}
          </span>
          {hasPreferences && <span className={styles.badge}>✓</span>}
        </button>
      </div>

      {showModal && (
        <PersonalizationModal
          chapterId={chapterId}
          chapterTitle={chapterTitle}
          currentPreferences={currentPreferences}
          onSave={handleSavePreferences}
          onClose={() => setShowModal(false)}
          isLoading={loading}
        />
      )}
    </>
  );
}
