/**
 * Chapter Translation Button Component
 * Allows authenticated users to translate chapter content to different languages
 * Features:
 * - Toggle translation on/off
 * - Loading state indicator
 * - Error handling with fallback
 * - Accessibility features (ARIA labels)
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../context/AuthContext';
import { translateChapterContent, getTranslatedContent } from '../services/translationApi';
import styles from './ChapterTranslateButton.module.css';

export default function ChapterTranslateButton({ chapterId, chapterTitle }) {
  const { isAuthenticated, tokens } = useAuth();
  const [isTranslated, setIsTranslated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [targetLanguage, setTargetLanguage] = useState('urdu');

  /**
   * Check if content is already cached when component mounts
   */
  useEffect(() => {
    if (isAuthenticated && chapterId) {
      // Check if we have cached translation
      const cacheKey = `translation_${chapterId}_${targetLanguage}`;
      const cached = localStorage.getItem(cacheKey);
      if (cached) {
        setTranslatedContent(JSON.parse(cached));
      }
    }
  }, [isAuthenticated, chapterId, targetLanguage]);

  /**
   * Translate the current chapter content
   * - Extracts content from DOM
   * - Calls translation API
   * - Caches result
   * - Updates display
   */
  /**
   * Get content element from DOM using multiple selectors
   * Docusaurus uses different class names in different versions
   */
  const getContentElement = () => {
    // Try multiple selectors in order of preference
    const selectors = [
      '.docItemContent',           // Docusaurus default
      '.theme-doc-markdown',       // Some versions use this
      'article',                   // Fallback to article tag
      '[role="main"]',             // Semantic HTML
      '.markdown',                 // Another variant
      '.doc-content'               // Another variant
    ];

    for (const selector of selectors) {
      const element = document.querySelector(selector);
      if (element && element.textContent.trim().length > 50) {
        console.log('[Translation] Found content with selector:', selector);
        return element;
      }
    }

    return null;
  };

  const handleTranslate = async () => {
    if (!isAuthenticated || !tokens.access_token) {
      setError('Please log in to use translation feature');
      return;
    }

    try {
      setIsLoading(true);
      setError(null);

      // Extract current chapter content from DOM
      const contentElement = getContentElement();
      if (!contentElement) {
        console.error('[Translation] Content element selectors tried:', [
          '.docItemContent', '.theme-doc-markdown', 'article', '[role="main"]'
        ]);
        throw new Error('Chapter content not found. Please check browser console.');
      }

      const originalContent = contentElement.innerHTML;
      console.log('[Translation] Extracted content length:', originalContent.length);

      // Call translation API
      const response = await translateChapterContent(
        {
          content: originalContent,
          chapter_id: chapterId,
          target_language: targetLanguage
        },
        tokens.access_token
      );

      if (!response.translated_content) {
        throw new Error('Translation failed: No content returned');
      }

      // Cache the translation
      const cacheKey = `translation_${chapterId}_${targetLanguage}`;
      localStorage.setItem(cacheKey, JSON.stringify({
        content: response.translated_content,
        timestamp: new Date().toISOString(),
        source: 'translation_api'
      }));

      setTranslatedContent(response.translated_content);
      setIsTranslated(true);

      console.log('[Translation] Successfully translated chapter to', targetLanguage);
    } catch (err) {
      console.error('[Translation Error]', err);
      setError(err.message || 'Failed to translate chapter. Please try again.');
      setIsTranslated(false);
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Toggle between original and translated content
   * Updates DOM with appropriate content
   */
  const handleToggleTranslation = async () => {
    try {
      setError(null);

      const contentElement = getContentElement();
      if (!contentElement) {
        throw new Error('Chapter content not found');
      }

      if (isTranslated && translatedContent) {
        // Switch to translated version if we have it cached
        console.log('[Translation] Switching to translated version');
        contentElement.innerHTML = translatedContent;
        setIsTranslated(true);
      } else {
        // Translate if not already done
        console.log('[Translation] Starting translation from toggle');
        await handleTranslate();
      }
    } catch (err) {
      console.error('[Translation Toggle Error]', err);
      setError(err.message || 'Failed to toggle translation');
    }
  };

  /**
   * Restore original content
   */
  const handleRestore = () => {
    try {
      // Reload the page to restore original content
      window.location.reload();
    } catch (err) {
      console.error('[Translation Error] Failed to restore', err);
      setError('Failed to restore original content');
    }
  };

  // Don't render if not authenticated or on non-chapter page
  if (!isAuthenticated) {
    return null;
  }

  return (
    <>
      <div className={styles.translationContainer}>
        <div className={styles.buttonGroup}>
          {/* Main translate button */}
          <button
            className={`${styles.translateBtn} ${
              isTranslated ? styles.translated : ''
            }`}
            onClick={handleToggleTranslation}
            disabled={isLoading}
            aria-label={
              isTranslated
                ? `Switch to original language (currently showing ${targetLanguage})`
                : `Translate this chapter to ${targetLanguage}`
            }
            title={
              isTranslated
                ? `Showing ${targetLanguage} version - Click to view original`
                : `Translate to ${targetLanguage}`
            }
          >
            <span className={styles.icon}>🌐</span>
            <span className={styles.text}>
              {isLoading ? (
                <>
                  <span className={styles.spinner}></span>
                  Translating...
                </>
              ) : isTranslated ? (
                `Back to English`
              ) : (
                `Translate to Urdu`
              )}
            </span>
          </button>

          {/* Restore button - shows when translated */}
          {isTranslated && (
            <button
              className={`${styles.btn} ${styles.restoreBtn}`}
              onClick={handleRestore}
              disabled={isLoading}
              aria-label="Restore original chapter content"
              title="Restore original content"
            >
              Reset
            </button>
          )}
        </div>

        {/* Error message display */}
        {error && (
          <div
            className={styles.errorMessage}
            role="alert"
            aria-live="polite"
          >
            ⚠️ {error}
          </div>
        )}

        {/* Translation status indicator */}
        {isTranslated && !isLoading && (
          <div
            className={styles.statusMessage}
            role="status"
            aria-live="polite"
          >
            ✓ Chapter translated to {targetLanguage}
          </div>
        )}
      </div>
    </>
  );
}
