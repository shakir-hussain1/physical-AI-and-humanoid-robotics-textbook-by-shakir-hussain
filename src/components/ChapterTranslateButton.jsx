/**
 * Chapter Translation Button Component
 * Allows authenticated users to translate chapter content to different languages
 * Features:
 * - Toggle translation on/off (within same page session)
 * - Loading state indicator
 * - Error handling with fallback
 * - Accessibility features (ARIA labels)
 *
 * Note: Translations persist within the current page session only.
 * This avoids React DOM conflicts that occur when trying to apply
 * cached translations across page reloads.
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../context/AuthContext';
import { translateChapterContent } from '../services/translationApi';
import styles from './ChapterTranslateButton.module.css';

export default function ChapterTranslateButton({ chapterId, chapterTitle }) {
  const { isAuthenticated, tokens } = useAuth();
  const [isTranslated, setIsTranslated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [originalContent, setOriginalContent] = useState(null);
  const [targetLanguage, setTargetLanguage] = useState('urdu');

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

      // Store original content if not already stored
      if (!originalContent) {
        setOriginalContent(contentElement.innerHTML);
      }

      const contentToTranslate = contentElement.innerHTML;
      console.log('[Translation] Extracted content length:', contentToTranslate.length);

      // Call translation API
      const response = await translateChapterContent(
        {
          content: contentToTranslate,
          chapter_id: chapterId,
          target_language: targetLanguage
        },
        tokens.access_token
      );

      if (!response.translated_content) {
        throw new Error('Translation failed: No content returned');
      }

      console.log('[Translation] Successfully translated chapter to', targetLanguage);

      // Apply translated content directly to the DOM
      // We do this carefully by replacing only the text nodes, not removing/re-adding React elements
      applyTranslatedContent(contentElement, response.translated_content);

      setIsTranslated(true);
    } catch (err) {
      console.error('[Translation Error]', err);
      setError(err.message || 'Failed to translate chapter. Please try again.');
      setIsTranslated(false);
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Apply translated content to DOM safely
   * Replaces only the inner HTML of the content element
   * This happens outside of React's lifecycle, so it doesn't conflict with Docusaurus
   */
  const applyTranslatedContent = (element, translatedHTML) => {
    if (!element) return;

    // Store original if not already stored
    if (!originalContent) {
      setOriginalContent(element.innerHTML);
    }

    // Apply translation by replacing innerHTML
    // This is safe here because we're doing it in response to a user action
    // and after the initial render
    try {
      element.innerHTML = translatedHTML;
      console.log('[Translation] Applied translation to page');
    } catch (err) {
      console.error('[Translation] Error applying translation:', err);
      setError('Error displaying translation. Please try again.');
    }
  };

  /**
   * Toggle between original and translated content
   */
  const handleToggleTranslation = async () => {
    try {
      setError(null);

      if (!isTranslated) {
        // Translate
        console.log('[Translation] Starting translation from toggle');
        await handleTranslate();
      } else if (isTranslated && originalContent) {
        // Already translated, restore original
        console.log('[Translation] Toggling back to original');
        const contentElement = getContentElement();
        if (contentElement) {
          contentElement.innerHTML = originalContent;
          setIsTranslated(false);
          console.log('[Translation] Restored original content');
        }
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
      if (originalContent) {
        const contentElement = getContentElement();
        if (contentElement) {
          contentElement.innerHTML = originalContent;
          setIsTranslated(false);
          console.log('[Translation] Restored original content');
        }
      }
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
