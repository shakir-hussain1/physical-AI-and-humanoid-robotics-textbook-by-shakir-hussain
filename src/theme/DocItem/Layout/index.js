import React, { useEffect, useState } from 'react';
import DocItemLayout from '@theme-original/DocItem/Layout';
import ChapterPersonalizeButton from '../../../components/ChapterPersonalizeButton';
import { useAuth } from '../../../context/AuthContext';
import { getChapterPersonalization } from '../../../services/personalizationApi';
import styles from './Layout.module.css';

// Try to safely use useDoc if available
function useDocSafe() {
  try {
    // eslint-disable-next-line react-hooks/rules-of-hooks
    const { useDoc: useDocHook } = require('@docusaurus/theme-common/internal');
    // eslint-disable-next-line react-hooks/rules-of-hooks
    return useDocHook();
  } catch (err) {
    return null;
  }
}

export default function DocItemLayoutWrapper(props) {
  const { isAuthenticated, tokens } = useAuth();
  const [chapterId, setChapterId] = useState(null);
  const [chapterTitle, setChapterTitle] = useState(null);

  // Try to get doc context, but don't fail if it's not available
  let docContext = null;
  try {
    // Using a more direct approach - check if we can access doc metadata from props
    if (props.children && typeof props.children === 'function') {
      // This is a render function, we might not have direct access
      docContext = null;
    }
  } catch (err) {
    // Silently fail
  }

  // Extract metadata from URL and DOM as fallback
  useEffect(() => {
    const updateChapterInfo = () => {
      const pathname = window.location.pathname;
      const pathParts = pathname.split('/').filter(p => p);

      // Try to find chapter identifier in path
      let foundChapterId = null;
      for (let i = pathParts.length - 1; i >= 0; i--) {
        if (pathParts[i].includes('chapter-') || pathParts[i].includes('chapter')) {
          foundChapterId = pathParts.slice(i).join('/');
          break;
        }
      }

      if (!foundChapterId && pathParts.length > 0) {
        foundChapterId = pathParts[pathParts.length - 1];
      }

      setChapterId(foundChapterId);

      // Get title from page
      const titleElement = document.querySelector('h1');
      const pageTitle = titleElement ? titleElement.textContent : document.title;
      setChapterTitle(pageTitle);
    };

    // Initial update
    updateChapterInfo();

    // Also listen for navigation changes
    window.addEventListener('popstate', updateChapterInfo);
    return () => window.removeEventListener('popstate', updateChapterInfo);
  }, []);

  // Load personalization preferences
  useEffect(() => {
    if (isAuthenticated && chapterId && tokens.access_token && chapterId.includes('chapter-')) {
      loadPersonalization();
    }
  }, [isAuthenticated, chapterId, tokens.access_token]);

  const loadPersonalization = async () => {
    try {
      const prefs = await getChapterPersonalization(chapterId, tokens.access_token);
      if (prefs) {
        applyPersonalizationStyles(prefs);
      }
    } catch (err) {
      console.error('Failed to load personalization:', err);
    }
  };

  const applyPersonalizationStyles = (settings) => {
    // Find the main content container - wait a bit for DOM to be ready
    const applyStyles = () => {
      const contentElement = document.querySelector('.docItemContent');
      if (contentElement) {
        // Remove previous personalization classes
        let className = contentElement.className
          .replace(/personalized-\w+/g, '')
          .replace(/content-style-\w+/g, '')
          .replace(/example-density-\w+/g, '')
          .replace(/pace-\w+/g, '')
          .replace(/\s+/g, ' ')
          .trim();

        contentElement.className = className;

        // Add new personalization classes
        contentElement.classList.add(
          `personalized-${settings.difficulty_level}`,
          `content-style-${settings.content_style}`,
          `example-density-${settings.example_density}`,
          `pace-${settings.learning_pace}`
        );
      }
    };

    // Try immediately, then with a delay
    applyStyles();
    setTimeout(applyStyles, 100);
    setTimeout(applyStyles, 300);
  };

  // Check if this is a chapter page
  const isChapter = chapterId && chapterId.includes('chapter-');

  return (
    <>
      {isChapter && isAuthenticated && chapterId && (
        <div className={styles.buttonContainer}>
          <ChapterPersonalizeButton
            chapterId={chapterId}
            chapterTitle={chapterTitle || 'Chapter'}
          />
        </div>
      )}

      {/* Original layout content */}
      <DocItemLayout {...props} />
    </>
  );
}
