import { useState, useCallback, useEffect } from 'react';
import { useAuth } from './useAuth';

export function useTranslation() {
  const { getAuthHeader } = useAuth();
  const [translations, setTranslations] = useState({});
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [currentLanguage, setCurrentLanguage] = useState('en');

  const API_BASE_URL = 'http://localhost:8001/api';

  // Get RTL configuration
  const getRTLConfig = useCallback(
    async (language) => {
      try {
        const response = await fetch(
          `${API_BASE_URL}/translation/rtl-config?language=${language}`,
          {
            headers: getAuthHeader(),
          }
        );

        if (!response.ok) {
          throw new Error('Failed to get RTL config');
        }

        return await response.json();
      } catch (err) {
        console.error('RTL config error:', err);
        return {
          is_rtl: false,
          text_align: 'left',
          direction: 'ltr',
          font_family: 'system-ui',
        };
      }
    },
    [API_BASE_URL, getAuthHeader]
  );

  // Translate content
  const translateContent = useCallback(
    async (contentId, contentText, contentType = 'section') => {
      try {
        setIsLoading(true);
        setError(null);

        const response = await fetch(`${API_BASE_URL}/translation/translate`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            ...getAuthHeader(),
          },
          body: JSON.stringify({
            content_id: contentId,
            content_text: contentText,
            content_type: contentType,
            target_language: 'ur',
          }),
        });

        if (!response.ok) {
          const errData = await response.json();
          throw new Error(errData.detail || 'Translation failed');
        }

        const data = await response.json();

        // Store in cache
        setTranslations((prev) => ({
          ...prev,
          [contentId]: data,
        }));

        return data;
      } catch (err) {
        setError(err.message);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [API_BASE_URL, getAuthHeader]
  );

  // Get cached translation
  const getTranslation = useCallback(
    async (contentId) => {
      try {
        // Check cache first
        if (translations[contentId]) {
          return translations[contentId];
        }

        setIsLoading(true);
        setError(null);

        const response = await fetch(
          `${API_BASE_URL}/translation/content/${contentId}`,
          {
            headers: getAuthHeader(),
          }
        );

        if (!response.ok) {
          if (response.status === 404) {
            return null;
          }
          throw new Error('Failed to get translation');
        }

        const data = await response.json();

        // Store in cache
        setTranslations((prev) => ({
          ...prev,
          [contentId]: data,
        }));

        return data;
      } catch (err) {
        setError(err.message);
        return null;
      } finally {
        setIsLoading(false);
      }
    },
    [translations, API_BASE_URL, getAuthHeader]
  );

  // Get translation status
  const getTranslationStatus = useCallback(
    async (contentId) => {
      try {
        const response = await fetch(
          `${API_BASE_URL}/translation/status/${contentId}`,
          {
            headers: getAuthHeader(),
          }
        );

        if (!response.ok) {
          return {
            available: false,
            status: 'not_translated',
          };
        }

        return await response.json();
      } catch (err) {
        console.error('Status check error:', err);
        return {
          available: false,
          status: 'error',
        };
      }
    },
    [API_BASE_URL, getAuthHeader]
  );

  // Toggle language
  const toggleLanguage = useCallback(async (contentId) => {
    try {
      if (currentLanguage === 'en') {
        // Switch to Urdu
        const translation = await getTranslation(contentId);
        if (!translation) {
          setError('Translation not available. Please translate first.');
          return false;
        }
        setCurrentLanguage('ur');
        return true;
      } else {
        // Switch to English
        setCurrentLanguage('en');
        return true;
      }
    } catch (err) {
      setError(err.message);
      return false;
    }
  }, [currentLanguage, getTranslation]);

  // Batch translate chapters
  const batchTranslate = useCallback(
    async (chapters) => {
      try {
        setIsLoading(true);
        setError(null);

        const response = await fetch(`${API_BASE_URL}/translation/batch`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            ...getAuthHeader(),
          },
          body: JSON.stringify({
            chapters: chapters,
            target_language: 'ur',
          }),
        });

        if (!response.ok) {
          const errData = await response.json();
          throw new Error(errData.detail || 'Batch translation failed');
        }

        const data = await response.json();

        // Store all translations in cache
        const newTranslations = {};
        data.forEach((t) => {
          newTranslations[t.content_id] = t;
        });

        setTranslations((prev) => ({
          ...prev,
          ...newTranslations,
        }));

        return data;
      } catch (err) {
        setError(err.message);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [API_BASE_URL, getAuthHeader]
  );

  // Clear cache
  const clearCache = useCallback(
    async (contentId = null) => {
      try {
        const endpoint = contentId
          ? `${API_BASE_URL}/translation/cache/${contentId}`
          : `${API_BASE_URL}/translation/cache`;

        const response = await fetch(endpoint, {
          method: 'DELETE',
          headers: getAuthHeader(),
        });

        if (!response.ok) {
          throw new Error('Failed to clear cache');
        }

        if (contentId) {
          setTranslations((prev) => {
            const newTrans = { ...prev };
            delete newTrans[contentId];
            return newTrans;
          });
        } else {
          setTranslations({});
        }

        return true;
      } catch (err) {
        setError(err.message);
        return false;
      }
    },
    [API_BASE_URL, getAuthHeader]
  );

  return {
    translations,
    isLoading,
    error,
    currentLanguage,
    translateContent,
    getTranslation,
    getTranslationStatus,
    toggleLanguage,
    batchTranslate,
    clearCache,
    getRTLConfig,
  };
}
