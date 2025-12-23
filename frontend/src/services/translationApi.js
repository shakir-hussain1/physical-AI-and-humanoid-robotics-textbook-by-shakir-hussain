/**
 * Translation API Service
 * Handles all translation-related API calls and local caching
 *
 * Features:
 * - Smart caching with localStorage
 * - Fallback to original content on error
 * - Comprehensive error logging
 * - Support for multiple languages (extensible)
 */

const getAPIUrl = () => {
  try {
    if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL) {
      return process.env.REACT_APP_API_URL;
    }
  } catch (e) {
    // process not available in browser
  }

  if (typeof window !== 'undefined' && window.location.hostname === 'localhost') {
    return 'http://localhost:8000';
  }

  // Production - use Hugging Face backend
  return 'https://shakir-rag-chatbot-backend.hf.space';
};

const API_URL = getAPIUrl();

// Cache configuration
const CACHE_CONFIG = {
  EXPIRY_HOURS: 24,
  PREFIX: 'translation_',
};

/**
 * Calculate cache key for storing translated content
 * Format: translation_{chapterId}_{language}
 *
 * @param {string} chapterId - Chapter identifier
 * @param {string} language - Target language code
 * @returns {string} Cache key
 */
const getCacheKey = (chapterId, language) => {
  return `${CACHE_CONFIG.PREFIX}${chapterId}_${language}`;
};

/**
 * Check if cached translation is still valid
 *
 * @param {object} cachedData - Cached translation object
 * @returns {boolean} True if cache is still valid
 */
const isCacheValid = (cachedData) => {
  if (!cachedData || !cachedData.timestamp) {
    return false;
  }

  const cacheTime = new Date(cachedData.timestamp).getTime();
  const now = new Date().getTime();
  const expiryTime = CACHE_CONFIG.EXPIRY_HOURS * 60 * 60 * 1000;

  return now - cacheTime < expiryTime;
};

/**
 * Get cached translation if available and valid
 *
 * @param {string} chapterId - Chapter identifier
 * @param {string} language - Target language
 * @returns {object|null} Cached content or null if not available/expired
 */
const getCachedTranslation = (chapterId, language) => {
  try {
    const cacheKey = getCacheKey(chapterId, language);
    const cached = localStorage.getItem(cacheKey);

    if (cached) {
      const parsedCache = JSON.parse(cached);
      if (isCacheValid(parsedCache)) {
        console.log('[Translation Cache] Cache hit for', chapterId, language);
        return parsedCache.content;
      } else {
        // Cache expired, remove it
        localStorage.removeItem(cacheKey);
        console.log('[Translation Cache] Cache expired for', chapterId, language);
      }
    }
  } catch (error) {
    console.error('[Translation Cache Error]', error);
  }

  return null;
};

/**
 * Save translation to local cache
 *
 * @param {string} chapterId - Chapter identifier
 * @param {string} language - Target language
 * @param {string} content - Translated content
 */
const cacheTranslation = (chapterId, language, content) => {
  try {
    const cacheKey = getCacheKey(chapterId, language);
    const cacheData = {
      content: content,
      timestamp: new Date().toISOString(),
      source: 'translation_api',
    };

    localStorage.setItem(cacheKey, JSON.stringify(cacheData));
    console.log('[Translation Cache] Cached translation for', chapterId, language);
  } catch (error) {
    console.error('[Translation Cache Error]', error);
    // Don't throw - caching is optional
  }
};

/**
 * Clear all cached translations for a chapter
 * Useful for refreshing or testing
 *
 * @param {string} chapterId - Chapter identifier
 */
export const clearTranslationCache = (chapterId) => {
  try {
    const keys = Object.keys(localStorage);
    keys.forEach(key => {
      if (key.includes(`${CACHE_CONFIG.PREFIX}${chapterId}`)) {
        localStorage.removeItem(key);
      }
    });
    console.log('[Translation Cache] Cleared cache for', chapterId);
  } catch (error) {
    console.error('[Translation Cache Error]', error);
  }
};

/**
 * Translate chapter content to target language
 *
 * Process:
 * 1. Check local cache first
 * 2. If not cached, call backend API
 * 3. Cache result for future use
 * 4. Return translated content
 *
 * @param {object} data - Translation request data
 * @param {string} data.content - HTML content to translate
 * @param {string} data.chapter_id - Chapter identifier
 * @param {string} data.target_language - Target language code (e.g., 'urdu', 'spanish')
 * @param {string} token - JWT authentication token
 * @returns {Promise<object>} Translation response with translated_content
 * @throws {Error} On network or translation errors
 */
export async function translateChapterContent(data, token) {
  if (!token) {
    throw new Error('No authentication token provided');
  }

  if (!data.content) {
    throw new Error('Content is required for translation');
  }

  if (!data.chapter_id) {
    throw new Error('Chapter ID is required');
  }

  if (!data.target_language) {
    throw new Error('Target language is required');
  }

  try {
    console.log('[Translation API] Translating to', data.target_language);

    // Check cache first
    const cachedContent = getCachedTranslation(data.chapter_id, data.target_language);
    if (cachedContent) {
      console.log('[Translation API] Using cached translation');
      return {
        translated_content: cachedContent,
        from_cache: true,
        source: 'local_cache',
      };
    }

    // Call backend API
    const response = await fetch(`${API_URL}/translation/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
      },
      body: JSON.stringify(data),
    });

    if (response.status === 401) {
      throw new Error('Authentication failed. Please log in again.');
    }

    if (!response.ok) {
      let errorMessage = `Translation failed (${response.status})`;
      try {
        const errorData = await response.json();
        errorMessage = errorData.detail || errorData.message || errorMessage;
      } catch (e) {
        // Could not parse error response
      }

      console.error('[Translation API Error]', {
        status: response.status,
        statusText: response.statusText,
        message: errorMessage,
        apiUrl: API_URL,
      });

      throw new Error(errorMessage);
    }

    const result = await response.json();

    if (!result.translated_content) {
      throw new Error('Invalid response from translation service');
    }

    // Cache the translation
    cacheTranslation(data.chapter_id, data.target_language, result.translated_content);

    console.log('[Translation API] Successfully translated content');
    return {
      translated_content: result.translated_content,
      from_cache: false,
      source: 'api',
      confidence: result.confidence_score || 'unknown',
    };
  } catch (error) {
    console.error('[Translation API Exception]', error);
    throw error;
  }
}

/**
 * Get information about supported languages
 * Used for language selection UI
 *
 * @param {string} token - JWT authentication token
 * @returns {Promise<array>} List of supported languages
 */
export async function getSupportedLanguages(token) {
  if (!token) {
    throw new Error('No authentication token provided');
  }

  try {
    const response = await fetch(`${API_URL}/translation/languages`, {
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });

    if (!response.ok) {
      throw new Error(`Failed to fetch languages (${response.status})`);
    }

    const data = await response.json();
    return data.languages || [];
  } catch (error) {
    console.error('[Translation API] Error fetching languages:', error);
    // Return default languages
    return [
      { code: 'urdu', name: 'Urdu', native_name: 'اردو' },
      { code: 'spanish', name: 'Spanish', native_name: 'Español' },
      { code: 'french', name: 'French', native_name: 'Français' },
    ];
  }
}

/**
 * Get translation statistics for a chapter
 * Used for analytics and debugging
 *
 * @param {string} chapterId - Chapter identifier
 * @param {string} token - JWT authentication token
 * @returns {Promise<object>} Translation statistics
 */
export async function getTranslationStats(chapterId, token) {
  if (!token) {
    throw new Error('No authentication token provided');
  }

  try {
    const response = await fetch(`${API_URL}/translation/stats/${chapterId}`, {
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });

    if (!response.ok) {
      throw new Error(`Failed to fetch stats (${response.status})`);
    }

    return await response.json();
  } catch (error) {
    console.error('[Translation API] Error fetching stats:', error);
    return null;
  }
}

/**
 * Clear a specific translation cache entry
 * Useful for manual cache invalidation
 *
 * @param {string} chapterId - Chapter identifier
 * @param {string} language - Language code
 */
export function clearCacheEntry(chapterId, language) {
  try {
    const cacheKey = getCacheKey(chapterId, language);
    localStorage.removeItem(cacheKey);
    console.log('[Translation Cache] Cleared entry for', chapterId, language);
  } catch (error) {
    console.error('[Translation Cache Error]', error);
  }
}

/**
 * Get API URL for testing
 * Exposed for testing purposes
 *
 * @returns {string} Current API URL
 */
export function getAPIUrlForTesting() {
  return API_URL;
}
