/**
 * Chapter Personalization API Service
 * Handles all API calls for chapter personalization preferences
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

/**
 * Save chapter personalization settings
 * @param {string} chapterId - Unique chapter identifier
 * @param {object} settings - Personalization settings
 * @param {string} token - JWT access token
 * @returns {Promise<object>} Saved personalization object
 */
export async function saveChapterPersonalization(chapterId, settings, token) {
  if (!token) {
    throw new Error('No authentication token provided');
  }

  if (!chapterId) {
    throw new Error('Chapter ID is required');
  }

  try {
    const response = await fetch(
      `${API_URL}/personalization/chapter/${encodeURIComponent(chapterId)}`,
      {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify(settings)
      }
    );

    if (response.status === 401) {
      throw new Error('Authentication failed. Please log in again.');
    }

    if (!response.ok) {
      let errorMessage = `Failed to save personalization (${response.status})`;
      try {
        const errorData = await response.json();
        errorMessage = errorData.detail || errorData.message || errorMessage;
      } catch (e) {
        // Could not parse error response
      }
      console.error('[API Error] Save personalization failed:', {
        status: response.status,
        statusText: response.statusText,
        message: errorMessage,
        chapterId: chapterId,
        apiUrl: API_URL
      });
      throw new Error(errorMessage);
    }

    const data = await response.json();
    console.log('[API Success] Personalization saved for chapter:', chapterId);
    return data.data || data;
  } catch (error) {
    console.error('[API Exception] Error saving personalization:', error);
    throw error;
  }
}

/**
 * Get chapter personalization settings
 * @param {string} chapterId - Unique chapter identifier
 * @param {string} token - JWT access token
 * @returns {Promise<object|null>} Personalization object or null if not found
 */
export async function getChapterPersonalization(chapterId, token) {
  if (!token) {
    throw new Error('No authentication token provided');
  }

  if (!chapterId) {
    throw new Error('Chapter ID is required');
  }

  try {
    const response = await fetch(
      `${API_URL}/personalization/chapter/${encodeURIComponent(chapterId)}`,
      {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`
        }
      }
    );

    if (response.status === 401) {
      throw new Error('Authentication failed. Please log in again.');
    }

    if (response.status === 404) {
      // No personalization found - return null
      return null;
    }

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(
        errorData.detail || `Failed to fetch personalization (${response.status})`
      );
    }

    const data = await response.json();
    return data.data || null;
  } catch (error) {
    console.error('Error fetching personalization:', error);
    throw error;
  }
}

/**
 * Get all personalized chapters for current user
 * @param {string} token - JWT access token
 * @returns {Promise<array>} Array of personalization objects
 */
export async function getAllChapterPersonalizations(token) {
  if (!token) {
    throw new Error('No authentication token provided');
  }

  try {
    const response = await fetch(
      `${API_URL}/personalization/chapters`,
      {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`
        }
      }
    );

    if (response.status === 401) {
      throw new Error('Authentication failed. Please log in again.');
    }

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(
        errorData.detail || `Failed to fetch personalizations (${response.status})`
      );
    }

    const data = await response.json();
    return data.data || [];
  } catch (error) {
    console.error('Error fetching personalizations:', error);
    throw error;
  }
}

/**
 * Reset chapter personalization to default
 * @param {string} chapterId - Unique chapter identifier
 * @param {string} token - JWT access token
 * @returns {Promise<object>} Success response
 */
export async function resetChapterPersonalization(chapterId, token) {
  if (!token) {
    throw new Error('No authentication token provided');
  }

  if (!chapterId) {
    throw new Error('Chapter ID is required');
  }

  try {
    const response = await fetch(
      `${API_URL}/personalization/chapter/${encodeURIComponent(chapterId)}`,
      {
        method: 'DELETE',
        headers: {
          'Authorization': `Bearer ${token}`
        }
      }
    );

    if (response.status === 401) {
      throw new Error('Authentication failed. Please log in again.');
    }

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(
        errorData.detail || `Failed to reset personalization (${response.status})`
      );
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error resetting personalization:', error);
    throw error;
  }
}

/**
 * Get API URL (exposed for testing)
 * @returns {string} Current API URL
 */
export function getAPIUrlForTesting() {
  return API_URL;
}
