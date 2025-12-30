import { useState, useCallback, useEffect } from 'react';
import { useAuth } from './useAuth';

export function usePersonalization() {
  const { getAuthHeader } = useAuth();
  const [preferences, setPreferences] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const API_BASE_URL = 'http://localhost:8001/api';

  // Fetch all preferences
  const fetchAllPreferences = useCallback(async () => {
    try {
      setIsLoading(true);
      setError(null);

      const response = await fetch(`${API_BASE_URL}/personalization/all`, {
        headers: getAuthHeader(),
      });

      if (!response.ok) {
        throw new Error('Failed to fetch preferences');
      }

      const data = await response.json();
      setPreferences({
        global: data.global_preference,
        modules: data.module_preferences || {},
        chapters: data.chapter_preferences || {},
      });

      return data;
    } catch (err) {
      setError(err.message);
      throw err;
    } finally {
      setIsLoading(false);
    }
  }, [API_BASE_URL, getAuthHeader]);

  // Get effective preference for a chapter/module
  const getEffectivePreference = useCallback(
    async (chapterId, moduleId = null) => {
      try {
        setError(null);

        const params = new URLSearchParams();
        params.append('chapter_id', chapterId);
        if (moduleId) {
          params.append('module_id', moduleId);
        }

        const response = await fetch(
          `${API_BASE_URL}/personalization/effective-preference?${params.toString()}`,
          {
            headers: getAuthHeader(),
          }
        );

        if (!response.ok) {
          throw new Error('Failed to fetch effective preference');
        }

        const data = await response.json();
        return data;
      } catch (err) {
        setError(err.message);
        throw err;
      }
    },
    [API_BASE_URL, getAuthHeader]
  );

  // Set global preference
  const setGlobalPreference = useCallback(
    async (preferenceData) => {
      try {
        setIsLoading(true);
        setError(null);

        const response = await fetch(`${API_BASE_URL}/personalization/global-preference`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            ...getAuthHeader(),
          },
          body: JSON.stringify(preferenceData),
        });

        if (!response.ok) {
          const errData = await response.json();
          throw new Error(errData.detail || 'Failed to set preference');
        }

        // Refresh preferences
        await fetchAllPreferences();

        return await response.json();
      } catch (err) {
        setError(err.message);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [API_BASE_URL, getAuthHeader, fetchAllPreferences]
  );

  // Set module preference
  const setModulePreference = useCallback(
    async (moduleId, preferenceData) => {
      try {
        setIsLoading(true);
        setError(null);

        const response = await fetch(`${API_BASE_URL}/personalization/module-preference`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            ...getAuthHeader(),
          },
          body: JSON.stringify({
            ...preferenceData,
            module_id: moduleId,
          }),
        });

        if (!response.ok) {
          const errData = await response.json();
          throw new Error(errData.detail || 'Failed to set preference');
        }

        // Refresh preferences
        await fetchAllPreferences();

        return await response.json();
      } catch (err) {
        setError(err.message);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [API_BASE_URL, getAuthHeader, fetchAllPreferences]
  );

  // Set chapter preference
  const setChapterPreference = useCallback(
    async (chapterId, preferenceData) => {
      try {
        setIsLoading(true);
        setError(null);

        const response = await fetch(`${API_BASE_URL}/personalization/chapter-preference`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            ...getAuthHeader(),
          },
          body: JSON.stringify({
            ...preferenceData,
            chapter_id: chapterId,
          }),
        });

        if (!response.ok) {
          const errData = await response.json();
          throw new Error(errData.detail || 'Failed to set preference');
        }

        // Refresh preferences
        await fetchAllPreferences();

        return await response.json();
      } catch (err) {
        setError(err.message);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [API_BASE_URL, getAuthHeader, fetchAllPreferences]
  );

  // Delete preference
  const deletePreference = useCallback(
    async (level, chapterId = null, moduleId = null) => {
      try {
        setIsLoading(true);
        setError(null);

        const params = new URLSearchParams();
        if (chapterId) params.append('chapter_id', chapterId);
        if (moduleId) params.append('module_id', moduleId);

        const response = await fetch(
          `${API_BASE_URL}/personalization/preference/${level}?${params.toString()}`,
          {
            method: 'DELETE',
            headers: getAuthHeader(),
          }
        );

        if (!response.ok) {
          const errData = await response.json();
          throw new Error(errData.detail || 'Failed to delete preference');
        }

        // Refresh preferences
        await fetchAllPreferences();

        return true;
      } catch (err) {
        setError(err.message);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [API_BASE_URL, getAuthHeader, fetchAllPreferences]
  );

  return {
    preferences,
    isLoading,
    error,
    fetchAllPreferences,
    getEffectivePreference,
    setGlobalPreference,
    setModulePreference,
    setChapterPreference,
    deletePreference,
  };
}
