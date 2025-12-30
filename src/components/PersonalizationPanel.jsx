import React, { useState, useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';
import styles from './PersonalizationPanel.module.css';

export function PersonalizationPanel({ onClose }) {
  const { user, getAuthHeader } = useAuth();
  const [activeTab, setActiveTab] = useState('global');
  const [preferences, setPreferences] = useState({
    global: null,
    modules: {},
    chapters: {},
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  const API_BASE_URL = 'http://localhost:8001/api';

  useEffect(() => {
    fetchAllPreferences();
  }, []);

  const fetchAllPreferences = async () => {
    try {
      setIsLoading(true);
      setError('');

      const response = await fetch(`${API_BASE_URL}/personalization/all`, {
        headers: getAuthHeader(),
      });

      if (!response.ok) {
        throw new Error('Failed to load preferences');
      }

      const data = await response.json();
      setPreferences({
        global: data.global_preference,
        modules: data.module_preferences || {},
        chapters: data.chapter_preferences || {},
      });
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  const savePreference = async (level, preferenceData) => {
    try {
      setIsLoading(true);
      setError('');
      setSuccess('');

      let endpoint = '/personalization/global-preference';
      if (level === 'module') {
        endpoint = '/personalization/module-preference';
      } else if (level === 'chapter') {
        endpoint = '/personalization/chapter-preference';
      }

      const response = await fetch(`${API_BASE_URL}${endpoint}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          ...getAuthHeader(),
        },
        body: JSON.stringify(preferenceData),
      });

      if (!response.ok) {
        const errData = await response.json();
        throw new Error(errData.detail || 'Failed to save preference');
      }

      setSuccess('Preference saved successfully');
      await fetchAllPreferences();

      // Clear success message after 3 seconds
      setTimeout(() => setSuccess(''), 3000);
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.panel}>
      <div className={styles.header}>
        <h2>Content Personalization</h2>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close panel"
        >
          &times;
        </button>
      </div>

      {error && <div className={styles.errorAlert}>{error}</div>}
      {success && <div className={styles.successAlert}>{success}</div>}

      <div className={styles.tabs}>
        <button
          className={`${styles.tab} ${activeTab === 'global' ? styles.active : ''}`}
          onClick={() => setActiveTab('global')}
        >
          Global
        </button>
        <button
          className={`${styles.tab} ${activeTab === 'modules' ? styles.active : ''}`}
          onClick={() => setActiveTab('modules')}
        >
          Modules
        </button>
        <button
          className={`${styles.tab} ${activeTab === 'chapters' ? styles.active : ''}`}
          onClick={() => setActiveTab('chapters')}
        >
          Chapters
        </button>
      </div>

      <div className={styles.content}>
        {activeTab === 'global' && (
          <GlobalPreferenceTab
            preference={preferences.global}
            onSave={(data) => savePreference('global', data)}
            isLoading={isLoading}
          />
        )}

        {activeTab === 'modules' && (
          <ModulePreferenceTab
            preferences={preferences.modules}
            onSave={(moduleId, data) => savePreference('module', { ...data, module_id: moduleId })}
            isLoading={isLoading}
          />
        )}

        {activeTab === 'chapters' && (
          <ChapterPreferenceTab
            preferences={preferences.chapters}
            onSave={(chapterId, data) => {
              const preferenceData = {
                ...data,
                chapter_id: chapterId
              };
              // module_id is optional but include if provided
              savePreference('chapter', preferenceData);
            }}
            isLoading={isLoading}
          />
        )}
      </div>
    </div>
  );
}

function GlobalPreferenceTab({ preference, onSave, isLoading }) {
  const [formData, setFormData] = useState(
    preference || {
      content_level: 'beginner',
      show_math: true,
      show_code: true,
      show_diagrams: true,
      show_advanced_topics: false,
      focus_areas: [],
    }
  );

  const handleChange = (field, value) => {
    setFormData((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    onSave(formData);
  };

  return (
    <div className={styles.preferenceForm}>
      <h3>Global Content Settings</h3>
      <p className={styles.description}>
        These settings apply to all chapters unless overridden at the module or chapter level.
      </p>

      <form onSubmit={handleSubmit}>
        <div className={styles.formGroup}>
          <label className={styles.label}>Content Level</label>
          <select
            value={formData.content_level}
            onChange={(e) => handleChange('content_level', e.target.value)}
            className={styles.select}
            disabled={isLoading}
          >
            <option value="beginner">Beginner - Introductory concepts</option>
            <option value="intermediate">Intermediate - Building on basics</option>
            <option value="advanced">Advanced - In-depth topics</option>
          </select>
        </div>

        <div className={styles.checkboxGroup}>
          <label className={styles.checkboxLabel}>
            <input
              type="checkbox"
              checked={formData.show_math}
              onChange={(e) => handleChange('show_math', e.target.checked)}
              disabled={isLoading}
            />
            <span>Show Mathematical Content</span>
          </label>

          <label className={styles.checkboxLabel}>
            <input
              type="checkbox"
              checked={formData.show_code}
              onChange={(e) => handleChange('show_code', e.target.checked)}
              disabled={isLoading}
            />
            <span>Show Code Examples</span>
          </label>

          <label className={styles.checkboxLabel}>
            <input
              type="checkbox"
              checked={formData.show_diagrams}
              onChange={(e) => handleChange('show_diagrams', e.target.checked)}
              disabled={isLoading}
            />
            <span>Show Diagrams & Visuals</span>
          </label>

          <label className={styles.checkboxLabel}>
            <input
              type="checkbox"
              checked={formData.show_advanced_topics}
              onChange={(e) => handleChange('show_advanced_topics', e.target.checked)}
              disabled={isLoading}
            />
            <span>Show Advanced Topics</span>
          </label>
        </div>

        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? 'Saving...' : 'Save Preferences'}
        </button>
      </form>
    </div>
  );
}

function ModulePreferenceTab({ preferences, onSave, isLoading }) {
  const [mode, setMode] = useState('select'); // 'select' or 'edit'
  const [selectedModule, setSelectedModule] = useState(
    Object.keys(preferences)[0] || null
  );
  const [newModuleId, setNewModuleId] = useState('');
  const [formData, setFormData] = useState({
    content_level: 'beginner',
    show_math: true,
    show_code: true,
    show_diagrams: true,
    show_advanced_topics: false,
    focus_areas: [],
  });

  const handleSelectExisting = (moduleId) => {
    if (moduleId === '') {
      setMode('create');
      setSelectedModule(null);
      setNewModuleId('');
      return;
    }
    setMode('edit');
    setSelectedModule(moduleId);
    setFormData(
      preferences[moduleId] || {
        content_level: 'beginner',
        show_math: true,
        show_code: true,
        show_diagrams: true,
        show_advanced_topics: false,
        focus_areas: [],
      }
    );
  };

  const handleCreateNew = () => {
    if (!newModuleId.trim()) {
      alert('Please enter a module ID');
      return;
    }
    setSelectedModule(newModuleId);
    setMode('edit');
    setFormData({
      content_level: 'beginner',
      show_math: true,
      show_code: true,
      show_diagrams: true,
      show_advanced_topics: false,
      focus_areas: [],
    });
  };

  const handleChange = (field, value) => {
    setFormData((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (selectedModule) {
      onSave(selectedModule, formData);
      // Reset after save
      setMode('select');
      setSelectedModule(null);
      setNewModuleId('');
    }
  };

  return (
    <div className={styles.preferenceForm}>
      <h3>Module-Level Settings</h3>
      <p className={styles.description}>
        Set content preferences for specific modules.
      </p>

      {mode === 'select' && (
        <div className={styles.formGroup}>
          <label className={styles.label}>Select or Create Module</label>
          <select
            value={selectedModule || ''}
            onChange={(e) => handleSelectExisting(e.target.value)}
            className={styles.select}
            disabled={isLoading}
          >
            <option value="">+ Create new module preference...</option>
            {Object.keys(preferences).length > 0 && (
              <>
                <option value="" disabled>
                  ────────────────────────
                </option>
                {Object.keys(preferences).map((moduleId) => (
                  <option key={moduleId} value={moduleId}>
                    {moduleId}
                  </option>
                ))}
              </>
            )}
          </select>
        </div>
      )}

      {mode === 'create' && (
        <div className={styles.formGroup}>
          <label className={styles.label}>Module ID</label>
          <input
            type="text"
            placeholder="Enter module ID (e.g., module1, introduction)"
            value={newModuleId}
            onChange={(e) => setNewModuleId(e.target.value)}
            className={styles.input}
            disabled={isLoading}
          />
          <button
            type="button"
            onClick={handleCreateNew}
            className={styles.submitButton}
            disabled={isLoading || !newModuleId.trim()}
            style={{ marginTop: '10px' }}
          >
            Continue with this Module
          </button>
          <button
            type="button"
            onClick={() => setMode('select')}
            className={styles.submitButton}
            disabled={isLoading}
            style={{
              marginTop: '10px',
              background: '#666',
            }}
          >
            Back
          </button>
        </div>
      )}

      {mode === 'edit' && selectedModule && (
        <form onSubmit={handleSubmit}>
          <div
            style={{
              background: '#f0f0f0',
              padding: '10px',
              borderRadius: '4px',
              marginBottom: '15px',
            }}
          >
            <p style={{ margin: 0, fontSize: '14px' }}>
              <strong>Module:</strong> {selectedModule}
            </p>
          </div>

          <div className={styles.formGroup}>
            <label className={styles.label}>Content Level</label>
            <select
              value={formData.content_level}
              onChange={(e) => handleChange('content_level', e.target.value)}
              className={styles.select}
              disabled={isLoading}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className={styles.checkboxGroup}>
            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={formData.show_math}
                onChange={(e) => handleChange('show_math', e.target.checked)}
                disabled={isLoading}
              />
              <span>Show Math</span>
            </label>

            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={formData.show_code}
                onChange={(e) => handleChange('show_code', e.target.checked)}
                disabled={isLoading}
              />
              <span>Show Code</span>
            </label>

            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={formData.show_diagrams}
                onChange={(e) => handleChange('show_diagrams', e.target.checked)}
                disabled={isLoading}
              />
              <span>Show Diagrams</span>
            </label>

            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={formData.show_advanced_topics}
                onChange={(e) => handleChange('show_advanced_topics', e.target.checked)}
                disabled={isLoading}
              />
              <span>Show Advanced Topics</span>
            </label>
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? 'Saving...' : 'Save Module Settings'}
          </button>
          <button
            type="button"
            onClick={() => setMode('select')}
            className={styles.submitButton}
            disabled={isLoading}
            style={{
              marginLeft: '10px',
              background: '#666',
            }}
          >
            Back
          </button>
        </form>
      )}
    </div>
  );
}

function ChapterPreferenceTab({ preferences, onSave, isLoading }) {
  const [mode, setMode] = useState('select'); // 'select' or 'edit'
  const [selectedChapter, setSelectedChapter] = useState(
    Object.keys(preferences)[0] || null
  );
  const [newChapterId, setNewChapterId] = useState('');
  const [newModuleId, setNewModuleId] = useState('');
  const [formData, setFormData] = useState({
    content_level: 'beginner',
    show_math: true,
    show_code: true,
    show_diagrams: true,
    show_advanced_topics: false,
    focus_areas: [],
  });

  const handleSelectExisting = (chapterId) => {
    if (chapterId === '') {
      setMode('create');
      setSelectedChapter(null);
      setNewChapterId('');
      setNewModuleId('');
      return;
    }
    setMode('edit');
    setSelectedChapter(chapterId);
    setFormData(
      preferences[chapterId] || {
        content_level: 'beginner',
        show_math: true,
        show_code: true,
        show_diagrams: true,
        show_advanced_topics: false,
        focus_areas: [],
      }
    );
  };

  const handleCreateNew = () => {
    if (!newChapterId.trim()) {
      alert('Please enter a chapter ID');
      return;
    }
    setSelectedChapter(newChapterId);
    setMode('edit');
    setFormData({
      content_level: 'beginner',
      show_math: true,
      show_code: true,
      show_diagrams: true,
      show_advanced_topics: false,
      focus_areas: [],
    });
  };

  const handleChange = (field, value) => {
    setFormData((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (selectedChapter) {
      const submitData = { ...formData };
      if (newModuleId.trim()) {
        submitData.module_id = newModuleId;
      }
      onSave(selectedChapter, submitData);
      // Reset after save
      setMode('select');
      setSelectedChapter(null);
      setNewChapterId('');
      setNewModuleId('');
    }
  };

  return (
    <div className={styles.preferenceForm}>
      <h3>Chapter-Level Settings</h3>
      <p className={styles.description}>
        Fine-tune content for specific chapters.
      </p>

      {mode === 'select' && (
        <div className={styles.formGroup}>
          <label className={styles.label}>Select or Create Chapter</label>
          <select
            value={selectedChapter || ''}
            onChange={(e) => handleSelectExisting(e.target.value)}
            className={styles.select}
            disabled={isLoading}
          >
            <option value="">+ Create new chapter preference...</option>
            {Object.keys(preferences).length > 0 && (
              <>
                <option value="" disabled>
                  ────────────────────────
                </option>
                {Object.keys(preferences).map((chapterId) => (
                  <option key={chapterId} value={chapterId}>
                    {chapterId}
                  </option>
                ))}
              </>
            )}
          </select>
        </div>
      )}

      {mode === 'create' && (
        <div>
          <div className={styles.formGroup}>
            <label className={styles.label}>Chapter ID</label>
            <input
              type="text"
              placeholder="Enter chapter ID (e.g., chapter1, introduction)"
              value={newChapterId}
              onChange={(e) => setNewChapterId(e.target.value)}
              className={styles.input}
              disabled={isLoading}
            />
          </div>

          <div className={styles.formGroup}>
            <label className={styles.label}>Module ID (optional)</label>
            <input
              type="text"
              placeholder="Enter module ID if this chapter belongs to a module"
              value={newModuleId}
              onChange={(e) => setNewModuleId(e.target.value)}
              className={styles.input}
              disabled={isLoading}
            />
          </div>

          <button
            type="button"
            onClick={handleCreateNew}
            className={styles.submitButton}
            disabled={isLoading || !newChapterId.trim()}
          >
            Continue with this Chapter
          </button>
          <button
            type="button"
            onClick={() => setMode('select')}
            className={styles.submitButton}
            disabled={isLoading}
            style={{
              marginLeft: '10px',
              background: '#666',
            }}
          >
            Back
          </button>
        </div>
      )}

      {mode === 'edit' && selectedChapter && (
        <form onSubmit={handleSubmit}>
          <div
            style={{
              background: '#f0f0f0',
              padding: '10px',
              borderRadius: '4px',
              marginBottom: '15px',
            }}
          >
            <p style={{ margin: '5px 0', fontSize: '14px' }}>
              <strong>Chapter:</strong> {selectedChapter}
            </p>
            {newModuleId && (
              <p style={{ margin: '5px 0', fontSize: '14px' }}>
                <strong>Module:</strong> {newModuleId}
              </p>
            )}
          </div>

          <div className={styles.formGroup}>
            <label className={styles.label}>Content Level</label>
            <select
              value={formData.content_level}
              onChange={(e) => handleChange('content_level', e.target.value)}
              className={styles.select}
              disabled={isLoading}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className={styles.checkboxGroup}>
            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={formData.show_math}
                onChange={(e) => handleChange('show_math', e.target.checked)}
                disabled={isLoading}
              />
              <span>Show Math</span>
            </label>

            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={formData.show_code}
                onChange={(e) => handleChange('show_code', e.target.checked)}
                disabled={isLoading}
              />
              <span>Show Code</span>
            </label>

            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={formData.show_diagrams}
                onChange={(e) => handleChange('show_diagrams', e.target.checked)}
                disabled={isLoading}
              />
              <span>Show Diagrams</span>
            </label>

            <label className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={formData.show_advanced_topics}
                onChange={(e) => handleChange('show_advanced_topics', e.target.checked)}
                disabled={isLoading}
              />
              <span>Show Advanced Topics</span>
            </label>
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? 'Saving...' : 'Save Chapter Settings'}
          </button>
          <button
            type="button"
            onClick={() => setMode('select')}
            className={styles.submitButton}
            disabled={isLoading}
            style={{
              marginLeft: '10px',
              background: '#666',
            }}
          >
            Back
          </button>
        </form>
      )}
    </div>
  );
}
