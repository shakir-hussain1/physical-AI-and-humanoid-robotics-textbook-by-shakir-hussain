import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';
import styles from './UrduTranslationPanel.module.css';

/**
 * Urdu Translation Panel - Complete working component
 * Users can enter English text and get Urdu translation
 */
export function UrduTranslationPanel({ onClose }) {
  const { getAuthHeader } = useAuth();

  const [englishText, setEnglishText] = useState('');
  const [urduText, setUrduText] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [accuracy, setAccuracy] = useState(null);
  const [showTranslation, setShowTranslation] = useState(false);

  const API_BASE_URL = 'http://localhost:8001/api';

  const handleTranslate = async () => {
    // Validation
    if (!englishText.trim()) {
      setError('براہ کرم متن درج کریں (Please enter text to translate)');
      return;
    }

    setLoading(true);
    setError('');
    setSuccess('');
    setShowTranslation(false);

    try {
      const response = await fetch(`${API_BASE_URL}/translation/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          ...getAuthHeader(),
        },
        body: JSON.stringify({
          content_id: `trans_${Date.now()}`,
          content_text: englishText,
          content_type: 'paragraph',
          target_language: 'ur',
        }),
      });

      if (!response.ok) {
        const errData = await response.json();
        throw new Error(errData.detail || 'ترجمہ ناکام (Translation failed)');
      }

      const data = await response.json();

      setUrduText(data.translated_content);
      setAccuracy(data.accuracy_score);
      setShowTranslation(true);
      setSuccess(`✓ ترجمہ مکمل! درستگی: ${data.accuracy_score.toFixed(0)}%`);
    } catch (err) {
      setError(`خرابی: ${err.message}`);
      setShowTranslation(false);
    } finally {
      setLoading(false);
    }
  };

  const handleCopy = (text) => {
    navigator.clipboard.writeText(text);
    setSuccess('✓ کاپی ہو گیا!');
    setTimeout(() => setSuccess(''), 2000);
  };

  const handleClear = () => {
    setEnglishText('');
    setUrduText('');
    setShowTranslation(false);
    setError('');
    setSuccess('');
    setAccuracy(null);
  };

  return (
    <div className={styles.panel}>
      <div className={styles.header}>
        <h2>📖 Urdu Translation / اردو ترجمہ</h2>
        <button className={styles.closeBtn} onClick={onClose}>✕</button>
      </div>

      <div className={styles.content}>
        {/* Input Section */}
        <div className={styles.section}>
          <label className={styles.label}>English Text (انگریزی متن)</label>
          <textarea
            className={styles.textarea}
            placeholder="Enter English text to translate... (ترجمہ کے لیے انگریزی متن درج کریں)"
            value={englishText}
            onChange={(e) => setEnglishText(e.target.value)}
            disabled={loading}
            rows="4"
          />
          <div className={styles.charCount}>
            {englishText.length} characters
          </div>
        </div>

        {/* Error Message */}
        {error && (
          <div className={styles.errorBox}>
            <span className={styles.errorIcon}>⚠️</span>
            {error}
          </div>
        )}

        {/* Success Message */}
        {success && (
          <div className={styles.successBox}>
            {success}
          </div>
        )}

        {/* Translate Button */}
        <button
          className={styles.translateBtn}
          onClick={handleTranslate}
          disabled={loading || !englishText.trim()}
        >
          {loading ? (
            <>
              <span className={styles.spinner}>⏳</span>
              ترجمہ جاری ہے... (Translating...)
            </>
          ) : (
            <>
              <span>🔄</span>
              اردو میں ترجمہ کریں (Translate to Urdu)
            </>
          )}
        </button>

        {/* Output Section */}
        {showTranslation && (
          <div className={styles.section}>
            <div className={styles.translationHeader}>
              <label className={styles.label}>Urdu Translation (اردو ترجمہ)</label>
              {accuracy !== null && (
                <div className={styles.accuracyBadge}>
                  <span className={styles.accuracyLabel}>Accuracy</span>
                  <span className={styles.accuracyValue}>{accuracy.toFixed(0)}%</span>
                </div>
              )}
            </div>

            <div className={styles.urduBox}>
              <p className={styles.urduText} dir="rtl">{urduText}</p>
            </div>

            <button
              className={styles.copyBtn}
              onClick={() => handleCopy(urduText)}
              title="کاپی کریں (Copy)"
            >
              📋 کاپی کریں (Copy)
            </button>
          </div>
        )}

        {/* Info Box */}
        <div className={styles.infoBox}>
          <div className={styles.infoTitle}>💡 کیسے استعمال کریں</div>
          <ul className={styles.infoList}>
            <li>انگریزی متن درج کریں</li>
            <li>"اردو میں ترجمہ کریں" بٹن دبائیں</li>
            <li>اردو ترجمہ دیکھیں اور کاپی کریں</li>
            <li>ہر ترجمے کی درستگی فیصد دکھائی دے گی</li>
          </ul>
        </div>

        {/* Clear Button */}
        {(englishText || urduText) && (
          <button className={styles.clearBtn} onClick={handleClear}>
            🗑️ صاف کریں (Clear)
          </button>
        )}
      </div>
    </div>
  );
}

export default UrduTranslationPanel;
