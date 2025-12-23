import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

// Use Railway deployment or local backend
const API_URL = process.env.REACT_APP_API_URL ||
                 (typeof window !== 'undefined' && window.location.hostname === 'localhost'
                   ? 'http://localhost:8000'
                   : 'https://physical-ai-and-humanoid-robotics-textbook-by-sh-production.up.railway.app');

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 'initial',
      type: 'assistant',
      content: 'Hello! I am your Physical AI & Humanoid Robotics textbook assistant. You can ask me any questions about the textbook content.',
      sources: []
    }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [error, setError] = useState('');
  const messagesEndRef = useRef(null);
  const conversationHistoryRef = useRef([]);

  // Auto scroll to bottom
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Get selected text from page
  const handleTextSelection = () => {
    const selection = window.getSelection();
    const selected = selection.toString().trim();
    if (selected) {
      setSelectedText(selected);
    }
  };

  const clearSelection = () => {
    setSelectedText('');
  };

  const sendMessage = async () => {
    if (!input.trim()) return;

    setError('');
    const userMessage = {
      id: Date.now(),
      type: 'user',
      content: input,
      sources: []
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      // Build conversation history
      const conversationHistory = messages
        .filter(m => m.type !== 'error')
        .map(m => ({
          role: m.type === 'user' ? 'user' : 'assistant',
          content: m.content
        }));

      const payload = {
        query: userMessage.content,
        user_context: selectedText || null,
        conversation_history: conversationHistory
      };

      const response = await fetch(`${API_URL}/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP ${response.status}`);
      }

      const data = await response.json();

      const assistantMessage = {
        id: Date.now() + 1,
        type: 'assistant',
        content: data.answer,
        sources: data.sources || [],
        confidence: data.confidence,
        confidenceLevel: data.confidence_level
      };

      setMessages(prev => [...prev, assistantMessage]);
      conversationHistoryRef.current.push(userMessage, assistantMessage);

    } catch (err) {
      console.error('Chat error:', err);
      setError(`Error: ${err.message}`);

      const errorMessage = {
        id: Date.now() + 2,
        type: 'error',
        content: `Sorry, I encountered an error: ${err.message}. Please check if the backend server is running.`,
        sources: []
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Floating chat button when closed
  if (!isOpen) {
    return (
      <div className={styles.container}>
        <button
          className={styles.fab}
          onClick={() => setIsOpen(true)}
          title="Open Chat"
          onMouseEnter={handleTextSelection}
        >
          <span className={styles.fabIcon}>💬</span>
        </button>
      </div>
    );
  }

  // Chat window when open
  return (
    <div className={styles.container}>
      <div className={styles.chatWindow}>
        {/* Header */}
        <div className={styles.header}>
          <div className={styles.headerTitle}>
            <h3>Physical AI & Robotics Assistant</h3>
            <p className={styles.headerSubtitle}>Powered by RAG Chatbot</p>
          </div>
          <button
            className={styles.closeButton}
            onClick={() => setIsOpen(false)}
            title="Close Chat"
          >
            ✕
          </button>
        </div>

        {/* Selected Text Display */}
        {selectedText && (
          <div className={styles.selectedTextBanner}>
            <div className={styles.selectedTextContent}>
              <strong>Selected Text:</strong>
              <p>{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}</p>
            </div>
            <button
              className={styles.clearButton}
              onClick={clearSelection}
              title="Clear selection"
            >
              ✕
            </button>
          </div>
        )}

        {/* Messages */}
        <div className={styles.messagesContainer}>
          {messages.map((message) => (
            <div key={message.id} className={`${styles.message} ${styles[message.type]}`}>
              <div className={styles.messageContent}>
                <div className={styles.messageText}>
                  {message.content}
                </div>

                {/* Sources */}
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <p className={styles.sourcesLabel}>Sources:</p>
                    {message.sources.map((source, idx) => (
                      <div key={idx} className={styles.sourceItem}>
                        <strong>{source.title}</strong>
                        <span className={styles.sourceMetadata}>
                          {source.source}
                          {source.category && <span className={styles.category}>{source.category}</span>}
                        </span>
                      </div>
                    ))}
                  </div>
                )}

                {/* Confidence Badge */}
                {message.confidenceLevel && (
                  <div className={styles.confidenceBadge}>
                    Confidence: <span className={styles[`confidence${message.confidenceLevel}`]}>
                      {message.confidenceLevel}
                    </span>
                  </div>
                )}
              </div>
            </div>
          ))}

          {loading && (
            <div className={`${styles.message} ${styles.assistant}`}>
              <div className={styles.messageContent}>
                <div className={styles.loading}>
                  <span className={styles.dot}></span>
                  <span className={styles.dot}></span>
                  <span className={styles.dot}></span>
                </div>
              </div>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        {/* Error Display */}
        {error && (
          <div className={styles.errorBanner}>
            {error}
          </div>
        )}

        {/* Input Area */}
        <div className={styles.inputArea}>
          <textarea
            className={styles.input}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask a question about the textbook... (Shift+Enter for new line)"
            rows="3"
            disabled={loading}
          />
          <div className={styles.inputFooter}>
            <small className={styles.helperText}>
              {selectedText ? '✓ Text selected as context' : 'Select text on the page for context'}
            </small>
            <button
              className={`${styles.sendButton} ${loading ? styles.disabled : ''}`}
              onClick={sendMessage}
              disabled={!input.trim() || loading}
              title="Send message (Enter)"
            >
              {loading ? '...' : 'Send'}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
