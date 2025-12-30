import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatbotWidget.module.css';

export default function ChatbotWidget() {
  const [messages, setMessages] = useState([
    {
      id: 1,
      type: 'bot',
      text: 'Hello! 👋 I\'m your RAG Chatbot assistant. Ask me anything about Robotics, ROS2, Physics AI, Digital Twins, and Humanoid Robotics!',
      timestamp: new Date(),
    },
  ]);
  const [inputText, setInputText] = useState('');
  const [loading, setLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef(null);
  const [selectedText, setSelectedText] = useState('');

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputText.trim()) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      type: 'user',
      text: inputText,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputText('');
    setLoading(true);

    try {
      const response = await fetch('http://localhost:8001/api/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputText,
          conversation_history: [],
          user_role: 'student',
        }),
      });

      if (!response.ok) {
        throw new Error(`API Error: ${response.status}`);
      }

      const data = await response.json();

      // Only show top 1-2 most relevant sources if they're highly relevant
      const relevantSources = (data.sources || [])
        .filter(source => source.relevance_score > 0.7) // Only show if >70% relevant
        .slice(0, 1); // Only top 1 source

      const botMessage = {
        id: Date.now() + 1,
        type: 'bot',
        text: data.answer,
        sources: relevantSources.length > 0 ? relevantSources : null, // null if no relevant sources
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        type: 'bot',
        text: `Error: ${error.message}. Make sure the backend server is running: python -m uvicorn backend.src.api.main:app --reload`,
        timestamp: new Date(),
        isError: true,
      };
      setMessages((prev) => [...prev, errorMessage]);
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

  return (
    <div className={styles.chatbotContainer}>
      {/* Chat Toggle Button */}
      {!isOpen && (
        <button
          className={styles.chatbotToggle}
          onClick={() => setIsOpen(true)}
          title="Open Chat"
        >
          💬
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatbotWindow}>
          {/* Header */}
          <div className={styles.chatbotHeader}>
            <div className={styles.headerTitle}>
              <span className={styles.botIcon}>🤖</span>
              <div>
                <h3>RAG Chatbot</h3>
                <p>Ask about Robotics & AI</p>
              </div>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              title="Close Chat"
            >
              ✕
            </button>
          </div>

          {/* Messages Container */}
          <div className={styles.messagesContainer}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.type]} ${
                  message.isError ? styles.error : ''
                }`}
              >
                <div className={styles.messageContent}>
                  <p className={styles.messageText}>{message.text}</p>

                  {/* Simplified Source Display - Only if highly relevant */}
                  {message.sources && message.sources.length > 0 && (
                    <div className={styles.sources}>
                      <div className={styles.source}>
                        <a
                          href={message.sources[0].url}
                          target="_blank"
                          rel="noopener noreferrer"
                          className={styles.sourceLink}
                        >
                          📚 {message.sources[0].page_title || 'Reference'}
                        </a>
                      </div>
                    </div>
                  )}
                </div>
              </div>
            ))}

            {loading && (
              <div className={`${styles.message} ${styles.bot}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className={styles.inputArea}>
            <textarea
              className={styles.input}
              placeholder="Ask a question... (Press Enter to send)"
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={loading}
              rows="3"
            />
            <button
              className={styles.sendButton}
              onClick={sendMessage}
              disabled={loading || !inputText.trim()}
              title="Send Message"
            >
              {loading ? '⏳ Sending...' : '📤 Send'}
            </button>
          </div>

          {/* Footer */}
          <div className={styles.footer}>
            <small>Backend: http://localhost:8001</small>
          </div>
        </div>
      )}
    </div>
  );
}
