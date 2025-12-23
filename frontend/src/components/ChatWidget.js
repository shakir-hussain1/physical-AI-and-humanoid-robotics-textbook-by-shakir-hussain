
import React, { useState, useEffect } from 'react';

// Backend API URL - auto-detect based on environment
function getApiUrl() {
  if (typeof window === 'undefined') {
    return 'http://localhost:8000';
  }

  const isDev = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1';

  if (isDev) {
    return 'http://localhost:8000';
  }

  // For production (GitHub Pages, Railway, etc.)
  // Update this to your deployed backend URL
  return 'http://localhost:8000'; // Fallback - change for production
}

const API_URL = getApiUrl();

export default function ChatWidget() {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([
    { role: 'assistant', content: 'Hi! I\'m your Physical AI & Humanoid Robotics assistant. Ask me anything about the textbook. Tip: Select text on the page first to ask about it!' }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [backendHealthy, setBackendHealthy] = useState(null);

  // Check backend health on mount
  useEffect(() => {
    checkBackendHealth();
  }, []);

  const checkBackendHealth = async () => {
    try {
      const res = await fetch(`${API_URL}/health`, {
        method: 'GET',
        headers: { 'Content-Type': 'application/json' },
      });
      setBackendHealthy(res.ok);
    } catch (e) {
      console.warn('[ChatWidget] Backend health check failed:', e.message);
      setBackendHealthy(false);
    }
  };

  const getSelectedText = () => window.getSelection()?.toString() || '';

  const sendMessage = async () => {
    if (!input.trim()) return;

    if (!backendHealthy) {
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: `⚠️ Backend is not reachable at ${API_URL}.\n\n📝 Local Setup:\n1. Start backend: cd backend && uvicorn src.app:app --reload\n2. Backend should be at http://localhost:8000\n3. Check if port 8000 is in use: netstat -ano | findstr :8000\n\n🌐 For production, update API_URL in ChatWidget.js`
      }]);
      setInput('');
      return;
    }

    const selectedText = getSelectedText();
    const userMsg = { role: 'user', content: input };
    setMessages(prev => [...prev, userMsg]);
    setInput('');
    setLoading(true);

    try {
      // IMPORTANT: Backend expects 'user_context', not 'selected_text'
      const payload = {
        query: input,
        user_context: selectedText || null,
        conversation_history: []
      };

      console.log('[ChatWidget] Sending to:', `${API_URL}/chat/query`);
      console.log('[ChatWidget] Payload:', payload);

      const res = await fetch(`${API_URL}/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload)
      });

      if (!res.ok) {
        const errorText = await res.text();
        console.error('[ChatWidget] API Error:', res.status, errorText);
        throw new Error(`HTTP ${res.status}: ${errorText}`);
      }

      const data = await res.json();
      console.log('[ChatWidget] Response:', data);

      const assistantMsg = {
        role: 'assistant',
        content: data.answer || 'No answer received',
        sources: data.sources,
        confidence: data.confidence
      };

      setMessages(prev => [...prev, assistantMsg]);
    } catch (e) {
      console.error('[ChatWidget] Chat error:', e);
      const errorMsg = `❌ Error: ${e.message}\n\n🔧 Troubleshooting:\n- Is backend running? (cd backend && uvicorn src.app:app --reload)\n- Check console (F12) for CORS errors\n- Verify API_URL is correct: ${API_URL}`;
      setMessages(prev => [...prev, { role: 'assistant', content: errorMsg }]);
    }
    setLoading(false);
  };

  if (!open) {
    return (
      <button onClick={() => setOpen(true)} style={styles.fab}>
        💬
      </button>
    );
  }

  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <span>Book Assistant</span>
        <button onClick={() => setOpen(false)} style={styles.close}>×</button>
      </div>
      <div style={styles.messages}>
        {messages.map((m, i) => (
          <div key={i} style={m.role === 'user' ? styles.user : styles.bot}>
            {m.content}
          </div>
        ))}
        {loading && <div style={styles.bot}>Thinking...</div>}
      </div>
      <div style={styles.inputArea}>
        <input
          value={input}
          onChange={e => setInput(e.target.value)}
          onKeyPress={e => e.key === 'Enter' && sendMessage()}
          placeholder="Ask about the book..."
          style={styles.input}
        />
        <button onClick={sendMessage} style={styles.send}>Send</button>
      </div>
      <div style={styles.hint}>Tip: Select text first to ask about it</div>
    </div>
  );
}

const styles = {
  fab: { position: 'fixed', bottom: 20, right: 20, width: 56, height: 56, borderRadius: '50%', border: 'none', background: '#3b82f6', color: '#fff', fontSize: 24, cursor: 'pointer', boxShadow: '0 4px 12px rgba(0,0,0,0.3)', zIndex: 1000 },
  container: { position: 'fixed', bottom: 20, right: 20, width: 350, height: 450, background: '#fff', borderRadius: 12, boxShadow: '0 4px 20px rgba(0,0,0,0.2)', display: 'flex', flexDirection: 'column', zIndex: 1000 },
  header: { padding: 12, background: '#3b82f6', color: '#fff', borderRadius: '12px 12px 0 0', display: 'flex', justifyContent: 'space-between', alignItems: 'center' },
  close: { background: 'none', border: 'none', color: '#fff', fontSize: 20, cursor: 'pointer' },
  messages: { flex: 1, padding: 12, overflowY: 'auto' },
  user: { background: '#e3f2fd', padding: 8, borderRadius: 8, marginBottom: 8, marginLeft: 40 },
  bot: { background: '#f5f5f5', padding: 8, borderRadius: 8, marginBottom: 8, marginRight: 40 },
  inputArea: { padding: 12, display: 'flex', gap: 8, borderTop: '1px solid #eee' },
  input: { flex: 1, padding: 8, border: '1px solid #ddd', borderRadius: 4 },
  send: { padding: '8px 16px', background: '#3b82f6', color: '#fff', border: 'none', borderRadius: 4, cursor: 'pointer' },
  hint: { fontSize: 11, color: '#888', textAlign: 'center', padding: 4 }
};

