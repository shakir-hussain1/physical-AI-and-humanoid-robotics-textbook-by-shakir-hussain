import React, { useState } from 'react';

// Chatbot functionality temporarily disabled
const API_URL = null; // 'https://physical-ai-and-humanoid-robotics-textbook-by-sh-production.up.railway.app' // deployed API_URL

export default function ChatWidget() {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);

  const getSelectedText = () => window.getSelection()?.toString() || '';

  const sendMessage = async () => {
    if (!input.trim()) return;
    const selectedText = getSelectedText();
    const userMsg = { role: 'user', content: input };
    setMessages(prev => [...prev, userMsg]);
    setInput('');
    setLoading(true);

    // Check if API_URL is available
    if (!API_URL) {
      // Simulate a response when API is not configured
      setTimeout(() => {
        setMessages(prev => [...prev, {
          role: 'assistant',
          content: 'Chatbot is currently disabled. The backend service is not configured.'
        }]);
        setLoading(false);
      }, 500);
      return;
    }

    try {
      const res = await fetch(`${API_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question: input, selected_text: selectedText })
      });

      if (!res.ok) {
        throw new Error(`HTTP error! status: ${res.status}`);
      }

      const data = await res.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.answer }]);
    } catch (e) {
      console.error('Chat error:', e);
      setMessages(prev => [...prev, { role: 'assistant', content: `Error connecting to server: ${e.message || 'Unknown error'}` }]);
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