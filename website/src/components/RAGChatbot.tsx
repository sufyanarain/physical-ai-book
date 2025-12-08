import React, { useState, useEffect, useRef } from 'react';
import ReactMarkdown from 'react-markdown';
import '@site/src/css/custom.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  context?: string;
}

// Use Railway backend for both dev and production
const BACKEND_URL = 'https://physical-ai-backend-production-b62f.up.railway.app';

export default function RAGChatbot(): React.ReactElement {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      role: 'assistant',
      content: 'Hi! I\'m your Physical AI textbook assistant. Ask me anything about robotics, ROS 2, simulation, or any content from the book! You can also select text on the page and ask me questions about it.'
    }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 3) {
        setSelectedText(text);
        // Auto-open chatbot when text is selected
        setIsOpen(true);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage: Message = {
      role: 'user',
      content: selectedText ? `Context: "${selectedText}"\n\nQuestion: ${input}` : input,
      context: selectedText || undefined
    };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      // Build conversation history for the chat endpoint
      const conversationHistory = [...messages, userMessage].map(msg => ({
        role: msg.role,
        content: msg.role === 'user' && msg.context
          ? `Context: "${msg.context}"\n\nQuestion: ${msg.content}`
          : msg.content
      }));

      const response = await fetch(`${BACKEND_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          messages: conversationHistory
        })
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();
      const botMessage: Message = { role: 'assistant', content: data.response };
      setMessages(prev => [...prev, botMessage]);
      setSelectedText(''); // Clear selected text after using it
    } catch (error) {
      console.error('Error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please make sure the backend server is running at ' + BACKEND_URL
      }]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="chatbot-container">
      {/* Floating button */}
      {!isOpen && (
        <button
          className="chatbot-button"
          onClick={() => setIsOpen(true)}
          aria-label="Open chatbot"
        >
          💬
        </button>
      )}

      {/* Chatbot window */}
      {isOpen && (
        <div className="chatbot-window">
          {/* Header */}
          <div className="chatbot-header">
            <span>🤖 Physical AI Assistant</span>
            <button
              className="chatbot-close"
              onClick={() => setIsOpen(false)}
              aria-label="Close chatbot"
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className="chatbot-messages">
            {messages.map((msg, idx) => (
              <div key={idx}>
                {msg.context && msg.role === 'user' && (
                  <div className="context-bubble">
                    <div className="context-label">📄 Context</div>
                    <div className="context-text">{msg.context}</div>
                  </div>
                )}
                <div
                  className={`message ${msg.role === 'user' ? 'message-user' : 'message-bot'}`}
                >
                  {msg.role === 'assistant' ? (
                    <ReactMarkdown>{msg.content}</ReactMarkdown>
                  ) : (
                    msg.content
                  )}
                </div>
              </div>
            ))}
            {loading && (
              <div className="message message-bot loading-message">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className="chatbot-input-container">
            {selectedText && (
              <div className="selected-text-preview">
                <div className="selected-text-header">
                  <span>📄 Selected Context</span>
                  <button 
                    className="clear-context-btn"
                    onClick={() => setSelectedText('')}
                    title="Clear context"
                  >
                    ✕
                  </button>
                </div>
                <div className="selected-text-content">
                  {selectedText.length > 100 ? selectedText.substring(0, 100) + '...' : selectedText}
                </div>
              </div>
            )}
            <div className="input-wrapper">
              <input
                type="text"
                className="chatbot-input"
                placeholder="Ask a question..."
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                disabled={loading}
              />
              <button
                className="chatbot-send"
                onClick={sendMessage}
                disabled={loading || !input.trim()}
                title="Send message"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
