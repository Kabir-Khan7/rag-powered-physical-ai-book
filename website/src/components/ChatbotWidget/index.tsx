import React, {useMemo, useState} from 'react';
import styles from './styles.module.css';

type Citation = {
  heading: string;
  source_url: string;
  snippet: string;
};

type Message = {
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
};

const backendUrl = process.env.BACKEND_URL ?? 'http://localhost:8000';

export const ChatbotWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [question, setQuestion] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [useSelectionOnly, setUseSelectionOnly] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const canSend = question.trim().length > 3 && (!useSelectionOnly || selectedText.trim().length > 0);

  const handleToggle = () => setIsOpen((prev) => !prev);

  const captureSelection = () => {
    const selection = window.getSelection()?.toString().trim();
    if (selection) {
      setSelectedText(selection);
      setUseSelectionOnly(true);
    }
  };

  const sendQuestion = async () => {
    if (!canSend || loading) return;
    setLoading(true);
    setError(null);
    const payload = {
      query: question.trim(),
      selected_text: useSelectionOnly ? selectedText.trim() : undefined,
    };
    setMessages((prev) => [...prev, {role: 'user', content: question}]);
    try {
      const response = await fetch(`${backendUrl}/api/qa`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload),
      });
      if (!response.ok) {
        throw new Error(`Backend error: ${response.status}`);
      }
      const data = await response.json();
      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant',
          content: data.answer,
          citations: data.citations,
        },
      ]);
      setQuestion('');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    } finally {
      setLoading(false);
    }
  };

  const CitationList: React.FC<{citations?: Citation[]}> = ({citations}) => {
    if (!citations?.length) return null;
    const origin = typeof window !== 'undefined' ? window.location.origin : '';
    return (
      <div className={styles.citations}>
        {citations.map((citation, index) => (
          <button
            key={`${citation.source_url}-${index}`}
            type="button"
            onClick={() => {
              if (origin && citation.source_url.startsWith(origin)) {
                window.location.href = citation.source_url;
              } else {
                window.open(citation.source_url, '_blank');
              }
            }}>
            [{index + 1}] {citation.heading}
          </button>
        ))}
      </div>
    );
  };

  return (
    <>
      <button className={styles.fab} type="button" onClick={handleToggle}>
        {isOpen ? '×' : 'Ask AI'}
      </button>
      {isOpen && (
        <div className={styles.panel} id="chatbot">
          <div className={styles.header}>
            <div>
              <strong>Physical AI Assistant</strong>
              <p>Ask the book or constrain answers to highlighted text.</p>
            </div>
            <button type="button" onClick={handleToggle}>
              ×
            </button>
          </div>
          <div className={styles.messages}>
            {messages.map((msg, idx) => (
              <div key={idx} className={msg.role === 'user' ? styles.userBubble : styles.assistantBubble}>
                <p>{msg.content}</p>
                {msg.role === 'assistant' && <CitationList citations={msg.citations} />}
              </div>
            ))}
            {messages.length === 0 && (
              <p className={styles.placeholder}>No questions yet. Highlight text and tap “capture selection”.</p>
            )}
          </div>
          <label className={styles.inputLabel}>
            Question
            <textarea
              value={question}
              onChange={(event) => setQuestion(event.target.value)}
              placeholder="e.g., Explain the perception-action loop."
              rows={2}
            />
          </label>
          <label className={styles.inputLabel}>
            Selected text (optional)
            <textarea
              value={selectedText}
              onChange={(event) => setSelectedText(event.target.value)}
              placeholder="Highlight text anywhere in the chapter, then click capture."
              rows={2}
            />
          </label>
          <div className={styles.controls}>
            <button type="button" onClick={captureSelection}>
              Capture selection
            </button>
            <label>
              <input
                type="checkbox"
                checked={useSelectionOnly}
                onChange={(event) => setUseSelectionOnly(event.target.checked)}
              />
              Use selected text only
            </label>
          </div>
          {error && <p className={styles.error}>{error}</p>}
          <button className={styles.sendButton} type="button" disabled={!canSend || loading} onClick={sendQuestion}>
            {loading ? 'Thinking…' : 'Ask'}
          </button>
        </div>
      )}
    </>
  );
};

export default ChatbotWidget;
