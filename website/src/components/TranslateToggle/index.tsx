import React, {useState} from 'react';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import styles from './styles.module.css';

const backendUrl = process.env.BACKEND_URL ?? 'http://localhost:8000';

const TranslateToggle: React.FC = () => {
  const {metadata} = useDoc();
  const [mode, setMode] = useState<'english' | 'urdu'>('english');
  const [translation, setTranslation] = useState<string>('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const toggleUrdu = async () => {
    if (mode === 'urdu') {
      setMode('english');
      return;
    }
    if (translation) {
      setMode('urdu');
      return;
    }
    try {
      setLoading(true);
      setError(null);
      const response = await fetch(`${backendUrl}/api/translate`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({chapter_id: metadata.id}),
      });
      if (!response.ok) {
        throw new Error('Translation failed');
      }
      const data = await response.json();
      setTranslation(data.urdu_text);
      setMode('urdu');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unexpected error');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <button type="button" onClick={toggleUrdu} disabled={loading}>
        {loading ? 'Translating…' : mode === 'english' ? 'Translate to Urdu' : 'Show English'}
      </button>
      {error && <p className={styles.error}>{error}</p>}
      {mode === 'urdu' && translation && (
        <div className={styles.translation}>
          <h4>Urdu Translation</h4>
          <p>{translation}</p>
        </div>
      )}
    </div>
  );
};

export default TranslateToggle;
