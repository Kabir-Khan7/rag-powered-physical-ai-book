import React, {useState} from 'react';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import {useAuth} from '../../context/AuthContext';
import styles from './styles.module.css';

const backendUrl = process.env.BACKEND_URL ?? 'http://localhost:8000';

const PersonalizeButton: React.FC = () => {
  const {metadata} = useDoc();
  const {token, profile} = useAuth();
  const [overlay, setOverlay] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  if (!token || !profile) return null;

  const requestPersonalization = async () => {
    try {
      setLoading(true);
      setError(null);
      const response = await fetch(`${backendUrl}/api/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_id: metadata.id,
          difficulty: metadata.frontMatter?.difficulty ?? 'intermediate',
          hardware_focus: profile.hardware_background,
          learning_preference: profile.learning_preference,
        }),
      });
      if (!response.ok) {
        throw new Error('Unable to personalize');
      }
      const data = await response.json();
      setOverlay(data.overlay);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unexpected error');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <button type="button" onClick={requestPersonalization} disabled={loading}>
        {loading ? 'Personalizing…' : 'Personalize this chapter'}
      </button>
      {error && <p className={styles.error}>{error}</p>}
      {overlay && (
        <div className={styles.overlay}>
          <h4>Personalized overlay</h4>
          <p>{overlay}</p>
        </div>
      )}
    </div>
  );
};

export default PersonalizeButton;
