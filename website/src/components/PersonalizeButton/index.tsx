import React, {useState} from 'react';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import styles from './styles.module.css';

const backendUrl = process.env.BACKEND_URL ?? 'http://localhost:8000';

const PersonalizeButton: React.FC = () => {
  const {metadata} = useDoc();
  const [overlay, setOverlay] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [form, setForm] = useState({
    hardwareFocus: '',
    learningPreference: '',
    softwareBackground: '',
    hardwareBackground: '',
    learningGoal: '',
  });

  const handleChange = (event: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const {name, value} = event.target;
    setForm((prev) => ({...prev, [name]: value}));
  };

  const requestPersonalization = async () => {
    try {
      setLoading(true);
      setError(null);
      const response = await fetch(`${backendUrl}/api/personalize`, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
          chapter_id: metadata.id,
          difficulty: metadata.frontMatter?.difficulty ?? 'intermediate',
          hardware_focus: form.hardwareFocus || undefined,
          learning_preference: form.learningPreference || undefined,
          software_background: form.softwareBackground || undefined,
          hardware_background: form.hardwareBackground || undefined,
          learning_goal: form.learningGoal || undefined,
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
      <div className={styles.formGrid}>
        <input
          name="softwareBackground"
          placeholder="Software background (optional)"
          value={form.softwareBackground}
          onChange={handleChange}
        />
        <input
          name="hardwareBackground"
          placeholder="Hardware background (optional)"
          value={form.hardwareBackground}
          onChange={handleChange}
        />
        <input name="hardwareFocus" placeholder="Hardware focus" value={form.hardwareFocus} onChange={handleChange} />
        <select name="learningPreference" value={form.learningPreference} onChange={handleChange}>
          <option value="">Learning preference</option>
          <option value="visual">Visual</option>
          <option value="hands-on">Hands-on</option>
          <option value="theoretical">Theoretical</option>
        </select>
        <input name="learningGoal" placeholder="Learning goal" value={form.learningGoal} onChange={handleChange} />
      </div>
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
