import React, {useState} from 'react';
import {useAuth} from '../../context/AuthContext';
import styles from './styles.module.css';

type Mode = 'signin' | 'signup';

const AuthPanel: React.FC = () => {
  const {token, profile, loading, signIn, signUp, signOut} = useAuth();
  const [mode, setMode] = useState<Mode>('signin');
  const [error, setError] = useState<string | null>(null);
  const [form, setForm] = useState({
    name: '',
    email: '',
    password: '',
    software_background: '',
    hardware_background: '',
    learning_preference: '',
  });

  const handleChange = (event: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const {name, value} = event.target;
    setForm((prev) => ({...prev, [name]: value}));
  };

  const handleSubmit = async () => {
    try {
      setError(null);
      if (mode === 'signin') {
        await signIn(form.email, form.password);
      } else {
        await signUp({
          name: form.name,
          email: form.email,
          password: form.password,
          software_background: form.software_background,
          hardware_background: form.hardware_background,
          learning_preference: form.learning_preference,
        });
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Auth error');
    }
  };

  if (loading) return null;

  if (token && profile) {
    return (
      <div className={styles.badge}>
        <div>
          <strong>{profile.email}</strong>
          <span>{profile.learning_preference} learner</span>
        </div>
        <button type="button" onClick={signOut}>
          Sign out
        </button>
      </div>
    );
  }

  return (
    <div className={styles.panel}>
      <div className={styles.tabs}>
        <button type="button" className={mode === 'signin' ? styles.active : ''} onClick={() => setMode('signin')}>
          Sign in
        </button>
        <button type="button" className={mode === 'signup' ? styles.active : ''} onClick={() => setMode('signup')}>
          Sign up
        </button>
      </div>
      {mode === 'signup' && (
        <input name="name" placeholder="Full name" value={form.name} onChange={handleChange} required />
      )}
      <input name="email" placeholder="Email" value={form.email} onChange={handleChange} required />
      <input
        type="password"
        name="password"
        placeholder="Password"
        value={form.password}
        onChange={handleChange}
        required
      />
      {mode === 'signup' && (
        <>
          <input
            name="software_background"
            placeholder="Software background"
            value={form.software_background}
            onChange={handleChange}
          />
          <input
            name="hardware_background"
            placeholder="Hardware background"
            value={form.hardware_background}
            onChange={handleChange}
          />
          <select name="learning_preference" value={form.learning_preference} onChange={handleChange}>
            <option value="">Learning preference</option>
            <option value="visual">Visual</option>
            <option value="hands-on">Hands-on</option>
            <option value="theoretical">Theoretical</option>
          </select>
        </>
      )}
      {error && <p className={styles.error}>{error}</p>}
      <button type="button" onClick={handleSubmit}>
        {mode === 'signin' ? 'Sign in' : 'Create account'}
      </button>
    </div>
  );
};

export default AuthPanel;
