import React, {createContext, useContext, useEffect, useMemo, useState} from 'react';

const backendUrl = process.env.BACKEND_URL ?? 'http://localhost:8000';

type Profile = {
  email: string;
  software_background: string;
  hardware_background: string;
  learning_preference: string;
};

type AuthContextValue = {
  token: string | null;
  profile: Profile | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (payload: {
    name: string;
    email: string;
    password: string;
    software_background: string;
    hardware_background: string;
    learning_preference: string;
  }) => Promise<void>;
  signOut: () => void;
};

const AuthContext = createContext<AuthContextValue | undefined>(undefined);

export const AuthProvider: React.FC<{children: React.ReactNode}> = ({children}) => {
  const [token, setToken] = useState<string | null>(null);
  const [profile, setProfile] = useState<Profile | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    const storedToken = window.localStorage.getItem('physical-ai-token');
    if (storedToken) {
      setToken(storedToken);
      fetchProfile(storedToken);
    } else {
      setLoading(false);
    }
  }, []);

  const fetchProfile = async (authToken: string) => {
    try {
      const response = await fetch(`${backendUrl}/api/profile`, {
        headers: {Authorization: `Bearer ${authToken}`},
      });
      if (response.ok) {
        const data = await response.json();
        setProfile(data);
      }
    } catch {
      // ignore
    } finally {
      setLoading(false);
    }
  };

  const signIn = async (email: string, password: string) => {
    const response = await fetch(`${backendUrl}/api/auth/signin`, {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({email, password}),
    });
    if (!response.ok) {
      throw new Error('Unable to sign in');
    }
    const data = await response.json();
    window.localStorage.setItem('physical-ai-token', data.token);
    setToken(data.token);
    await fetchProfile(data.token);
  };

  const signUp = async (payload: {
    name: string;
    email: string;
    password: string;
    software_background: string;
    hardware_background: string;
    learning_preference: string;
  }) => {
    const response = await fetch(`${backendUrl}/api/auth/signup`, {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload),
    });
    if (!response.ok) {
      throw new Error('Unable to sign up');
    }
    const data = await response.json();
    window.localStorage.setItem('physical-ai-token', data.token);
    setToken(data.token);
    await fetchProfile(data.token);
  };

  const signOut = () => {
    window.localStorage.removeItem('physical-ai-token');
    setToken(null);
    setProfile(null);
  };

  const value = useMemo(
    () => ({
      token,
      profile,
      loading,
      signIn,
      signUp,
      signOut,
    }),
    [token, profile, loading],
  );

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = () => {
  const ctx = useContext(AuthContext);
  if (!ctx) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return ctx;
};
