import React from 'react';
import type {Props} from '@theme/Root';
import ChatbotWidget from '../components/ChatbotWidget';
import AuthPanel from '../components/AuthPanel';
import {AuthProvider} from '../context/AuthContext';

const Root = ({children}: Props) => (
  <AuthProvider>
    {children}
    <AuthPanel />
    <ChatbotWidget />
  </AuthProvider>
);

export default Root;
