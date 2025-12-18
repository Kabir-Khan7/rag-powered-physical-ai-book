import React from 'react';
import type {Props} from '@theme/Root';
import ChatbotWidget from '../components/ChatbotWidget';

const Root = ({children}: Props) => (
  <>
    {children}
    <ChatbotWidget />
  </>
);

export default Root;
