import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';

// Root component is rendered once for the entire app
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}