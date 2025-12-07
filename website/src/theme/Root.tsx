import React from 'react';
import RAGChatbot from '../components/RAGChatbot';

export default function Root({children}: {children: React.ReactNode}): React.ReactElement {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
}
