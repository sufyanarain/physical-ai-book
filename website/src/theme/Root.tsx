import React from 'react';
import RAGChatbot from '@site/src/components/RAGChatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
}
