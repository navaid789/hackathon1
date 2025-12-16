import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatbotUI from '@site/frontend/src/components/Chatbot/ChatbotUI';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        <ChatbotUI />
      </OriginalLayout>
    </>
  );
}