// frontend/src/components/Chatbot/ChatbotUI.jsx
// Main chatbot UI component that captures selected text and query input

import React, { useState, useEffect, useRef } from 'react';
import { apiClient } from '../../services/apiClient';
import { textSelectionService } from '../../services/textSelection';
import { stateManager } from '../../services/stateManager';
import { connectionManager } from '../../services/connectionManager';
import { QueryRequest } from '../../models/QueryRequest';
import './ChatbotUI.css';

const ChatbotUI = ({ initialThreadId = null }) => {
  const [query, setQuery] = useState('');
  const [threadId, setThreadId] = useState(initialThreadId);
  const [selectedText, setSelectedText] = useState('');
  const [context, setContext] = useState(null);
  const [responses, setResponses] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [showChatbot, setShowChatbot] = useState(false);
  const [isMinimized, setIsMinimized] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('DISCONNECTED');

  const chatContainerRef = useRef(null);
  const inputRef = useRef(null);

  // Handle text selection changes
  useEffect(() => {
    const handleSelectionChange = (selectionInfo) => {
      if (selectionInfo && selectionInfo.text) {
        setSelectedText(selectionInfo.text);
        setContext(selectionInfo.context || null);
      } else {
        setSelectedText('');
        setContext(null);
      }
    };

    textSelectionService.addSelectionListener(handleSelectionChange);

    // Cleanup on unmount
    return () => {
      textSelectionService.removeSelectionListener(handleSelectionChange);
    };
  }, []);

  // Subscribe to state changes
  useEffect(() => {
    const handleStateChange = (newState) => {
      setIsLoading(newState.query.loading);
      if (newState.query.error) {
        setError(newState.query.error);
      }

      // Update connection status
      if (newState.connection) {
        setConnectionStatus(newState.connection.status);
      }
    };

    stateManager.addListener(handleStateChange);

    // Initialize with current state
    handleStateChange(stateManager.getState());

    return () => {
      stateManager.removeListener(handleStateChange);
    };
  }, []);

  // Initialize connection manager
  useEffect(() => {
    connectionManager.initialize();

    // Cleanup on unmount
    return () => {
      connectionManager.cleanup();
    };
  }, []);

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!query.trim()) {
      setError('Query cannot be empty');
      return;
    }

    try {
      setError(null);
      setIsLoading(true);

      // Create query request with validation
      const queryRequest = new QueryRequest({
        query: query.trim(),
        context: selectedText || null,
        thread_id: threadId || null
      });

      // Update state to loading
      stateManager.setQueryState('LOADING');

      // Make API call using new chat endpoint
      const response = await apiClient.chat(
        queryRequest.query,
        queryRequest.context,
        threadId || null
      );

      // Update thread ID if it was returned (in new format, it might be in user_id)
      // For now, we'll keep the original threadId functionality for backward compatibility

      // Add response to history
      const newResponse = {
        id: Date.now(),
        query: queryRequest.query,
        response: response.response,
        sources: response.sources || [],
        timestamp: new Date().toISOString(),
        threadId: threadId || null,
        grounded: response.grounded || false,
        confidence: response.confidence || null
      };

      setResponses(prev => [...prev, newResponse]);

      // Update state to success
      stateManager.setQueryState('SUCCESS', newResponse);

      // Clear input
      setQuery('');

    } catch (err) {
      console.error('Query error:', err);
      setError(err.message || 'An error occurred while processing your query');
      stateManager.setQueryState('ERROR', null, err.message || 'Query failed');
    } finally {
      setIsLoading(false);
    }
  };

  // Toggle chatbot visibility
  const toggleChatbot = () => {
    setShowChatbot(!showChatbot);
    if (!showChatbot && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  // Toggle minimized state
  const toggleMinimized = () => {
    setIsMinimized(!isMinimized);
  };

  // Clear chat history
  const clearChat = () => {
    setResponses([]);
    setError(null);
    stateManager.resetQueryState();
  };

  // Get connection status display text and color
  const getConnectionStatusDisplay = () => {
    switch (connectionStatus) {
      case 'CONNECTED':
        return { text: 'Connected', color: 'green', icon: '●' };
      case 'CONNECTING':
        return { text: 'Connecting', color: 'orange', icon: '↻' };
      case 'FAILED':
        return { text: 'Disconnected', color: 'red', icon: '●' };
      case 'DISCONNECTED':
        return { text: 'Disconnected', color: 'red', icon: '●' };
      default:
        return { text: 'Unknown', color: 'gray', icon: '?' };
    }
  };

  const statusDisplay = getConnectionStatusDisplay();

  return (
    <div className={`chatbot-container ${showChatbot ? 'visible' : 'hidden'}`}>
      <button
        className="chatbot-toggle-btn"
        onClick={toggleChatbot}
        aria-label={showChatbot ? "Hide chatbot" : "Show chatbot"}
      >
        {showChatbot ? '×' : '💬'}
      </button>

      {showChatbot && (
        <div className={`chatbot-widget ${isMinimized ? 'minimized' : ''}`} ref={chatContainerRef}>
          <div className="chatbot-header">
            <div className="chatbot-title">Book Assistant</div>
            <div className="chatbot-controls">
              <div className="connection-status" title={`Status: ${statusDisplay.text}`}>
                <span className="status-indicator" style={{ color: statusDisplay.color }}>
                  {statusDisplay.icon}
                </span>
                <span className="status-text">{statusDisplay.text}</span>
              </div>
              <button
                className="minimize-btn"
                onClick={toggleMinimized}
                aria-label={isMinimized ? "Expand chat" : "Minimize chat"}
              >
                {isMinimized ? '+' : '−'}
              </button>
              <button
                className="close-btn"
                onClick={toggleChatbot}
                aria-label="Close chat"
              >
                ×
              </button>
            </div>
          </div>

          {!isMinimized && (
            <div className="chatbot-content">
              {selectedText && (
                <div className="selected-text-preview">
                  <strong>Selected text:</strong>
                  <p>"{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</p>
                </div>
              )}

              <div className="chat-history">
                {responses.map((item) => (
                  <div key={item.id} className="chat-item">
                    <div className="user-query">
                      <strong>You:</strong> {item.query}
                    </div>
                    <div className="ai-response">
                      <strong>Assistant:</strong> {item.response}
                      {item.sources && item.sources.length > 0 && (
                        <div className="sources">
                          <details>
                            <summary>Sources</summary>
                            <ul>
                              {item.sources.map((source, idx) => (
                                <li key={idx}>
                                  {source} {/* New format returns source strings instead of objects */}
                                </li>
                              ))}
                            </ul>
                          </details>
                        </div>
                      )}
                      {item.confidence && (
                        <div className="confidence">
                          Confidence: {(item.confidence * 100).toFixed(1)}%
                        </div>
                      )}
                    </div>
                  </div>
                ))}
              </div>

              {error && (
                <div className="error-message">
                  <strong>Error:</strong> {error}
                </div>
              )}

              <form onSubmit={handleSubmit} className="query-form">
                <textarea
                  ref={inputRef}
                  value={query}
                  onChange={(e) => setQuery(e.target.value)}
                  placeholder={selectedText
                    ? "Ask about the selected text..."
                    : "Ask a question about the book..."
                  }
                  disabled={isLoading}
                  rows={3}
                  className="query-input"
                />
                <div className="form-controls">
                  <button
                    type="submit"
                    disabled={isLoading || !query.trim()}
                    className="submit-btn"
                  >
                    {isLoading ? 'Sending...' : 'Send'}
                  </button>
                  <button
                    type="button"
                    onClick={clearChat}
                    className="clear-btn"
                    disabled={responses.length === 0}
                  >
                    Clear
                  </button>
                </div>
              </form>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default ChatbotUI;