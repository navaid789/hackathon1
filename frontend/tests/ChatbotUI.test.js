// frontend/tests/ChatbotUI.test.js
// Unit tests for the Chatbot UI component

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatbotUI from '../src/components/Chatbot/ChatbotUI';
import { apiClient } from '../src/services/apiClient';
import { textSelectionService } from '../src/services/textSelection';
import { stateManager } from '../src/services/stateManager';

// Mock the services
jest.mock('../src/services/apiClient', () => ({
  apiClient: {
    query: jest.fn()
  }
}));

jest.mock('../src/services/textSelection', () => ({
  textSelectionService: {
    addSelectionListener: jest.fn(),
    removeSelectionListener: jest.fn()
  }
}));

jest.mock('../src/services/stateManager', () => ({
  stateManager: {
    addListener: jest.fn(),
    removeListener: jest.fn(),
    getState: jest.fn(),
    setQueryState: jest.fn(),
    resetQueryState: jest.fn()
  }
}));

describe('ChatbotUI Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();

    // Mock default state
    stateManager.getState.mockReturnValue({
      query: {
        status: 'IDLE',
        loading: false,
        error: null,
        data: null,
        timestamp: null
      },
      connection: {
        status: 'CONNECTED',
        connected: true,
        lastError: null,
        lastChecked: null
      },
      apiCalls: new Map()
    });
  });

  test('renders without crashing', () => {
    render(<ChatbotUI />);
    expect(screen.getByLabelText(/Show chatbot/i)).toBeInTheDocument();
  });

  test('toggles chatbot visibility when toggle button is clicked', () => {
    render(<ChatbotUI />);

    const toggleButton = screen.getByLabelText(/Show chatbot/i);
    fireEvent.click(toggleButton);

    expect(screen.getByText('Book Assistant')).toBeInTheDocument();
  });

  test('submits query when form is submitted', async () => {
    const mockQueryResponse = {
      response: 'This is a test response',
      sources: [],
      thread_id: 'test-thread-id',
      query_id: 'test-query-id',
      timestamp: new Date().toISOString()
    };

    apiClient.query.mockResolvedValue(mockQueryResponse);
    stateManager.getState.mockReturnValue({
      query: {
        status: 'SUCCESS',
        loading: false,
        error: null,
        data: mockQueryResponse,
        timestamp: new Date().toISOString()
      },
      connection: {
        status: 'CONNECTED',
        connected: true,
        lastError: null,
        lastChecked: null
      },
      apiCalls: new Map()
    });

    render(<ChatbotUI />);

    // Show the chatbot
    const toggleButton = screen.getByLabelText(/Show chatbot/i);
    fireEvent.click(toggleButton);

    // Fill in the query
    const queryInput = screen.getByPlaceholderText(/Ask a question about the book/i);
    fireEvent.change(queryInput, { target: { value: 'Test query' } });

    // Submit the form
    const submitButton = screen.getByText('Send');
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(apiClient.query).toHaveBeenCalledWith('Test query', null, null);
    });
  });

  test('shows error when query is empty', async () => {
    render(<ChatbotUI />);

    // Show the chatbot
    const toggleButton = screen.getByLabelText(/Show chatbot/i);
    fireEvent.click(toggleButton);

    // Submit the form with empty query
    const submitButton = screen.getByText('Send');
    fireEvent.click(submitButton);

    expect(screen.getByText(/Query cannot be empty/i)).toBeInTheDocument();
  });

  test('displays selected text when available', () => {
    // Mock the text selection service to return selected text
    const mockAddListener = jest.fn((callback) => {
      // Immediately call the callback with selected text to simulate selection
      callback({
        text: 'This is the selected text',
        context: null,
        position: null
      });
    });

    textSelectionService.addSelectionListener.mockImplementation(mockAddListener);

    render(<ChatbotUI />);

    // Show the chatbot
    const toggleButton = screen.getByLabelText(/Show chatbot/i);
    fireEvent.click(toggleButton);

    expect(screen.getByText(/Selected text:/i)).toBeInTheDocument();
    expect(screen.getByText(/This is the selected text/i)).toBeInTheDocument();
  });

  test('clears chat history when clear button is clicked', () => {
    render(<ChatbotUI />);

    // Show the chatbot
    const toggleButton = screen.getByLabelText(/Show chatbot/i);
    fireEvent.click(toggleButton);

    // Click the clear button
    const clearButton = screen.getByText('Clear');
    fireEvent.click(clearButton);

    expect(stateManager.resetQueryState).toHaveBeenCalled();
  });

  test('shows loading state when query is in progress', async () => {
    // Mock a delayed response to show loading state
    const mockQueryPromise = new Promise((resolve) => {
      setTimeout(() => resolve({
        response: 'Delayed response',
        sources: [],
        thread_id: 'test-thread-id',
        query_id: 'test-query-id',
        timestamp: new Date().toISOString()
      }), 100);
    });

    apiClient.query.mockReturnValue(mockQueryPromise);

    render(<ChatbotUI />);

    // Show the chatbot
    const toggleButton = screen.getByLabelText(/Show chatbot/i);
    fireEvent.click(toggleButton);

    // Fill in the query
    const queryInput = screen.getByPlaceholderText(/Ask a question about the book/i);
    fireEvent.change(queryInput, { target: { value: 'Test query' } });

    // Submit the form
    const submitButton = screen.getByText('Send');
    fireEvent.click(submitButton);

    // Check that loading state is shown
    expect(submitButton).toHaveTextContent('Sending...');
  });

  test('handles API errors gracefully', async () => {
    const mockError = new Error('API Error');
    apiClient.query.mockRejectedValue(mockError);

    render(<ChatbotUI />);

    // Show the chatbot
    const toggleButton = screen.getByLabelText(/Show chatbot/i);
    fireEvent.click(toggleButton);

    // Fill in the query
    const queryInput = screen.getByPlaceholderText(/Ask a question about the book/i);
    fireEvent.change(queryInput, { target: { value: 'Test query' } });

    // Submit the form
    const submitButton = screen.getByText('Send');
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(screen.getByText(/An error occurred while processing your query/i)).toBeInTheDocument();
    });
  });
});