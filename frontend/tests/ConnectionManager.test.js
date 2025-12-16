// frontend/tests/ConnectionManager.test.js
// Integration tests for the connection management functionality

import { ConnectionManager } from '../src/services/connectionManager';
import { stateManager } from '../src/services/stateManager';

// Mock the fetch API
global.fetch = jest.fn();

describe('ConnectionManager Integration Tests', () => {
  let connectionManager;

  beforeEach(() => {
    jest.clearAllMocks();

    // Create a new instance for each test
    connectionManager = new ConnectionManager({
      primaryUrl: 'http://localhost:8000',
      fallbackUrls: ['http://0.0.0.0:8000'],
      timeout: 5000,
      retries: 2
    });
  });

  afterEach(() => {
    connectionManager.cleanup();
  });

  test('should successfully make a request when server is available', async () => {
    const mockResponse = {
      response: 'Test response',
      sources: [],
      thread_id: 'test-thread',
      query_id: 'test-query',
      timestamp: new Date().toISOString()
    };

    fetch.mockResolvedValueOnce({
      ok: true,
      json: () => Promise.resolve(mockResponse)
    });

    const result = await connectionManager.queryRequest('Test query');

    expect(fetch).toHaveBeenCalledWith(
      'http://localhost:8000/query',
      expect.objectContaining({
        method: 'POST',
        headers: expect.objectContaining({
          'Content-Type': 'application/json'
        })
      })
    );
    expect(result).toEqual(mockResponse);
    expect(connectionManager.connectionStatus).toBe('CONNECTED');
  });

  test('should retry on server error and succeed on subsequent attempt', async () => {
    const mockResponse = {
      response: 'Test response after retry',
      sources: [],
      thread_id: 'test-thread',
      query_id: 'test-query',
      timestamp: new Date().toISOString()
    };

    // First call fails, second succeeds
    fetch
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'
      })
      .mockResolvedValueOnce({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

    const result = await connectionManager.queryRequest('Test query');

    expect(fetch).toHaveBeenCalledTimes(2);
    expect(result).toEqual(mockResponse);
  });

  test('should try fallback URLs when primary fails', async () => {
    const mockResponse = {
      response: 'Response from fallback',
      sources: [],
      thread_id: 'test-thread',
      query_id: 'test-query',
      timestamp: new Date().toISOString()
    };

    // Primary fails, fallback succeeds
    fetch
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'
      })
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'
      })
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'
      })
      .mockResolvedValueOnce({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

    const result = await connectionManager.queryRequest('Test query');

    // Should have tried primary URL 3 times (original + 2 retries), then fallback 3 times
    expect(fetch).toHaveBeenCalledTimes(6);
    // The last call should be to the fallback URL
    expect(fetch).toHaveBeenLastCalledWith(
      'http://0.0.0.0:8000/query',
      expect.objectContaining({
        method: 'POST'
      })
    );
    expect(result).toEqual(mockResponse);
  });

  test('should handle timeout errors', async () => {
    // Simulate timeout by having fetch reject with a timeout-like error
    fetch.mockImplementation(() => new Promise((_, reject) => {
      setTimeout(() => reject(new Error('Failed to fetch')), 100);
    }));

    // Set a short timeout for testing
    connectionManager.config.timeout = 50;

    await expect(connectionManager.queryRequest('Test query'))
      .rejects
      .toThrow('Failed to fetch');

    expect(connectionManager.connectionStatus).toBe('FAILED');
  });

  test('should validate connection successfully', async () => {
    fetch.mockResolvedValueOnce({
      ok: true,
      json: () => Promise.resolve({ status: 'healthy', timestamp: new Date().toISOString() })
    });

    const result = await connectionManager.validateConnection();

    expect(fetch).toHaveBeenCalledWith(
      'http://localhost:8000/health',
      expect.objectContaining({
        method: 'GET'
      })
    );
    expect(result.connected).toBe(true);
    expect(connectionManager.connectionStatus).toBe('CONNECTED');
  });

  test('should handle connection validation failure', async () => {
    fetch.mockResolvedValueOnce({
      ok: false,
      status: 404,
      statusText: 'Not Found'
    });

    const result = await connectionManager.validateConnection();

    expect(result.connected).toBe(false);
    expect(connectionManager.connectionStatus).toBe('FAILED');
  });

  test('should update state manager on connection status changes', async () => {
    const stateChangeListener = jest.fn();
    stateManager.addListener(stateChangeListener);

    fetch.mockResolvedValueOnce({
      ok: true,
      json: () => Promise.resolve({
        response: 'Test response',
        sources: [],
        thread_id: 'test-thread',
        query_id: 'test-query',
        timestamp: new Date().toISOString()
      })
    });

    await connectionManager.queryRequest('Test query');

    // Verify that state manager was notified of connection status change
    expect(stateManager.setConnectionState).toHaveBeenCalledWith('CONNECTED');

    stateManager.removeListener(stateChangeListener);
  });

  test('should switch to next URL on failure', async () => {
    const mockResponse = {
      response: 'Response from switched URL',
      sources: [],
      thread_id: 'test-thread',
      query_id: 'test-query',
      timestamp: new Date().toISOString()
    };

    // Mock primary URL failure and fallback success
    fetch
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Server Error'
      })
      .mockResolvedValueOnce({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

    const initialUrl = connectionManager.getCurrentUrl();
    const result = await connectionManager.queryRequest('Test query');
    const finalUrl = connectionManager.getCurrentUrl();

    // Should have switched to the fallback URL
    expect(initialUrl).toBe('http://localhost:8000');
    expect(finalUrl).toBe('http://0.0.0.0:8000');
    expect(result).toEqual(mockResponse);
  });

  test('should cancel operations when cleanup is called', async () => {
    // This test verifies that cleanup methods work correctly
    const initialIntervalId = connectionManager.healthCheckIntervalId;

    connectionManager.startHealthChecks();
    expect(connectionManager.healthCheckIntervalId).toBeDefined();

    connectionManager.cleanup();
    expect(connectionManager.healthCheckIntervalId).toBeNull();
    expect(connectionManager.connectionStatus).toBe('DISCONNECTED');
  });
});