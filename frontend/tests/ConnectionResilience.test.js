// frontend/tests/ConnectionResilience.test.js
// Tests for connection resilience under various failure conditions

import { ConnectionManager } from '../src/services/connectionManager';

// Mock the fetch API
global.fetch = jest.fn();

describe('Connection Resilience Tests', () => {
  let connectionManager;

  beforeEach(() => {
    jest.clearAllMocks();

    // Create a new instance for each test with reduced retry settings for faster tests
    connectionManager = new ConnectionManager({
      primaryUrl: 'http://localhost:8000',
      fallbackUrls: ['http://0.0.0.0:8000', 'http://127.0.0.1:8000'],
      timeout: 2000,
      retries: 1,
      retryDelay: 10, // Fast retries for tests
      maxRetryDelay: 50
    });
  });

  afterEach(() => {
    connectionManager.cleanup();
  });

  test('should handle network timeout gracefully', async () => {
    // Simulate a timeout by having fetch reject with a network error
    fetch.mockRejectedValue(new Error('Network timeout'));

    await expect(connectionManager.queryRequest('Test query'))
      .rejects
      .toThrow('Network timeout');

    expect(connectionManager.connectionStatus).toBe('FAILED');
  });

  test('should handle server timeout (504 Gateway Timeout)', async () => {
    fetch.mockResolvedValue({
      ok: false,
      status: 504,
      statusText: 'Gateway Timeout'
    });

    await expect(connectionManager.queryRequest('Test query'))
      .rejects
      .toThrow('Server error: 504 - Gateway Timeout');

    expect(connectionManager.connectionStatus).toBe('FAILED');
  });

  test('should handle service unavailable (503) with retry', async () => {
    const mockResponse = {
      response: 'Success after 503 recovery',
      sources: [],
      thread_id: 'test-thread',
      query_id: 'test-query',
      timestamp: new Date().toISOString()
    };

    // First call returns 503, second call succeeds
    fetch
      .mockResolvedValueOnce({
        ok: false,
        status: 503,
        statusText: 'Service Unavailable'
      })
      .mockResolvedValueOnce({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

    const result = await connectionManager.queryRequest('Test query');

    expect(fetch).toHaveBeenCalledTimes(2);
    expect(result).toEqual(mockResponse);
    expect(connectionManager.connectionStatus).toBe('CONNECTED');
  });

  test('should handle bad gateway (502) error', async () => {
    fetch.mockResolvedValue({
      ok: false,
      status: 502,
      statusText: 'Bad Gateway'
    });

    await expect(connectionManager.queryRequest('Test query'))
      .rejects
      .toThrow('Server error: 502 - Bad Gateway');

    expect(connectionManager.connectionStatus).toBe('FAILED');
  });

  test('should fail completely when all fallback URLs are unavailable', async () => {
    // Mock all URLs to fail
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
        statusText: 'Internal Server Error'  // This would be on fallback URL
      })
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'
      })
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'  // Second try on fallback URL
      })
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'  // Third URL (second fallback)
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
      });

    await expect(connectionManager.queryRequest('Test query'))
      .rejects
      .toThrow('HTTP 500: Internal Server Error');

    expect(fetch).toHaveBeenCalledTimes(8); // 2 attempts × (1 primary + 3 URLs) = 8 calls
    expect(connectionManager.connectionStatus).toBe('FAILED');
  });

  test('should handle DNS resolution failure', async () => {
    fetch.mockRejectedValue(new TypeError('Failed to fetch')); // Common error for DNS issues

    await expect(connectionManager.queryRequest('Test query'))
      .rejects
      .toThrow('Failed to fetch');

    expect(connectionManager.connectionStatus).toBe('FAILED');
  });

  test('should handle CORS preflight failure', async () => {
    fetch.mockRejectedValue(new TypeError('NetworkError when attempting to fetch resource'));

    await expect(connectionManager.queryRequest('Test query'))
      .rejects
      .toThrow('NetworkError when attempting to fetch resource');

    expect(connectionManager.connectionStatus).toBe('FAILED');
  });

  test('should recover after temporary server unavailability', async () => {
    const mockResponse = {
      response: 'Recovered after temporary failure',
      sources: [],
      thread_id: 'test-thread',
      query_id: 'test-query',
      timestamp: new Date().toISOString()
    };

    // Simulate a scenario where the server is temporarily down, then recovers
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
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

    // First call will try primary URL twice (1 attempt + 1 retry) and fail
    await expect(connectionManager.queryRequest('Test query'))
      .rejects
      .toThrow('HTTP 500: Internal Server Error');

    // Now make the server available and try again
    fetch
      .mockResolvedValueOnce({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

    // Reset connection status to try again
    connectionManager.setConnectionStatus('CONNECTED');
    const result = await connectionManager.queryRequest('Test query');

    expect(result).toEqual(mockResponse);
    expect(connectionManager.connectionStatus).toBe('CONNECTED');
  });

  test('should maintain connection status during health checks', async () => {
    // Mock health check responses
    fetch
      .mockResolvedValueOnce({
        ok: true,
        json: () => Promise.resolve({ status: 'healthy' })
      })
      .mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Server Error'
      })
      .mockResolvedValueOnce({
        ok: true,
        json: () => Promise.resolve({ status: 'healthy' })
      });

    // Test initial connection validation
    const result1 = await connectionManager.validateConnection();
    expect(result1.connected).toBe(true);

    // Test failure scenario
    const result2 = await connectionManager.validateConnection();
    expect(result2.connected).toBe(false);

    // Test recovery
    const result3 = await connectionManager.validateConnection();
    expect(result3.connected).toBe(true);
  });

  test('should handle high load scenarios gracefully', async () => {
    // Simulate high load with slow responses
    jest.useFakeTimers();
    fetch.mockImplementation(() => {
      return new Promise((resolve) => {
        setTimeout(() => {
          resolve({
            ok: true,
            json: () => Promise.resolve({
              response: 'Response under high load',
              sources: [],
              thread_id: 'test-thread',
              query_id: 'test-query',
              timestamp: new Date().toISOString()
            })
          });
        }, 100); // Simulate slow response
      });
    });

    const result = await connectionManager.queryRequest('Test query');

    expect(result.response).toBe('Response under high load');
    expect(connectionManager.connectionStatus).toBe('CONNECTED');

    jest.useRealTimers();
  });

  test('should handle multiple concurrent requests during failure', async () => {
    // Mock responses for multiple concurrent requests
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
      });

    // Make multiple concurrent requests
    const promises = [
      connectionManager.queryRequest('Query 1'),
      connectionManager.queryRequest('Query 2'),
      connectionManager.queryRequest('Query 3')
    ];

    const results = await Promise.allSettled(promises);

    // All should be rejected due to server error
    results.forEach(result => {
      expect(result.status).toBe('rejected');
    });

    expect(connectionManager.connectionStatus).toBe('FAILED');
  });
});