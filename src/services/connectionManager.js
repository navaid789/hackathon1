// frontend/src/services/connectionManager.js
// Service for managing connection configuration with fallback and retry mechanisms

import { stateManager } from './stateManager';
import { connectionMetricsService } from './connectionMetrics';

export class ConnectionManager {
  constructor(config = {}) {
    // Default configuration
    this.config = {
      primaryUrl: config.primaryUrl || 'http://127.0.0.1:8001',
      fallbackUrls: config.fallbackUrls || [
        'http://localhost:8001',
        'http://0.0.0.0:8001'
      ],
      timeout: config.timeout || 30000,
      retries: config.retries || 3,
      retryDelay: config.retryDelay || 1000, // Initial delay in ms
      maxRetryDelay: config.maxRetryDelay || 10000, // Max delay in ms
      exponentialBackoff: config.exponentialBackoff !== false, // Default to true
      healthCheckInterval: config.healthCheckInterval || 30000, // 30 seconds
      validateConnectionOnStartup: config.validateConnectionOnStartup !== false
    };

    this.currentUrlIndex = 0;
    this.currentUrl = this.config.primaryUrl;
    this.connectionStatus = 'DISCONNECTED';
    this.healthCheckIntervalId = null;
    this.pendingRequests = new Map();
    this.retryTimers = new Set();
  }

  // Get current active URL
  getCurrentUrl() {
    return this.currentUrl;
  }

  // Get all available URLs
  getAvailableUrls() {
    return [this.config.primaryUrl, ...this.config.fallbackUrls];
  }

  // Set connection status and notify state manager
  setConnectionStatus(status, error = null) {
    this.connectionStatus = status;
    stateManager.setConnectionState(status, error);
  }

  // Validate connection by making a test request
  async validateConnection() {
    try {
      // Use fetch to test connectivity without making a full API call
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

      const response = await fetch(`${this.currentUrl}/health`, {
        method: 'GET',
        signal: controller.signal,
        headers: {
          'Accept': 'application/json'
        }
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        this.setConnectionStatus('CONNECTED');
        return { connected: true, url: this.currentUrl, response: await response.json() };
      } else {
        throw new Error(`Health check failed with status: ${response.status}`);
      }
    } catch (error) {
      this.setConnectionStatus('FAILED', error.message);
      return { connected: false, url: this.currentUrl, error: error.message };
    }
  }

  // Try to connect to the next available URL in the list
  async switchToNextUrl() {
    const urls = this.getAvailableUrls();
    this.currentUrlIndex = (this.currentUrlIndex + 1) % urls.length;
    this.currentUrl = urls[this.currentUrlIndex];

    const result = await this.validateConnection();
    return result;
  }

  // Make a request with retry and fallback logic
  async makeRequest(endpoint, options = {}) {
    const requestId = this.generateRequestId();
    const startTime = Date.now();

    // Add to pending requests
    this.pendingRequests.set(requestId, { startTime, endpoint, options });

    try {
      // Try each URL with retries
      let lastError = null;

      // First try the current URL with retries
      for (let attempt = 0; attempt < this.config.retries; attempt++) {
        try {
          const controller = new AbortController();
          const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

          const response = await fetch(`${this.currentUrl}${endpoint}`, {
            ...options,
            headers: {
              'Content-Type': 'application/json',
              ...options.headers,
            },
            signal: controller.signal,
          });

          clearTimeout(timeoutId);

          if (response.ok) {
            this.setConnectionStatus('CONNECTED');
            const result = await response.json();

            // Record successful request
            const responseTime = Date.now() - startTime;
            connectionMetricsService.recordSuccess(requestId, responseTime, endpoint);

            this.pendingRequests.delete(requestId);
            return result;
          } else {
            // If we get a 4xx error, don't retry, just throw
            if (response.status >= 400 && response.status < 500) {
              const errorData = await response.json().catch(() => ({}));
              const error = new Error(`Client error: ${response.status} - ${errorData.error || response.statusText}`);
              // Record failure
              connectionMetricsService.recordFailure(requestId, error, endpoint);
              throw error;
            }

            // For 5xx errors, continue to retry
            lastError = new Error(`Server error: ${response.status} - ${response.statusText}`);
          }
        } catch (error) {
          lastError = error;

          // If it's a timeout or network error, continue to retry
          if (error.name === 'AbortError' || error.message.includes('Failed to fetch')) {
            // Continue to retry
          } else {
            // Some other error, record failure and throw immediately
            connectionMetricsService.recordFailure(requestId, error, endpoint);
            throw error;
          }
        }

        // Wait before retrying (with exponential backoff)
        if (attempt < this.config.retries - 1) {
          const delay = this.config.exponentialBackoff
            ? Math.min(this.config.retryDelay * Math.pow(2, attempt), this.config.maxRetryDelay)
            : this.config.retryDelay;

          await this.delay(delay);
        }
      }

      // If all attempts on current URL failed, try fallback URLs
      const urls = this.getAvailableUrls();
      for (let urlIndex = 0; urlIndex < urls.length; urlIndex++) {
        if (urls[urlIndex] === this.currentUrl) continue; // Skip current URL since we already tried it

        this.currentUrl = urls[urlIndex];
        this.currentUrlIndex = urlIndex;

        for (let attempt = 0; attempt < this.config.retries; attempt++) {
          try {
            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), this.config.timeout);

            const response = await fetch(`${this.currentUrl}${endpoint}`, {
              ...options,
              headers: {
                'Content-Type': 'application/json',
                ...options.headers,
              },
              signal: controller.signal,
            });

            clearTimeout(timeoutId);

            if (response.ok) {
              this.setConnectionStatus('CONNECTED');
              const result = await response.json();

              // Record successful request
              const responseTime = Date.now() - startTime;
              connectionMetricsService.recordSuccess(requestId, responseTime, endpoint);

              this.pendingRequests.delete(requestId);
              return result;
            } else {
              // If we get a 4xx error, don't retry, just throw
              if (response.status >= 400 && response.status < 500) {
                const errorData = await response.json().catch(() => ({}));
                const error = new Error(`Client error: ${response.status} - ${errorData.error || response.statusText}`);
                // Record failure
                connectionMetricsService.recordFailure(requestId, error, endpoint);
                throw error;
              }

              // For 5xx errors, continue to retry on this URL
              lastError = new Error(`Server error: ${response.status} - ${response.statusText}`);
            }
          } catch (error) {
            lastError = error;

            // If it's a timeout or network error, continue to retry
            if (error.name === 'AbortError' || error.message.includes('Failed to fetch')) {
              // Continue to retry
            } else {
              // Some other error, record failure and throw immediately
              connectionMetricsService.recordFailure(requestId, error, endpoint);
              throw error;
            }
          }

          // Wait before retrying (with exponential backoff)
          if (attempt < this.config.retries - 1) {
            const delay = this.config.exponentialBackoff
              ? Math.min(this.config.retryDelay * Math.pow(2, attempt), this.config.maxRetryDelay)
              : this.config.retryDelay;

            await this.delay(delay);
          }
        }
      }

      // If all URLs and retries failed
      this.setConnectionStatus('FAILED', lastError.message);
      // Record final failure
      connectionMetricsService.recordFailure(requestId, lastError, endpoint);
      throw lastError;
    } finally {
      this.pendingRequests.delete(requestId);
    }
  }

  // Make a query request specifically
  async queryRequest(queryText, context = null, threadId = null) {
    const requestBody = {
      query: queryText,
    };

    if (context) {
      requestBody.context = context;
    }

    if (threadId) {
      requestBody.thread_id = threadId;
    }

    return await this.makeRequest('/query', {
      method: 'POST',
      body: JSON.stringify(requestBody),
    });
  }

  // Start periodic health checks
  startHealthChecks() {
    if (this.healthCheckIntervalId) {
      clearInterval(this.healthCheckIntervalId);
    }

    this.healthCheckIntervalId = setInterval(async () => {
      await this.validateConnection();
    }, this.config.healthCheckInterval);

    return this.healthCheckIntervalId;
  }

  // Stop periodic health checks
  stopHealthChecks() {
    if (this.healthCheckIntervalId) {
      clearInterval(this.healthCheckIntervalId);
      this.healthCheckIntervalId = null;
    }
  }

  // Generate a unique request ID
  generateRequestId() {
    return `req_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  // Delay helper function
  delay(ms) {
    return new Promise(resolve => {
      const timerId = setTimeout(resolve, ms);
      this.retryTimers.add(timerId);
    });
  }

  // Cancel all pending operations
  cancelAllOperations() {
    // Clear all retry timers
    this.retryTimers.forEach(timerId => clearTimeout(timerId));
    this.retryTimers.clear();

    // Clear health check interval
    this.stopHealthChecks();

    // Cancel all pending requests by aborting their controllers
    // Note: This is a simplified version - in a real implementation,
    // you'd want to store AbortControllers for each request
    this.pendingRequests.clear();
  }

  // Get connection statistics
  getConnectionStats() {
    return {
      currentUrl: this.currentUrl,
      status: this.connectionStatus,
      urls: this.getAvailableUrls(),
      currentUrlIndex: this.currentUrlIndex,
      pendingRequests: this.pendingRequests.size,
      config: { ...this.config }
    };
  }

  // Initialize connection manager
  async initialize() {
    if (this.config.validateConnectionOnStartup) {
      await this.validateConnection();
    }

    if (this.config.healthCheckInterval > 0) {
      this.startHealthChecks();
    }
  }

  // Cleanup method
  cleanup() {
    this.cancelAllOperations();
    this.stopHealthChecks();
    this.setConnectionStatus('DISCONNECTED');
  }
}

// Create a singleton instance
export const connectionManager = new ConnectionManager();

// Export the class for creating custom instances if needed
export default ConnectionManager;