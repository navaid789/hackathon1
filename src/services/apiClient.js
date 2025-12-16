// frontend/src/services/apiClient.js
// API client for communicating with the FastAPI backend
import { apiConfig } from '../config/apiConfig';

class ApiClient {
  constructor(config = {}) {
    // Use configuration from apiConfig with potential overrides
    this.baseURL = config.baseURL || apiConfig.baseURL;
    this.timeout = config.timeout || apiConfig.timeout;
    this.retries = config.retries || apiConfig.retries;
  }

  // Create a timeout promise for fetch requests
  timeoutPromise(timeoutMs) {
    return new Promise((_, reject) => {
      setTimeout(() => {
        reject(new Error(`Request timeout after ${timeoutMs}ms`));
      }, timeoutMs);
    });
  }

  // Main request method with timeout and retry logic
  async request(endpoint, options = {}) {
    const url = `${this.baseURL}${endpoint}`;
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      // Use Promise.race to implement timeout
      const timeoutPromise = new Promise((_, reject) => {
        setTimeout(() => {
          clearTimeout(timeoutId);
          reject(new Error(`Request timeout after ${this.timeout}ms`));
        }, this.timeout);
      });

      let response;
      let attempt = 0;

      while (attempt < this.retries) {
        try {
          const fetchPromise = fetch(url, {
            ...options,
            headers: {
              'Content-Type': 'application/json',
              ...options.headers,
            },
            signal: controller.signal,
          });

          // Race the fetch promise against the timeout
          response = await Promise.race([fetchPromise, timeoutPromise]);

          if (response.ok) {
            break; // Success, exit retry loop
          } else if (attempt < this.retries - 1) {
            // Only retry on certain status codes
            if (response.status >= 500 || response.status === 429) {
              attempt++;
              // Exponential backoff: wait 1s, 2s, 4s, etc.
              await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
              continue;
            } else {
              // Don't retry for client errors (4xx)
              break;
            }
          }
        } catch (error) {
          if (attempt < this.retries - 1) {
            attempt++;
            // Exponential backoff for network errors
            await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
          } else {
            throw error;
          }
        }
      }

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      return await response.json();
    } finally {
      clearTimeout(timeoutId);
    }
  }

  // Specific method for querying the AI agent
  async query(queryText, context = null, threadId = null) {
    const requestBody = {
      query: queryText,
    };

    if (context) {
      requestBody.context = context;
    }

    if (threadId) {
      requestBody.thread_id = threadId;
    }

    return await this.request('/query', {
      method: 'POST',
      body: JSON.stringify(requestBody),
    });
  }
}

// Create a singleton instance with optimized configuration
export const apiClient = new ApiClient({
  timeout: 8000, // Reduced from default 30s to 8s to meet 10s goal
  retries: 2     // Reduced from default 3 to 2 to optimize for speed
});

// Export the class for creating custom instances if needed
export default ApiClient;