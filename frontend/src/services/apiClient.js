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

  // Specific method for querying the AI agent (backward compatibility)
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

  // Specific method for the new chat endpoint as per API contract
  async chat(message, context = null, userId = null, chapterFilter = null) {
    const requestBody = {
      message: message,
    };

    if (context) {
      requestBody.context = context;
    }

    if (userId) {
      requestBody.user_id = userId;
    }

    if (chapterFilter) {
      requestBody.chapter_filter = chapterFilter;
    }

    return await this.request('/chat', {
      method: 'POST',
      body: JSON.stringify(requestBody),
    });
  }

  // Specific method for searching textbook content
  async search(query, limit = 5, chapterFilter = null) {
    const requestBody = {
      query: query,
      limit: limit,
    };

    if (chapterFilter) {
      requestBody.chapter_filter = chapterFilter;
    }

    return await this.request('/search', {
      method: 'POST',
      body: JSON.stringify(requestBody),
    });
  }

  // Specific method for validating responses
  async validateResponse(response, sources, selectedText = null) {
    const requestBody = {
      response: response,
      sources: sources,
    };

    if (selectedText) {
      requestBody.selected_text = selectedText;
    }

    return await this.request('/validate-response', {
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