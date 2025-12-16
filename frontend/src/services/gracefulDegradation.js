// frontend/src/services/gracefulDegradation.js
// Service for implementing graceful degradation when backend is unavailable

export class GracefulDegradationService {
  constructor(config = {}) {
    this.enabled = config.enabled !== false; // Default to true
    this.offlineMode = false;
    this.fallbackStrategies = config.fallbackStrategies || {
      cacheFirst: true, // Try cached responses first
      serviceWorker: config.serviceWorker ?? true,
      localFallback: true, // Use local fallback content
      degradedUI: true // Show degraded UI when backend is down
    };

    this.cacheConfig = {
      enabled: config.cache?.enabled ?? true,
      maxAge: config.cache?.maxAge || 300000, // 5 minutes
      maxSize: config.cache?.maxSize || 50, // 50 items
      namespace: config.cache?.namespace || 'chatbot-cache'
    };

    this.degradedFeatures = {
      queryInput: true, // Input field remains functional
      historyDisplay: true, // Show history from cache
      basicUI: true, // Core UI elements remain
      offlineIndicator: true // Show offline status
    };

    this.retryConfig = {
      enabled: config.retry?.enabled ?? true,
      maxAttempts: config.retry?.maxAttempts || 3,
      baseDelay: config.retry?.baseDelay || 1000, // 1 second
      maxDelay: config.retry?.maxDelay || 10000, // 10 seconds
      backoffMultiplier: config.retry?.backoffMultiplier || 2
    };

    this.cache = new Map(); // Simple in-memory cache
    this.offlineQueue = []; // Queue for offline requests
    this.retryTimers = new Set();

    this.statusListeners = [];
  }

  // Check if the service is in offline/degraded mode
  isInOfflineMode() {
    return this.offlineMode;
  }

  // Set offline mode
  setOfflineMode(isOffline) {
    this.offlineMode = isOffline;
    this.notifyStatusListeners(isOffline);
  }

  // Add a listener for status changes
  addStatusListener(callback) {
    if (typeof callback === 'function') {
      this.statusListeners.push(callback);
    }
  }

  // Remove a status listener
  removeStatusListener(callback) {
    const index = this.statusListeners.indexOf(callback);
    if (index > -1) {
      this.statusListeners.splice(index, 1);
    }
  }

  // Notify all status listeners
  notifyStatusListeners(isOffline) {
    this.statusListeners.forEach(callback => {
      try {
        callback(isOffline);
      } catch (error) {
        console.error('Error in status listener:', error);
      }
    });
  }

  // Cache a response
  async cacheResponse(key, response, ttl = this.cacheConfig.maxAge) {
    if (!this.cacheConfig.enabled) return;

    const cacheItem = {
      response,
      timestamp: Date.now(),
      ttl
    };

    // Remove oldest item if cache is full
    if (this.cache.size >= this.cacheConfig.maxSize) {
      const firstKey = this.cache.keys().next().value;
      this.cache.delete(firstKey);
    }

    this.cache.set(key, cacheItem);
  }

  // Get cached response
  async getCachedResponse(key) {
    if (!this.cacheConfig.enabled) return null;

    const cacheItem = this.cache.get(key);
    if (!cacheItem) return null;

    // Check if cache item is expired
    if (Date.now() - cacheItem.timestamp > cacheItem.ttl) {
      this.cache.delete(key);
      return null;
    }

    return cacheItem.response;
  }

  // Clear cache
  clearCache() {
    this.cache.clear();
  }

  // Queue a request for later processing when online
  queueRequest(requestData) {
    if (this.offlineQueue.length >= 100) { // Limit queue size
      this.offlineQueue.shift(); // Remove oldest
    }

    this.offlineQueue.push({
      ...requestData,
      queuedAt: Date.now(),
      attempts: 0
    });
  }

  // Process queued requests
  async processQueuedRequests() {
    if (!this.retryConfig.enabled || this.offlineQueue.length === 0) {
      return 0; // No requests to process
    }

    let processed = 0;
    const remainingQueue = [];

    for (const queuedRequest of this.offlineQueue) {
      if (queuedRequest.attempts >= this.retryConfig.maxAttempts) {
        // Give up on this request
        continue;
      }

      try {
        // Attempt to process the request
        // Note: In a real implementation, you'd call the actual API here
        // For this example, we'll simulate the call
        const success = await this.simulateProcessRequest(queuedRequest);

        if (success) {
          processed++;
        } else {
          // Increment attempts and keep in queue
          remainingQueue.push({
            ...queuedRequest,
            attempts: queuedRequest.attempts + 1
          });
        }
      } catch (error) {
        // Keep in queue for retry
        remainingQueue.push({
          ...queuedRequest,
          attempts: queuedRequest.attempts + 1
        });
      }
    }

    this.offlineQueue = remainingQueue;
    return processed;
  }

  // Simulate processing a request (would call actual API in real implementation)
  async simulateProcessRequest(requestData) {
    // This is a placeholder - in a real implementation, you would
    // make the actual API call here
    try {
      // Simulate API call
      const response = await fetch(requestData.url, requestData.options);
      return response.ok;
    } catch (error) {
      return false;
    }
  }

  // Handle API call with graceful degradation
  async makeApiCall(url, options = {}) {
    if (!this.enabled) {
      // If service is disabled, make normal call
      return await fetch(url, options);
    }

    try {
      // Try the API call
      const response = await fetch(url, options);

      if (response.ok) {
        // Success - update status to online
        if (this.offlineMode) {
          this.setOfflineMode(false);
          // Process any queued requests
          await this.processQueuedRequests();
        }

        // Cache the successful response
        const cacheKey = `${options.method || 'GET'}:${url}`;
        const responseClone = response.clone();
        const responseBody = await responseClone.json();
        await this.cacheResponse(cacheKey, responseBody);

        return response;
      } else {
        // Got a response but it's an error - might still be online
        if (response.status >= 500) {
          // Server error - treat as offline
          this.setOfflineMode(true);
        }
        return response;
      }
    } catch (error) {
      // Network error - definitely offline
      this.setOfflineMode(true);

      // Check if we have a cached response
      const cacheKey = `${options.method || 'GET'}:${url}`;
      const cachedResponse = await this.getCachedResponse(cacheKey);

      if (cachedResponse && this.fallbackStrategies.cacheFirst) {
        // Return cached response as fallback
        console.warn('Returning cached response due to network error');

        // Create a mock response object
        return {
          ok: true,
          status: 200,
          json: async () => cachedResponse,
          text: async () => JSON.stringify(cachedResponse),
          headers: new Headers({ 'content-type': 'application/json' })
        };
      }

      // Queue the request for later
      if (this.fallbackStrategies.localFallback) {
        this.queueRequest({ url, options, originalError: error });
      }

      // Throw a more descriptive error
      throw new Error(`Network error: ${error.message}. Service is operating in degraded mode.`);
    }
  }

  // Get degraded mode UI configuration
  getDegradedUIConfig() {
    if (!this.degradedFeatures.degradedUI) {
      return { isDegraded: false };
    }

    return {
      isDegraded: this.offlineMode,
      features: {
        ...this.degradedFeatures,
        canSubmitQueries: !this.offlineMode, // Can't submit when offline
        showCachedHistory: this.cache.size > 0,
        showOfflineIndicator: this.offlineMode && this.degradedFeatures.offlineIndicator,
        queuedRequests: this.offlineQueue.length
      }
    };
  }

  // Get service status
  getStatus() {
    return {
      isOffline: this.offlineMode,
      cacheSize: this.cache.size,
      queuedRequests: this.offlineQueue.length,
      enabled: this.enabled,
      timestamp: new Date().toISOString()
    };
  }

  // Initialize service worker for offline support (if enabled)
  async initializeServiceWorker() {
    if (!this.fallbackStrategies.serviceWorker || !('serviceWorker' in navigator)) {
      return false;
    }

    try {
      await navigator.serviceWorker.register('/sw.js'); // Would need to create this
      console.log('Service Worker registered for offline support');
      return true;
    } catch (error) {
      console.warn('Service Worker registration failed:', error);
      return false;
    }
  }

  // Get offline fallback content
  getOfflineFallbackContent(query) {
    if (!this.fallbackStrategies.localFallback) {
      return null;
    }

    // Provide helpful offline content
    const fallbackResponses = {
      default: "I'm currently unable to reach the AI service. You can continue to browse the book content, and your queries will be processed when the connection is restored.",
      help: "I'm currently offline. Please check your internet connection. Your queries are being queued and will be processed when the service is available again.",
      error: "The AI service is temporarily unavailable. Please try again later. In the meantime, you can browse the book content."
    };

    const queryLower = (query || '').toLowerCase();

    if (queryLower.includes('help') || queryLower.includes('support')) {
      return fallbackResponses.help;
    } else if (queryLower.includes('error') || queryLower.includes('problem')) {
      return fallbackResponses.error;
    } else {
      return fallbackResponses.default;
    }
  }

  // Retry a specific request with exponential backoff
  async retryWithBackoff(requestFn, maxAttempts = this.retryConfig.maxAttempts) {
    if (!this.retryConfig.enabled) {
      return await requestFn();
    }

    let lastError;

    for (let attempt = 0; attempt < maxAttempts; attempt++) {
      try {
        return await requestFn();
      } catch (error) {
        lastError = error;

        if (attempt < maxAttempts - 1) {
          // Calculate delay with exponential backoff
          const delay = Math.min(
            this.retryConfig.baseDelay * Math.pow(this.retryConfig.backoffMultiplier, attempt),
            this.retryConfig.maxDelay
          );

          // Wait before retrying
          await new Promise(resolve => {
            const timerId = setTimeout(resolve, delay);
            this.retryTimers.add(timerId);
          });
        }
      }
    }

    throw lastError;
  }

  // Cleanup resources
  cleanup() {
    // Clear retry timers
    this.retryTimers.forEach(timerId => clearTimeout(timerId));
    this.retryTimers.clear();

    // Clear all listeners
    this.statusListeners = [];
  }

  // Get a summary report of the service
  getReport() {
    return {
      status: this.getStatus(),
      config: {
        enabled: this.enabled,
        fallbackStrategies: this.fallbackStrategies,
        cacheConfig: this.cacheConfig,
        retryConfig: this.retryConfig
      },
      degradationEffects: this.degradedFeatures,
      timestamp: new Date().toISOString()
    };
  }
}

// Create a singleton instance
export const gracefulDegradationService = new GracefulDegradationService({
  fallbackStrategies: {
    cacheFirst: true,
    serviceWorker: true,
    localFallback: true,
    degradedUI: true
  },
  cache: {
    enabled: true,
    maxAge: 300000, // 5 minutes
    maxSize: 50
  },
  retry: {
    enabled: true,
    maxAttempts: 3,
    baseDelay: 1000,
    maxDelay: 10000,
    backoffMultiplier: 2
  }
});

// Export the class for creating custom instances if needed
export default GracefulDegradationService;