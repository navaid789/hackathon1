// frontend/src/services/performanceMonitor.js
// Performance monitoring service for API response times

export class PerformanceMonitor {
  constructor(config = {}) {
    this.enabled = config.enabled !== false; // Default to true
    this.thresholds = {
      warning: config.warningThreshold || 3000, // 3 seconds
      error: config.errorThreshold || 10000,    // 10 seconds
      critical: config.criticalThreshold || 20000 // 20 seconds
    };

    this.metrics = {
      totalRequests: 0,
      totalResponseTime: 0,
      averageResponseTime: 0,
      slowRequests: 0,
      errorRequests: 0,
      responseTimes: [] // Keep last 100 response times for analysis
    };

    this.performanceObservers = [];
    this.apiCallTimings = new Map();
  }

  // Start performance monitoring for an API call
  startAPICallTracking(callId, endpoint, method = 'GET') {
    if (!this.enabled) return;

    const startTime = performance.now();
    const timingData = {
      id: callId,
      endpoint,
      method,
      startTime,
      timestamp: new Date().toISOString()
    };

    this.apiCallTimings.set(callId, timingData);
  }

  // End performance monitoring for an API call and record metrics
  endAPICallTracking(callId, success = true, additionalData = {}) {
    if (!this.enabled) return null;

    const timingData = this.apiCallTimings.get(callId);
    if (!timingData) {
      console.warn(`Performance monitoring: No start time found for call ID: ${callId}`);
      return null;
    }

    const endTime = performance.now();
    const responseTime = endTime - timingData.startTime;

    // Remove from tracking map
    this.apiCallTimings.delete(callId);

    // Record metrics
    this.recordAPIMetric({
      ...timingData,
      endTime,
      responseTime,
      success,
      ...additionalData
    });

    // Check if response time exceeds thresholds
    this.checkResponseTimeThresholds(responseTime, timingData);

    return responseTime;
  }

  // Record API metric
  recordAPIMetric(metricData) {
    this.metrics.totalRequests++;
    this.metrics.totalResponseTime += metricData.responseTime;
    this.metrics.averageResponseTime = this.metrics.totalResponseTime / this.metrics.totalRequests;

    // Add to response times array (keep only last 100)
    this.metrics.responseTimes.push(metricData.responseTime);
    if (this.metrics.responseTimes.length > 100) {
      this.metrics.responseTimes.shift();
    }

    // Count slow requests
    if (metricData.responseTime > this.thresholds.warning) {
      this.metrics.slowRequests++;
    }

    // Count error requests
    if (!metricData.success) {
      this.metrics.errorRequests++;
    }
  }

  // Check response time against thresholds
  checkResponseTimeThresholds(responseTime, timingData) {
    if (responseTime > this.thresholds.critical) {
      console.error(`CRITICAL PERFORMANCE ALERT: API call to ${timingData.endpoint} took ${responseTime.toFixed(2)}ms`, timingData);
    } else if (responseTime > this.thresholds.error) {
      console.warn(`PERFORMANCE WARNING: API call to ${timingData.endpoint} took ${responseTime.toFixed(2)}ms (above error threshold)`, timingData);
    } else if (responseTime > this.thresholds.warning) {
      console.info(`API call to ${timingData.endpoint} took ${responseTime.toFixed(2)}ms (above warning threshold)`, timingData);
    }
  }

  // Get current performance metrics
  getMetrics() {
    return {
      ...this.metrics,
      thresholds: { ...this.thresholds },
      enabled: this.enabled,
      timestamp: new Date().toISOString()
    };
  }

  // Get performance summary
  getPerformanceSummary() {
    const summary = {
      totalRequests: this.metrics.totalRequests,
      averageResponseTime: this.metrics.averageResponseTime,
      slowRequestPercentage: this.metrics.totalRequests > 0
        ? (this.metrics.slowRequests / this.metrics.totalRequests) * 100
        : 0,
      errorPercentage: this.metrics.totalRequests > 0
        ? (this.metrics.errorRequests / this.metrics.totalRequests) * 100
        : 0,
      isPerformingWell: this.metrics.averageResponseTime < this.thresholds.warning
    };

    return summary;
  }

  // Get percentile response times (approximate)
  getPercentile(percentile) {
    if (this.metrics.responseTimes.length === 0) {
      return 0;
    }

    const sorted = [...this.metrics.responseTimes].sort((a, b) => a - b);
    const index = Math.floor((percentile / 100) * sorted.length);
    return sorted[Math.min(index, sorted.length - 1)] || 0;
  }

  // Monitor a function call with performance tracking
  async monitorFunction(fn, context = 'unknown', additionalData = {}) {
    if (!this.enabled) {
      return await fn();
    }

    const callId = `fn_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    this.startAPICallTracking(callId, context, 'FUNCTION');

    try {
      const result = await fn();
      this.endAPICallTracking(callId, true, { ...additionalData, result: 'success' });
      return result;
    } catch (error) {
      this.endAPICallTracking(callId, false, { ...additionalData, error: error.message });
      throw error;
    }
  }

  // Monitor an API call with performance tracking
  async monitorAPICall(url, options = {}) {
    if (!this.enabled) {
      return await fetch(url, options);
    }

    const callId = `api_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    this.startAPICallTracking(callId, url, options.method || 'GET');

    try {
      const response = await fetch(url, options);
      const success = response.ok;
      this.endAPICallTracking(callId, success, {
        status: response.status,
        statusText: response.statusText
      });
      return response;
    } catch (error) {
      this.endAPICallTracking(callId, false, { error: error.message });
      throw error;
    }
  }

  // Reset all metrics
  resetMetrics() {
    this.metrics = {
      totalRequests: 0,
      totalResponseTime: 0,
      averageResponseTime: 0,
      slowRequests: 0,
      errorRequests: 0,
      responseTimes: []
    };
  }

  // Enable/disable monitoring
  setEnabled(enabled) {
    this.enabled = !!enabled;
  }

  // Export performance data
  exportPerformanceData() {
    return {
      metrics: this.getMetrics(),
      summary: this.getPerformanceSummary(),
      percentiles: {
        p50: this.getPercentile(50),
        p75: this.getPercentile(75),
        p90: this.getPercentile(90),
        p95: this.getPercentile(95),
        p99: this.getPercentile(99)
      }
    };
  }

  // Generate performance report
  generatePerformanceReport() {
    const summary = this.getPerformanceSummary();
    const percentiles = {
      p50: this.getPercentile(50),
      p75: this.getPercentile(75),
      p90: this.getPercentile(90),
      p95: this.getPercentile(95),
      p99: this.getPercentile(99)
    };

    return `
API Performance Report
=======================
Timestamp: ${new Date().toISOString()}
Total Requests: ${summary.totalRequests}
Average Response Time: ${summary.averageResponseTime.toFixed(2)}ms
Slow Request Percentage: ${summary.slowRequestPercentage.toFixed(2)}%
Error Percentage: ${summary.errorPercentage.toFixed(2)}%
Performance Status: ${summary.isPerformingWell ? 'GOOD' : 'NEEDS ATTENTION'}

Response Time Percentiles:
  P50: ${percentiles.p50.toFixed(2)}ms
  P75: ${percentiles.p75.toFixed(2)}ms
  P90: ${percentiles.p90.toFixed(2)}ms
  P95: ${percentiles.p95.toFixed(2)}ms
  P99: ${percentiles.p99.toFixed(2)}ms

Thresholds:
  Warning: ${this.thresholds.warning}ms
  Error: ${this.thresholds.error}ms
  Critical: ${this.thresholds.critical}ms
    `;
  }
}

// Create a singleton instance
export const performanceMonitor = new PerformanceMonitor({
  warningThreshold: 3000,   // 3 seconds
  errorThreshold: 10000,    // 10 seconds
  criticalThreshold: 20000  // 20 seconds
});

// Export the class for creating custom instances if needed
export default PerformanceMonitor;