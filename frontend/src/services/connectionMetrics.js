// frontend/src/services/connectionMetrics.js
// Service for tracking connection metrics to validate the 90% success requirement

export class ConnectionMetricsService {
  constructor() {
    this.sessionData = {
      sessionId: this.generateSessionId(),
      startTime: new Date().toISOString(),
      requests: [],
      failures: [],
      successCount: 0,
      failureCount: 0,
      totalRequests: 0
    };

    this.globalStats = {
      totalSessions: 0,
      successfulSessions: 0,
      failedSessions: 0,
      overallSuccessRate: 0
    };

    // Load any persisted data
    this.loadFromStorage();
  }

  // Generate a unique session ID
  generateSessionId() {
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  // Record a successful request
  recordSuccess(requestId, responseTime, endpoint) {
    const requestRecord = {
      id: requestId,
      timestamp: new Date().toISOString(),
      endpoint,
      responseTime,
      status: 'SUCCESS',
      session: this.sessionData.sessionId
    };

    this.sessionData.requests.push(requestRecord);
    this.sessionData.successCount++;
    this.sessionData.totalRequests++;

    this.saveToStorage();
  }

  // Record a failed request
  recordFailure(requestId, error, endpoint) {
    const failureRecord = {
      id: requestId,
      timestamp: new Date().toISOString(),
      endpoint,
      error: error.message || error,
      status: 'FAILURE',
      session: this.sessionData.sessionId
    };

    this.sessionData.requests.push(failureRecord);
    this.sessionData.failures.push(failureRecord);
    this.sessionData.failureCount++;
    this.sessionData.totalRequests++;

    this.saveToStorage();
  }

  // Calculate current session success rate
  getCurrentSessionSuccessRate() {
    if (this.sessionData.totalRequests === 0) {
      return 100; // No requests yet, consider successful
    }
    return (this.sessionData.successCount / this.sessionData.totalRequests) * 100;
  }

  // Check if current session meets the 90% requirement
  doesSessionMeetRequirement() {
    return this.getCurrentSessionSuccessRate() >= 90;
  }

  // End the current session and update global stats
  endSession() {
    const sessionSuccessRate = this.getCurrentSessionSuccessRate();

    this.globalStats.totalSessions++;
    if (sessionSuccessRate >= 90) {
      this.globalStats.successfulSessions++;
    } else {
      this.globalStats.failedSessions++;
    }

    this.globalStats.overallSuccessRate =
      (this.globalStats.successfulSessions / this.globalStats.totalSessions) * 100;

    // Reset current session data
    this.sessionData = {
      sessionId: this.generateSessionId(),
      startTime: new Date().toISOString(),
      requests: [],
      failures: [],
      successCount: 0,
      failureCount: 0,
      totalRequests: 0
    };

    this.saveToStorage();
  }

  // Get current session metrics
  getSessionMetrics() {
    return {
      sessionId: this.sessionData.sessionId,
      startTime: this.sessionData.startTime,
      totalRequests: this.sessionData.totalRequests,
      successCount: this.sessionData.successCount,
      failureCount: this.sessionData.failureCount,
      successRate: this.getCurrentSessionSuccessRate(),
      meetsRequirement: this.doesSessionMeetRequirement(),
      recentFailures: this.sessionData.failures.slice(-5) // Last 5 failures
    };
  }

  // Get global metrics
  getGlobalMetrics() {
    return {
      totalSessions: this.globalStats.totalSessions,
      successfulSessions: this.globalStats.successfulSessions,
      failedSessions: this.globalStats.failedSessions,
      overallSuccessRate: this.globalStats.overallSuccessRate,
      meetsOverallRequirement: this.globalStats.overallSuccessRate >= 90
    };
  }

  // Get failure analysis
  getFailureAnalysis() {
    if (this.sessionData.failures.length === 0) {
      return {
        hasFailures: false,
        failureRate: 0,
        commonErrors: [],
        failurePatterns: []
      };
    }

    // Count error types
    const errorCounts = {};
    this.sessionData.failures.forEach(failure => {
      const errorType = this.classifyError(failure.error);
      errorCounts[errorType] = (errorCounts[errorType] || 0) + 1;
    });

    // Convert to sorted array
    const commonErrors = Object.entries(errorCounts)
      .map(([type, count]) => ({ type, count, percentage: (count / this.sessionData.failures.length) * 100 }))
      .sort((a, b) => b.count - a.count);

    // Identify patterns
    const failurePatterns = [];
    if (this.sessionData.failures.length > 1) {
      // Check for consecutive failures
      let consecutiveCount = 0;
      let maxConsecutive = 0;
      this.sessionData.failures.forEach((failure, index) => {
        if (index > 0 && this.sessionData.failures[index - 1].timestamp === failure.timestamp) {
          consecutiveCount++;
          maxConsecutive = Math.max(maxConsecutive, consecutiveCount);
        } else {
          consecutiveCount = 1;
        }
      });

      if (maxConsecutive > 1) {
        failurePatterns.push(`Consecutive failures detected (max: ${maxConsecutive})`);
      }
    }

    return {
      hasFailures: true,
      failureRate: (this.sessionData.failureCount / this.sessionData.totalRequests) * 100,
      commonErrors,
      failurePatterns
    };
  }

  // Classify error types
  classifyError(errorMsg) {
    if (errorMsg.includes('timeout') || errorMsg.includes('Timeout')) {
      return 'TIMEOUT_ERROR';
    } else if (errorMsg.includes('Failed to fetch') || errorMsg.includes('NetworkError')) {
      return 'NETWORK_ERROR';
    } else if (errorMsg.includes('500') || errorMsg.includes('Internal Server Error')) {
      return 'SERVER_ERROR';
    } else if (errorMsg.includes('503') || errorMsg.includes('Service Unavailable')) {
      return 'SERVICE_UNAVAILABLE';
    } else if (errorMsg.includes('502') || errorMsg.includes('Bad Gateway')) {
      return 'BAD_GATEWAY';
    } else if (errorMsg.includes('404') || errorMsg.includes('Not Found')) {
      return 'NOT_FOUND';
    } else if (errorMsg.includes('429') || errorMsg.includes('Too Many Requests')) {
      return 'RATE_LIMIT';
    } else {
      return 'OTHER_ERROR';
    }
  }

  // Save metrics to localStorage
  saveToStorage() {
    try {
      const dataToSave = {
        sessionData: this.sessionData,
        globalStats: this.globalStats,
        lastUpdated: new Date().toISOString()
      };
      localStorage.setItem('connectionMetrics', JSON.stringify(dataToSave));
    } catch (error) {
      console.warn('Could not save connection metrics to localStorage:', error);
    }
  }

  // Load metrics from localStorage
  loadFromStorage() {
    try {
      const savedData = localStorage.getItem('connectionMetrics');
      if (savedData) {
        const parsed = JSON.parse(savedData);
        this.sessionData = parsed.sessionData || this.sessionData;
        this.globalStats = parsed.globalStats || this.globalStats;
      }
    } catch (error) {
      console.warn('Could not load connection metrics from localStorage:', error);
    }
  }

  // Reset all metrics
  resetAll() {
    this.sessionData = {
      sessionId: this.generateSessionId(),
      startTime: new Date().toISOString(),
      requests: [],
      failures: [],
      successCount: 0,
      failureCount: 0,
      totalRequests: 0
    };

    this.globalStats = {
      totalSessions: 0,
      successfulSessions: 0,
      failedSessions: 0,
      overallSuccessRate: 0
    };

    this.saveToStorage();
  }

  // Get summary report
  getSummaryReport() {
    const sessionMetrics = this.getSessionMetrics();
    const globalMetrics = this.getGlobalMetrics();
    const failureAnalysis = this.getFailureAnalysis();

    return {
      timestamp: new Date().toISOString(),
      session: sessionMetrics,
      global: globalMetrics,
      failureAnalysis,
      recommendation: this.getRecommendation(sessionMetrics, failureAnalysis)
    };
  }

  // Get recommendation based on metrics
  getRecommendation(sessionMetrics, failureAnalysis) {
    if (sessionMetrics.successRate >= 90) {
      return 'CONNECTION_STABLE: Current session meets the 90% success rate requirement.';
    } else if (failureAnalysis.failureRate > 20) {
      return 'HIGH_FAILURE_RATE: Failure rate exceeds 20%. Investigate connection issues.';
    } else if (failureAnalysis.commonErrors.length > 0) {
      const topError = failureAnalysis.commonErrors[0];
      return `COMMON_ERROR_TYPE: ${topError.type} is the most frequent error (${topError.percentage.toFixed(1)}%). Consider specific remediation.`;
    } else {
      return 'INVESTIGATE: Success rate below 90% but error pattern not clearly identified.';
    }
  }
}

// Create a singleton instance
export const connectionMetricsService = new ConnectionMetricsService();

// Export the class for creating custom instances if needed
export default ConnectionMetricsService;