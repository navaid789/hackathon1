// frontend/src/services/logger.js
// Comprehensive logging service for the application

export class Logger {
  constructor(config = {}) {
    this.level = config.level || 'INFO';
    this.enabled = config.enabled !== false; // Default to true
    this.logToConsole = config.logToConsole !== false; // Default to true
    this.logToFile = config.logToFile || false;
    this.maxLogSize = config.maxLogSize || 1024 * 1024; // 1MB default
    this.logRetention = config.logRetention || 7; // 7 days default

    // Set up log levels
    this.levels = {
      'DEBUG': 0,
      'INFO': 1,
      'WARN': 2,
      'ERROR': 3,
      'FATAL': 4
    };

    // Initialize log buffer for file logging
    this.logBuffer = [];
    this.currentLogSize = 0;
  }

  // Check if logging is enabled for the given level
  shouldLog(level) {
    if (!this.enabled) return false;
    return this.levels[level] >= this.levels[this.level];
  }

  // Format log message
  formatMessage(level, message, meta = {}) {
    const timestamp = new Date().toISOString();
    const formattedMessage = {
      timestamp,
      level,
      message,
      meta: {
        ...meta,
        sessionId: meta.sessionId || this.getSessionId()
      }
    };

    return formattedMessage;
  }

  // Get session ID (create if doesn't exist)
  getSessionId() {
    if (!sessionStorage.getItem('sessionId')) {
      const sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      sessionStorage.setItem('sessionId', sessionId);
    }
    return sessionStorage.getItem('sessionId');
  }

  // Log to console
  logToConsole(level, formattedMessage) {
    if (!this.logToConsole) return;

    const consoleMethod = level === 'ERROR' || level === 'FATAL' ? 'error' :
                         level === 'WARN' ? 'warn' :
                         level === 'DEBUG' ? 'debug' : 'log';

    const logOutput = {
      timestamp: formattedMessage.timestamp,
      level: formattedMessage.level,
      message: formattedMessage.message,
      meta: formattedMessage.meta
    };

    console[consoleMethod](`[${formattedMessage.level}]`, logOutput);
  }

  // Log to file (simulated in browser using localStorage)
  logToFile(formattedMessage) {
    if (!this.logToFile) return;

    try {
      // Convert message to string
      const logEntry = JSON.stringify(formattedMessage) + '\n';
      const entrySize = new Blob([logEntry]).size;

      // Check if adding this entry would exceed max size
      if (this.currentLogSize + entrySize > this.maxLogSize) {
        // Rotate log - keep only recent entries
        this.rotateLog();
      }

      // Add to buffer
      this.logBuffer.push(logEntry);
      this.currentLogSize += entrySize;

      // Check if we're in a browser environment (not during SSR)
      if (typeof window !== 'undefined' && window.localStorage) {
        // Save to localStorage
        localStorage.setItem('appLogs', JSON.stringify({
          buffer: this.logBuffer,
          size: this.currentLogSize,
          lastUpdated: new Date().toISOString()
        }));
      }
    } catch (error) {
      console.warn('Failed to log to file:', error);
    }
  }

  // Rotate log to manage size
  rotateLog() {
    // Keep only the last 50% of logs to make space
    const keepCount = Math.max(10, Math.floor(this.logBuffer.length / 2));
    this.logBuffer = this.logBuffer.slice(-keepCount);

    // Recalculate size
    this.currentLogSize = this.logBuffer.reduce((total, entry) => {
      return total + new Blob([entry]).size;
    }, 0);
  }

  // Main log method
  log(level, message, meta = {}) {
    if (!this.shouldLog(level)) return;

    const formattedMessage = this.formatMessage(level, message, meta);

    // Log to console
    this.logToConsole(level, formattedMessage);

    // Log to file
    this.logToFile(formattedMessage);

    // Additional handling for errors
    if (level === 'ERROR' || level === 'FATAL') {
      this.handleAppError(formattedMessage);
    }
  }

  // Specific log level methods
  debug(message, meta = {}) {
    this.log('DEBUG', message, meta);
  }

  info(message, meta = {}) {
    this.log('INFO', message, meta);
  }

  warn(message, meta = {}) {
    this.log('WARN', message, meta);
  }

  error(message, meta = {}) {
    this.log('ERROR', message, meta);
  }

  fatal(message, meta = {}) {
    this.log('FATAL', message, meta);
  }

  // Handle application errors
  handleAppError(errorLog) {
    // Track error metrics
    if (typeof window !== 'undefined' && window.connectionMetricsService) {
      // If connection metrics service is available, record the error
      try {
        window.connectionMetricsService.recordFailure(
          `error_${Date.now()}`,
          new Error(errorLog.message),
          errorLog.meta.endpoint || 'unknown'
        );
      } catch (e) {
        // Ignore if metrics service is not available
      }
    }

    // Log to external service if configured
    this.sendErrorToExternalService(errorLog);
  }

  // Send error to external service (placeholder)
  sendErrorToExternalService(errorLog) {
    // This would typically send to an error tracking service like Sentry, etc.
    // For now, just log to console in production builds
    if (process.env.NODE_ENV === 'production') {
      console.error('External error tracking:', errorLog);
    }
  }

  // Get recent logs
  getRecentLogs(count = 50) {
    try {
      // Check if we're in a browser environment (not during SSR)
      if (typeof window !== 'undefined' && window.localStorage) {
        const stored = localStorage.getItem('appLogs');
        if (stored) {
          const data = JSON.parse(stored);
          return data.buffer.slice(-count).map(log => JSON.parse(log));
        }
      }
    } catch (error) {
      console.warn('Failed to retrieve logs:', error);
    }
    return [];
  }

  // Clear logs
  clearLogs() {
    this.logBuffer = [];
    this.currentLogSize = 0;
    // Check if we're in a browser environment (not during SSR)
    if (typeof window !== 'undefined' && window.localStorage) {
      localStorage.removeItem('appLogs');
    }
  }

  // Set log level
  setLevel(level) {
    if (this.levels[level] !== undefined) {
      this.level = level;
    }
  }

  // Export logs as downloadable file
  exportLogs() {
    try {
      const logs = this.getRecentLogs();
      const logContent = logs.map(log => JSON.stringify(log, null, 2)).join('\n\n');

      const blob = new Blob([logContent], { type: 'application/json' });
      const url = URL.createObjectURL(blob);

      const a = document.createElement('a');
      a.href = url;
      a.download = `app-logs-${new Date().toISOString().split('T')[0]}.json`;
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    } catch (error) {
      console.error('Failed to export logs:', error);
    }
  }
}

// Create a singleton instance
export const logger = new Logger({
  level: process.env.NODE_ENV === 'production' ? 'INFO' : 'DEBUG',
  enabled: true,
  logToConsole: true,
  logToFile: true
});

// Export the class for creating custom instances if needed
export default Logger;