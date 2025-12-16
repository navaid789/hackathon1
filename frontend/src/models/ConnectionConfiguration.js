// frontend/src/models/ConnectionConfiguration.js
// Data model for connection configuration settings

export class ConnectionConfiguration {
  constructor(data = {}) {
    // Validate and set api_base_url
    this.api_base_url = data.api_base_url || data.baseURL || 'http://localhost:8000';
    if (!this.isValidUrl(this.api_base_url)) {
      throw new Error('Invalid API base URL provided');
    }

    // Validate and set timeout (between 5000 and 60000 milliseconds)
    let timeout = data.timeout || 30000;
    if (typeof timeout !== 'number' || timeout < 5000 || timeout > 60000) {
      console.warn(`Timeout ${timeout}ms is outside recommended range (5000-60000ms), using default 30000ms`);
      timeout = 30000;
    }
    this.timeout = timeout;

    // Validate and set retries (between 0 and 5)
    let retries = data.retries || 3;
    if (typeof retries !== 'number' || retries < 0 || retries > 5) {
      console.warn(`Retries ${retries} is outside recommended range (0-5), using default 3`);
      retries = 3;
    }
    this.retries = retries;
  }

  // Validate URL format
  isValidUrl(string) {
    try {
      new URL(string);
      return true;
    } catch (_) {
      return false;
    }
  }

  // Convert to plain object for serialization
  toJSON() {
    return {
      api_base_url: this.api_base_url,
      timeout: this.timeout,
      retries: this.retries
    };
  }

  // Validate the configuration
  validate() {
    const errors = [];

    if (!this.isValidUrl(this.api_base_url)) {
      errors.push('api_base_url must be a valid URL');
    }

    if (typeof this.timeout !== 'number' || this.timeout < 5000 || this.timeout > 60000) {
      errors.push('timeout must be a number between 5000 and 60000 milliseconds');
    }

    if (typeof this.retries !== 'number' || this.retries < 0 || this.retries > 5) {
      errors.push('retries must be a number between 0 and 5');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }
}

export default ConnectionConfiguration;