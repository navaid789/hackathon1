// frontend/src/services/security.js
// Security measures for API communication

export class SecurityService {
  constructor(config = {}) {
    this.enabled = config.enabled !== false; // Default to true
    this.securityHeaders = config.securityHeaders || {
      'X-Requested-With': 'XMLHttpRequest',
      'X-Content-Type-Options': 'nosniff',
      'X-Frame-Options': 'DENY',
      'X-XSS-Protection': '1; mode=block'
    };

    this.rateLimiting = {
      enabled: config.rateLimiting?.enabled ?? true,
      maxRequests: config.rateLimiting?.maxRequests || 10,
      windowMs: config.rateLimiting?.windowMs || 60000, // 1 minute
      store: new Map() // Store request counts by IP/client
    };

    this.inputValidation = {
      enabled: config.inputValidation?.enabled !== false,
      maxQueryLength: config.inputValidation?.maxQueryLength || 2000,
      maxContextLength: config.inputValidation?.maxContextLength || 5000,
      allowedCharacters: config.inputValidation?.allowedCharacters || /^[a-zA-Z0-9\s\-\.,!?;:'"(){}[\]\/\\_+=<>@#$%^&*~`|]+$/,
      sanitizeHtml: config.inputValidation?.sanitizeHtml ?? true
    };

    this.csrfProtection = {
      enabled: config.csrfProtection?.enabled !== false,
      tokenEndpoint: config.csrfProtection?.tokenEndpoint || '/csrf-token',
      tokenName: config.csrfProtection?.tokenName || 'csrf_token',
      token: null
    };

    this.trustedDomains = config.trustedDomains || [
      'localhost',
      '127.0.0.1',
      '0.0.0.0',
      '.vercel.app' // For Vercel deployments
    ];
  }

  // Validate input data
  validateInput(data) {
    if (!this.inputValidation.enabled) {
      return { isValid: true, errors: [] };
    }

    const errors = [];

    // Validate query length
    if (data.query && data.query.length > this.inputValidation.maxQueryLength) {
      errors.push(`Query exceeds maximum length of ${this.inputValidation.maxQueryLength} characters`);
    }

    // Validate context length
    if (data.context && data.context.length > this.inputValidation.maxContextLength) {
      errors.push(`Context exceeds maximum length of ${this.inputValidation.maxContextLength} characters`);
    }

    // Validate character patterns
    if (data.query && !this.inputValidation.allowedCharacters.test(data.query)) {
      errors.push('Query contains disallowed characters');
    }

    if (data.context && !this.inputValidation.allowedCharacters.test(data.context)) {
      errors.push('Context contains disallowed characters');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  // Sanitize input to prevent XSS
  sanitizeInput(input) {
    if (!this.inputValidation.sanitizeHtml || typeof input !== 'string') {
      return input;
    }

    // Remove potentially dangerous HTML elements and attributes
    const div = document.createElement('div');
    div.textContent = input;
    return div.innerHTML
      .replace(/<(script|iframe|object|embed|form|input|img)[^>]*>.*?<\/\1>/gi, '')
      .replace(/<(script|iframe|object|embed|form|input|img)/gi, '')
      .replace(/javascript:/gi, '')
      .replace(/on\w+="[^"]*"/gi, '');
  }

  // Check if request is rate limited
  isRateLimited(identifier = 'default') {
    if (!this.rateLimiting.enabled) {
      return false;
    }

    const now = Date.now();
    const windowStart = now - this.rateLimiting.windowMs;

    // Clean up old entries
    for (const [key, timestamps] of this.rateLimiting.store.entries()) {
      const filtered = timestamps.filter(timestamp => timestamp > windowStart);
      this.rateLimiting.store.set(key, filtered);
    }

    // Get current requests for this identifier
    let requests = this.rateLimiting.store.get(identifier) || [];

    // Check if rate limit exceeded
    if (requests.length >= this.rateLimiting.maxRequests) {
      return true;
    }

    // Add current request
    requests.push(now);
    this.rateLimiting.store.set(identifier, requests);

    return false;
  }

  // Get security headers for API requests
  getSecurityHeaders() {
    if (!this.enabled) {
      return {};
    }

    const headers = { ...this.securityHeaders };

    // Add CSRF token if available
    if (this.csrfProtection.token) {
      headers[this.csrfProtection.tokenName] = this.csrfProtection.token;
    }

    return headers;
  }

  // Validate URL for security
  validateUrl(url) {
    try {
      const parsedUrl = new URL(url);

      // Check if hostname is in trusted domains
      const isTrusted = this.trustedDomains.some(domain =>
        parsedUrl.hostname === domain || parsedUrl.hostname.endsWith(domain)
      );

      if (!isTrusted) {
        throw new Error(`Untrusted domain: ${parsedUrl.hostname}`);
      }

      // Ensure HTTPS in production
      if (process.env.NODE_ENV === 'production' && parsedUrl.protocol !== 'https:') {
        throw new Error('Production requests must use HTTPS');
      }

      return { isValid: true, url: parsedUrl.href };
    } catch (error) {
      return { isValid: false, error: error.message };
    }
  }

  // Prepare secure request options
  prepareSecureRequest(options = {}, endpoint) {
    const secureOptions = { ...options };

    // Add security headers
    secureOptions.headers = {
      ...secureOptions.headers,
      ...this.getSecurityHeaders()
    };

    // Validate and sanitize input if present
    if (secureOptions.body) {
      let bodyData;

      if (typeof secureOptions.body === 'string') {
        try {
          bodyData = JSON.parse(secureOptions.body);
        } catch {
          bodyData = { raw: secureOptions.body }; // Treat as raw data
        }
      } else {
        bodyData = secureOptions.body;
      }

      // Validate input
      const validation = this.validateInput(bodyData);
      if (!validation.isValid) {
        throw new Error(`Input validation failed: ${validation.errors.join(', ')}`);
      }

      // Sanitize input
      if (bodyData.query) {
        bodyData.query = this.sanitizeInput(bodyData.query);
      }
      if (bodyData.context) {
        bodyData.context = this.sanitizeInput(bodyData.context);
      }

      // Update body
      if (typeof options.body === 'string') {
        secureOptions.body = JSON.stringify(bodyData);
      } else {
        secureOptions.body = bodyData;
      }
    }

    return secureOptions;
  }

  // Make a secure API call
  async makeSecureRequest(url, options = {}) {
    if (!this.enabled) {
      return await fetch(url, options);
    }

    // Validate URL
    const urlValidation = this.validateUrl(url);
    if (!urlValidation.isValid) {
      throw new Error(`Invalid URL: ${urlValidation.error}`);
    }

    // Check rate limiting (use IP or a generated client ID as identifier)
    const clientId = this.getClientId();
    if (this.isRateLimited(clientId)) {
      throw new Error('Rate limit exceeded. Please try again later.');
    }

    // Prepare secure request
    const secureOptions = this.prepareSecureRequest(options, url);

    return await fetch(url, secureOptions);
  }

  // Get client identifier for rate limiting
  getClientId() {
    // In browser environment, we can use various identifiers
    if (typeof window !== 'undefined') {
      // Use a combination of factors to identify the client
      const fingerprint = `${navigator.userAgent}-${location.hostname}-${Date.now()}`;
      return btoa(fingerprint).substring(0, 16); // Simple hash-like identifier
    }
    return 'server-client';
  }

  // Refresh CSRF token
  async refreshCsrfToken() {
    if (!this.csrfProtection.enabled) {
      return;
    }

    try {
      const response = await fetch(this.csrfProtection.tokenEndpoint, {
        method: 'GET',
        headers: {
          'X-Requested-With': 'XMLHttpRequest'
        }
      });

      if (response.ok) {
        const data = await response.json();
        this.csrfProtection.token = data[this.csrfProtection.tokenName] || data.token;
      }
    } catch (error) {
      console.warn('Failed to refresh CSRF token:', error);
      // Token refresh failures shouldn't break functionality
    }
  }

  // Check if content is safe to display
  isContentSafe(content) {
    if (!content || typeof content !== 'string') {
      return true;
    }

    // Check for potentially dangerous patterns
    const dangerousPatterns = [
      /<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi,
      /javascript:/gi,
      /on\w+\s*=/gi,
      /<iframe/gi,
      /<object/gi,
      /<embed/gi
    ];

    return !dangerousPatterns.some(pattern => pattern.test(content));
  }

  // Apply security measures to content before displaying
  secureDisplayContent(content) {
    if (!this.inputValidation.sanitizeHtml) {
      return content;
    }

    if (!this.isContentSafe(content)) {
      console.warn('Potentially unsafe content detected and sanitized:', content);
      return this.sanitizeInput(content);
    }

    return content;
  }

  // Get security report
  getSecurityReport() {
    return {
      enabled: this.enabled,
      rateLimiting: {
        enabled: this.rateLimiting.enabled,
        maxRequests: this.rateLimiting.maxRequests,
        windowMs: this.rateLimiting.windowMs,
        currentStoreSize: this.rateLimiting.store.size
      },
      inputValidation: {
        enabled: this.inputValidation.enabled,
        maxQueryLength: this.inputValidation.maxQueryLength,
        maxContextLength: this.inputValidation.maxContextLength
      },
      csrfProtection: {
        enabled: this.csrfProtection.enabled,
        hasToken: !!this.csrfProtection.token
      },
      trustedDomains: this.trustedDomains,
      timestamp: new Date().toISOString()
    };
  }
}

// Create a singleton instance
export const securityService = new SecurityService({
  rateLimiting: {
    enabled: true,
    maxRequests: 10,
    windowMs: 60000
  },
  inputValidation: {
    enabled: true,
    maxQueryLength: 2000,
    maxContextLength: 5000
  }
});

// Export the class for creating custom instances if needed
export default SecurityService;