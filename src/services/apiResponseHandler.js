// frontend/src/services/apiResponseHandler.js
// Service for handling API responses and errors

import { AgentResponse } from '../models/AgentResponse';

export class ApiResponseHandler {
  constructor() {
    this.responseHandlers = new Map();
  }

  // Handle successful API responses
  handleSuccess(responseData, requestType = 'query') {
    try {
      switch (requestType) {
        case 'query':
          return this.handleQueryResponse(responseData);
        default:
          return this.handleGenericResponse(responseData);
      }
    } catch (error) {
      console.error(`Error handling ${requestType} response:`, error);
      throw new Error(`Invalid response format for ${requestType}: ${error.message}`);
    }
  }

  // Handle query-specific response
  handleQueryResponse(responseData) {
    // Validate required fields
    if (!responseData.response) {
      throw new Error('Response data missing required "response" field');
    }

    if (!responseData.thread_id) {
      throw new Error('Response data missing required "thread_id" field');
    }

    if (!responseData.query_id) {
      throw new Error('Response data missing required "query_id" field');
    }

    if (!responseData.timestamp) {
      throw new Error('Response data missing required "timestamp" field');
    }

    // Create and return AgentResponse instance
    return new AgentResponse({
      response: responseData.response,
      sources: responseData.sources,
      thread_id: responseData.thread_id,
      query_id: responseData.query_id,
      timestamp: responseData.timestamp
    });
  }

  // Handle generic API response
  handleGenericResponse(responseData) {
    return {
      data: responseData,
      success: true,
      timestamp: new Date().toISOString()
    };
  }

  // Handle API errors
  handleError(error, requestType = 'query') {
    let errorInfo = {
      type: 'UNKNOWN_ERROR',
      message: 'An unknown error occurred',
      details: null,
      status: null,
      requestType: requestType
    };

    // Handle different types of errors
    if (error instanceof TypeError && error.message.includes('fetch')) {
      // Network error
      errorInfo.type = 'NETWORK_ERROR';
      errorInfo.message = 'Network error: Unable to connect to the server';
      errorInfo.details = 'Please check your internet connection and ensure the backend server is running';
    } else if (error.message.includes('timeout')) {
      // Timeout error
      errorInfo.type = 'TIMEOUT_ERROR';
      errorInfo.message = 'Request timeout: The server took too long to respond';
      errorInfo.details = 'Please try again later';
    } else if (error.message.includes('HTTP')) {
      // HTTP error (e.g., "HTTP 400: Bad Request")
      const match = error.message.match(/HTTP (\d+): (.+)/);
      if (match) {
        const status = parseInt(match[1]);
        errorInfo.status = status;
        errorInfo.message = match[2];

        switch (status) {
          case 400:
            errorInfo.type = 'BAD_REQUEST_ERROR';
            errorInfo.message = 'Bad request: Invalid query parameters';
            break;
          case 401:
            errorInfo.type = 'UNAUTHORIZED_ERROR';
            errorInfo.message = 'Unauthorized: Please check your authentication';
            break;
          case 403:
            errorInfo.type = 'FORBIDDEN_ERROR';
            errorInfo.message = 'Forbidden: You do not have permission to access this resource';
            break;
          case 404:
            errorInfo.type = 'NOT_FOUND_ERROR';
            errorInfo.message = 'Not found: The requested resource was not found';
            break;
          case 429:
            errorInfo.type = 'RATE_LIMIT_ERROR';
            errorInfo.message = 'Rate limit exceeded: Too many requests';
            errorInfo.details = 'Please wait before making another request';
            break;
          case 500:
            errorInfo.type = 'SERVER_ERROR';
            errorInfo.message = 'Server error: An internal server error occurred';
            errorInfo.details = 'Please try again later';
            break;
          case 502:
            errorInfo.type = 'GATEWAY_ERROR';
            errorInfo.message = 'Gateway error: The server received an invalid response';
            break;
          case 503:
            errorInfo.type = 'SERVICE_UNAVAILABLE_ERROR';
            errorInfo.message = 'Service unavailable: The server is temporarily unable to handle the request';
            errorInfo.details = 'Please try again later';
            break;
          default:
            errorInfo.type = 'HTTP_ERROR';
            errorInfo.message = `HTTP ${status}: ${errorInfo.message}`;
        }
      }
    } else if (error.message.includes('Invalid response format')) {
      // Format error
      errorInfo.type = 'FORMAT_ERROR';
      errorInfo.message = error.message;
    } else {
      // Other error
      errorInfo.type = 'GENERAL_ERROR';
      errorInfo.message = error.message || 'An error occurred';
    }

    // Log the error for debugging
    console.error(`API Error (${errorInfo.type}):`, error);

    return errorInfo;
  }

  // Validate API response structure
  validateResponse(responseData, expectedStructure) {
    if (!responseData) {
      return {
        isValid: false,
        errors: ['Response data is null or undefined']
      };
    }

    const errors = [];
    if (expectedStructure) {
      // Validate based on expected structure
      for (const field of expectedStructure.required || []) {
        if (responseData[field] === undefined || responseData[field] === null) {
          errors.push(`Missing required field: ${field}`);
        }
      }
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  // Register a custom response handler
  registerHandler(name, handler) {
    if (typeof handler === 'function') {
      this.responseHandlers.set(name, handler);
    }
  }

  // Execute a registered response handler
  executeHandler(name, ...args) {
    const handler = this.responseHandlers.get(name);
    if (handler) {
      return handler(...args);
    }
    throw new Error(`No handler registered with name: ${name}`);
  }

  // Format error response for UI display
  formatErrorForDisplay(errorInfo) {
    const displayError = {
      title: 'Error',
      message: errorInfo.message,
      details: errorInfo.details || null,
      type: errorInfo.type,
      status: errorInfo.status || null
    };

    // Customize title based on error type
    switch (errorInfo.type) {
      case 'NETWORK_ERROR':
        displayError.title = 'Network Error';
        break;
      case 'TIMEOUT_ERROR':
        displayError.title = 'Timeout Error';
        break;
      case 'SERVER_ERROR':
        displayError.title = 'Server Error';
        break;
      case 'RATE_LIMIT_ERROR':
        displayError.title = 'Rate Limit Exceeded';
        break;
      case 'BAD_REQUEST_ERROR':
        displayError.title = 'Invalid Request';
        break;
      default:
        displayError.title = 'Error';
    }

    return displayError;
  }
}

// Create a singleton instance
export const apiResponseHandler = new ApiResponseHandler();

// Export the class for creating custom instances if needed
export default ApiResponseHandler;