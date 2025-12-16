// frontend/src/models/QueryRequest.js
// Data model for query requests

export class QueryRequest {
  constructor(data = {}) {
    // Validate and set query (required, 1-2000 characters)
    if (!data.query || typeof data.query !== 'string' || data.query.trim().length === 0) {
      throw new Error('Query is required and must be a non-empty string');
    }

    if (data.query.length > 2000) {
      throw new Error('Query length must not exceed 2000 characters');
    }

    this.query = data.query;

    // Validate and set context (optional, max 5000 characters)
    if (data.context && typeof data.context !== 'string') {
      throw new Error('Context must be a string if provided');
    }

    if (data.context && data.context.length > 5000) {
      throw new Error('Context length must not exceed 5000 characters');
    }

    this.context = data.context || null;

    // Set thread_id (optional)
    this.thread_id = data.thread_id || null;

    // Set metadata (optional)
    this.metadata = data.metadata || null;
  }

  // Convert to plain object for API requests
  toJSON() {
    return {
      query: this.query,
      context: this.context,
      thread_id: this.thread_id,
      metadata: this.metadata
    };
  }

  // Validate the query request
  validate() {
    const errors = [];

    if (!this.query || typeof this.query !== 'string' || this.query.trim().length === 0) {
      errors.push('Query is required and must be a non-empty string');
    }

    if (this.query && this.query.length > 2000) {
      errors.push('Query length must not exceed 2000 characters');
    }

    if (this.context && typeof this.context !== 'string') {
      errors.push('Context must be a string if provided');
    }

    if (this.context && this.context.length > 5000) {
      errors.push('Context length must not exceed 5000 characters');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  // Create a new instance from API response data
  static fromJSON(data) {
    return new QueryRequest({
      query: data.query,
      context: data.context,
      thread_id: data.thread_id,
      metadata: data.metadata
    });
  }
}

export default QueryRequest;