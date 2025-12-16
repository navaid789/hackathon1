// frontend/src/models/AgentResponse.js
// Data model for agent responses

import { SourceReference } from './SourceReference';

export class AgentResponse {
  constructor(data = {}) {
    // Validate and set response (required)
    if (!data.response || typeof data.response !== 'string' || data.response.trim().length === 0) {
      throw new Error('Response is required and must be a non-empty string');
    }

    this.response = data.response;

    // Validate and set sources (optional array of SourceReference)
    this.sources = [];
    if (data.sources && Array.isArray(data.sources)) {
      if (data.sources.length > 10) {
        throw new Error('Sources array should have maximum 10 items');
      }

      this.sources = data.sources.map(source =>
        source instanceof SourceReference ? source : new SourceReference(source)
      );
    }

    // Validate and set thread_id (required)
    if (!data.thread_id || typeof data.thread_id !== 'string') {
      throw new Error('thread_id is required and must be a string');
    }

    this.thread_id = data.thread_id;

    // Validate and set timestamp (required, ISO 8601 format)
    if (!data.timestamp) {
      throw new Error('timestamp is required');
    }

    // Try to parse the timestamp to ensure it's valid
    try {
      this.timestamp = new Date(data.timestamp).toISOString();
    } catch (error) {
      throw new Error('timestamp must be a valid date or ISO 8601 string');
    }

    // Validate and set query_id (required)
    if (!data.query_id || typeof data.query_id !== 'string') {
      throw new Error('query_id is required and must be a string');
    }

    this.query_id = data.query_id;
  }

  // Convert to plain object
  toJSON() {
    return {
      response: this.response,
      sources: this.sources.map(source => source.toJSON()),
      thread_id: this.thread_id,
      timestamp: this.timestamp,
      query_id: this.query_id
    };
  }

  // Validate the agent response
  validate() {
    const errors = [];

    if (!this.response || typeof this.response !== 'string' || this.response.trim().length === 0) {
      errors.push('Response is required and must be a non-empty string');
    }

    if (!Array.isArray(this.sources)) {
      errors.push('Sources must be an array');
    } else if (this.sources.length > 10) {
      errors.push('Sources array should have maximum 10 items');
    }

    if (!this.thread_id || typeof this.thread_id !== 'string') {
      errors.push('thread_id is required and must be a string');
    }

    if (!this.timestamp) {
      errors.push('timestamp is required');
    } else {
      try {
        new Date(this.timestamp).toISOString();
      } catch (error) {
        errors.push('timestamp must be a valid date or ISO 8601 string');
      }
    }

    if (!this.query_id || typeof this.query_id !== 'string') {
      errors.push('query_id is required and must be a string');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  // Create a new instance from API response data
  static fromJSON(data) {
    return new AgentResponse({
      response: data.response,
      sources: data.sources,
      thread_id: data.thread_id,
      timestamp: data.timestamp,
      query_id: data.query_id
    });
  }
}

export default AgentResponse;