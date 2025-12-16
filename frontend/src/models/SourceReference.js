// frontend/src/models/SourceReference.js
// Data model for source references

export class SourceReference {
  constructor(data = {}) {
    // Validate and set url (required)
    if (!data.url || typeof data.url !== 'string') {
      throw new Error('URL is required and must be a string');
    }

    // Validate URL format
    try {
      new URL(data.url);
      this.url = data.url;
    } catch (error) {
      throw new Error('URL must be a valid URL string');
    }

    // Set section (optional)
    this.section = data.section || null;

    // Set text (optional)
    this.text = data.text || null;

    // Validate and set confidence (optional, 0-1)
    if (data.confidence !== undefined && data.confidence !== null) {
      if (typeof data.confidence !== 'number' || data.confidence < 0 || data.confidence > 1) {
        throw new Error('Confidence must be a number between 0 and 1');
      }
      this.confidence = data.confidence;
    } else {
      this.confidence = null;
    }
  }

  // Convert to plain object
  toJSON() {
    return {
      url: this.url,
      section: this.section,
      text: this.text,
      confidence: this.confidence
    };
  }

  // Validate the source reference
  validate() {
    const errors = [];

    if (!this.url || typeof this.url !== 'string') {
      errors.push('URL is required and must be a string');
    } else {
      try {
        new URL(this.url);
      } catch (error) {
        errors.push('URL must be a valid URL');
      }
    }

    if (this.confidence !== null && this.confidence !== undefined) {
      if (typeof this.confidence !== 'number' || this.confidence < 0 || this.confidence > 1) {
        errors.push('Confidence must be a number between 0 and 1');
      }
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  // Create a new instance from API response data
  static fromJSON(data) {
    return new SourceReference({
      url: data.url,
      section: data.section,
      text: data.text,
      confidence: data.confidence
    });
  }
}

export { SourceReference };
export default SourceReference;