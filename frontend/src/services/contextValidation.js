// frontend/src/services/contextValidation.js
// Service for validating that selected text is correctly passed as context

export class ContextValidationService {
  constructor() {
    this.validationResults = [];
    this.metrics = {
      totalValidations: 0,
      successfulValidations: 0,
      accuracyRate: 0
    };
  }

  // Validate that selected text context is correctly passed to the API
  validateContext(selectedText, queryRequest, apiResponse) {
    const validationResult = {
      id: Date.now() + Math.random(),
      timestamp: new Date().toISOString(),
      selectedText: selectedText,
      queryText: queryRequest.query,
      contextPassed: queryRequest.context,
      response: apiResponse,
      isValid: false,
      confidence: 0,
      issues: []
    };

    // Check if context was passed
    if (!queryRequest.context) {
      validationResult.issues.push('No context was passed with the query');
    } else if (queryRequest.context !== selectedText) {
      validationResult.issues.push('Context does not match selected text');
    }

    // Check if the response seems to address the context
    if (selectedText && apiResponse && apiResponse.response) {
      const responseLower = apiResponse.response.toLowerCase();
      const contextLower = selectedText.toLowerCase();

      // Simple heuristic: check if response contains key terms from context
      const contextWords = contextLower.split(/\s+/).filter(word => word.length > 3);
      const matchingWords = contextWords.filter(word => responseLower.includes(word));

      if (contextWords.length > 0) {
        validationResult.confidence = matchingWords.length / contextWords.length;
      } else {
        validationResult.confidence = 0;
      }

      // If confidence is high, consider it valid
      if (validationResult.confidence > 0.3) { // 30% of context words found in response
        validationResult.isValid = true;
      } else {
        validationResult.issues.push(`Response may not adequately address the context (${(validationResult.confidence * 100).toFixed(1)}% word overlap)`);
      }
    } else {
      // If no context was provided, validation is valid as long as no errors occurred
      validationResult.isValid = !validationResult.issues.length;
    }

    // Update metrics
    this.metrics.totalValidations++;
    if (validationResult.isValid) {
      this.metrics.successfulValidations++;
    }
    this.metrics.accuracyRate = this.metrics.totalValidations > 0
      ? (this.metrics.successfulValidations / this.metrics.totalValidations) * 100
      : 0;

    validationResult.metrics = { ...this.metrics };

    // Store the result
    this.validationResults.push(validationResult);

    return validationResult;
  }

  // Get overall validation metrics
  getMetrics() {
    return { ...this.metrics };
  }

  // Check if accuracy meets the 95% requirement
  meetsAccuracyRequirement() {
    return this.metrics.accuracyRate >= 95;
  }

  // Get recent validation results
  getRecentResults(limit = 10) {
    return this.validationResults.slice(-limit).reverse();
  }

  // Clear validation history
  clearHistory() {
    this.validationResults = [];
    this.metrics = {
      totalValidations: 0,
      successfulValidations: 0,
      accuracyRate: 0
    };
  }

  // Validate context relevance for a specific query-response pair
  validateContextRelevance(selectedText, query, response) {
    if (!selectedText || !response) {
      return {
        isValid: false,
        confidence: 0,
        issues: selectedText ? ['No response provided'] : ['No context provided']
      };
    }

    // Calculate semantic similarity using simple word overlap
    const contextWords = new Set(
      selectedText.toLowerCase()
        .replace(/[^\w\s]/g, ' ')
        .split(/\s+/)
        .filter(word => word.length > 2)
    );

    const responseWords = new Set(
      response.toLowerCase()
        .replace(/[^\w\s]/g, ' ')
        .split(/\s+/)
        .filter(word => word.length > 2)
    );

    // Find intersection of words
    const commonWords = [...contextWords].filter(word => responseWords.has(word));
    const totalContextWords = contextWords.size;

    const relevanceScore = totalContextWords > 0 ? commonWords.length / totalContextWords : 0;

    return {
      isValid: relevanceScore > 0.1, // At least 10% of context words should appear in response
      confidence: relevanceScore,
      issues: relevanceScore <= 0.1 ? [`Low context-response relevance (${(relevanceScore * 100).toFixed(1)}% overlap)`] : []
    };
  }

  // Perform comprehensive validation
  async comprehensiveValidation(selectedText, query, response) {
    const basicValidation = this.validateContextRelevance(selectedText, query, response);

    // Additional checks could be implemented here:
    // - Semantic similarity using embeddings (would require additional API)
    // - NLP-based relevance scoring
    // - Comparison with baseline responses without context

    return {
      ...basicValidation,
      validationId: Date.now() + Math.random(),
      timestamp: new Date().toISOString()
    };
  }
}

// Create a singleton instance
export const contextValidationService = new ContextValidationService();

// Export the class for creating custom instances if needed
export default ContextValidationService;