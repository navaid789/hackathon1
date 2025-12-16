// frontend/src/services/stateManager.js
// Service for managing loading and error states for API calls

export class StateManager {
  constructor() {
    this.state = {
      // Query state
      query: {
        status: 'IDLE', // IDLE, LOADING, SUCCESS, ERROR
        loading: false,
        error: null,
        data: null,
        timestamp: null
      },
      // Connection state
      connection: {
        status: 'DISCONNECTED', // DISCONNECTED, CONNECTING, CONNECTED, FAILED
        connected: false,
        lastError: null,
        lastChecked: null
      },
      // General API call states
      apiCalls: new Map()
    };

    this.listeners = [];
  }

  // Set query state
  setQueryState(status, data = null, error = null) {
    this.state.query = {
      status,
      loading: status === 'LOADING',
      error,
      data,
      timestamp: status !== 'IDLE' ? new Date().toISOString() : null
    };

    this.notifyListeners();
  }

  // Get query state
  getQueryState() {
    return { ...this.state.query };
  }

  // Set connection state
  setConnectionState(status, error = null) {
    this.state.connection = {
      ...this.state.connection,
      status,
      connected: status === 'CONNECTED',
      lastError: error,
      lastChecked: new Date().toISOString()
    };

    this.notifyListeners();
  }

  // Get connection state
  getConnectionState() {
    return { ...this.state.connection };
  }

  // Create a state for a specific API call
  createApiCallState(callId) {
    this.state.apiCalls.set(callId, {
      status: 'IDLE',
      loading: false,
      error: null,
      data: null,
      timestamp: null
    });

    this.notifyListeners();
  }

  // Set state for a specific API call
  setApiCallState(callId, status, data = null, error = null) {
    if (!this.state.apiCalls.has(callId)) {
      this.createApiCallState(callId);
    }

    const callState = this.state.apiCalls.get(callId);
    callState.status = status;
    callState.loading = status === 'LOADING';
    callState.error = error;
    callState.data = data;
    callState.timestamp = status !== 'IDLE' ? new Date().toISOString() : null;

    this.state.apiCalls.set(callId, callState);
    this.notifyListeners();
  }

  // Get state for a specific API call
  getApiCallState(callId) {
    const callState = this.state.apiCalls.get(callId);
    return callState ? { ...callState } : null;
  }

  // Clear state for a specific API call
  clearApiCallState(callId) {
    this.state.apiCalls.delete(callId);
    this.notifyListeners();
  }

  // Add listener for state changes
  addListener(callback) {
    if (typeof callback === 'function') {
      this.listeners.push(callback);
    }
  }

  // Remove listener
  removeListener(callback) {
    const index = this.listeners.indexOf(callback);
    if (index > -1) {
      this.listeners.splice(index, 1);
    }
  }

  // Notify all listeners of state changes
  notifyListeners() {
    this.listeners.forEach(callback => {
      try {
        callback(this.getState());
      } catch (error) {
        console.error('Error in state listener:', error);
      }
    });
  }

  // Get current state
  getState() {
    return {
      query: { ...this.state.query },
      connection: { ...this.state.connection },
      apiCalls: new Map(this.state.apiCalls)
    };
  }

  // Reset query state to initial
  resetQueryState() {
    this.setQueryState('IDLE');
  }

  // Reset all states
  resetAll() {
    this.state = {
      query: {
        status: 'IDLE',
        loading: false,
        error: null,
        data: null,
        timestamp: null
      },
      connection: {
        status: 'DISCONNECTED',
        connected: false,
        lastError: null,
        lastChecked: null
      },
      apiCalls: new Map()
    };

    this.notifyListeners();
  }

  // Check if any API call is loading
  isAnyLoading() {
    // Check query state
    if (this.state.query.loading) {
      return true;
    }

    // Check all API call states
    for (const [_, callState] of this.state.apiCalls) {
      if (callState.loading) {
        return true;
      }
    }

    return false;
  }

  // Get loading state summary
  getLoadingSummary() {
    const loadingStates = [];

    if (this.state.query.loading) {
      loadingStates.push('query');
    }

    for (const [callId, callState] of this.state.apiCalls) {
      if (callState.loading) {
        loadingStates.push(callId);
      }
    }

    return {
      anyLoading: loadingStates.length > 0,
      loadingStates
    };
  }

  // Get error summary
  getErrorSummary() {
    const errors = [];

    if (this.state.query.error) {
      errors.push({
        type: 'query',
        error: this.state.query.error,
        timestamp: this.state.query.timestamp
      });
    }

    if (this.state.connection.lastError) {
      errors.push({
        type: 'connection',
        error: this.state.connection.lastError,
        timestamp: this.state.connection.lastChecked
      });
    }

    for (const [callId, callState] of this.state.apiCalls) {
      if (callState.error) {
        errors.push({
          type: callId,
          error: callState.error,
          timestamp: callState.timestamp
        });
      }
    }

    return {
      hasErrors: errors.length > 0,
      errors
    };
  }

  // Clear errors
  clearErrors() {
    // Clear query error
    if (this.state.query.error) {
      this.state.query.error = null;
      this.notifyListeners();
    }

    // Clear connection error
    if (this.state.connection.lastError) {
      this.state.connection.lastError = null;
      this.notifyListeners();
    }

    // Clear errors from all API calls
    for (const [callId, callState] of this.state.apiCalls) {
      if (callState.error) {
        callState.error = null;
        this.state.apiCalls.set(callId, callState);
      }
    }

    this.notifyListeners();
  }
}

// Create a singleton instance
export const stateManager = new StateManager();

// Export the class for creating custom instances if needed
export default StateManager;