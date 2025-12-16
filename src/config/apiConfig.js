// frontend/src/config/apiConfig.js
// Configuration for API client in different environments

const API_CONFIG = {
  development: {
    baseURL: process.env.REACT_APP_API_BASE_URL || 'http://127.0.0.1:8002',  // Use mock server for local dev
    timeout: 30000,
    retries: 3
  },
  production: {
    baseURL: process.env.REACT_APP_API_BASE_URL || 'https://your-backend-url.vercel.app',  // Use production backend
    timeout: 30000,
    retries: 3
  },
  test: {
    baseURL: process.env.REACT_APP_API_BASE_URL || 'http://127.0.0.1:8002',  // Use mock server for testing
    timeout: 10000,
    retries: 1
  }
};

// Determine the current environment
const getEnvironment = () => {
  if (typeof process !== 'undefined' && process.env && process.env.NODE_ENV) {
    return process.env.NODE_ENV;
  }

  // For browser environments, we can check for development indicators
  if (typeof window !== 'undefined') {
    // Docusaurus typically sets NODE_ENV in the build process
    // For now, default to development
    return 'development';
  }

  return 'development';
};

const currentEnv = getEnvironment();
export const apiConfig = API_CONFIG[currentEnv] || API_CONFIG.development;

// Export a function to get config based on environment
export const getApiConfig = (env = currentEnv) => {
  return API_CONFIG[env] || API_CONFIG.development;
};

export default apiConfig;