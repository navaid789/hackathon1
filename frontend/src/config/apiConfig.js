// frontend/src/config/apiConfig.js
// Browser-safe configuration for API client

// Use constants or environment variables injected by CRA or Vite
// For CRA: variables must be prefixed with REACT_APP_
// For Vite: variables are accessed via import.meta.env.VITE_*

// Default base URLs
const DEFAULT_BASE_URLS = {
  development: 'http://127.0.0.1:8000',  // Backend runs on port 8000
  production: 'https://hackathon1-backend-navaid789.vercel.app',  // Based on GitHub repo name
  test: 'http://127.0.0.1:8000',
};

// Determine current environment in a browser-safe way
const getEnvironment = () => {
  // CRA sets process.env.NODE_ENV, Vite uses import.meta.env.MODE
  if (typeof import.meta !== 'undefined' && import.meta.env?.MODE) {
    return import.meta.env.MODE;
  }

  // Fallback default
  return 'development';
};

const currentEnv = getEnvironment();

// Base URL: first check injected env vars, then fallback to defaults
const BASE_URL =
  (typeof import.meta !== 'undefined' && import.meta.env?.VITE_API_BASE_URL) ||
  DEFAULT_BASE_URLS[currentEnv];

export const apiConfig = {
  baseURL: BASE_URL,
  timeout: currentEnv === 'test' ? 10000 : 30000,
  retries: currentEnv === 'test' ? 1 : 3,
};

// Function to get config for a specific environment
export const getApiConfig = (env = currentEnv) => {
  const baseURL =
    (typeof import.meta !== 'undefined' && import.meta.env?.VITE_API_BASE_URL) ||
    DEFAULT_BASE_URLS[env];

  return {
    baseURL,
    timeout: env === 'test' ? 10000 : 30000,
    retries: env === 'test' ? 1 : 3,
  };
};

export default apiConfig;