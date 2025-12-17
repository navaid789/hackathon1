# Deployment Guide for RAGBot Backend

## Prerequisites

1. Install Vercel CLI:
```bash
npm install -g vercel
```

2. Make sure you have a Vercel account (sign up at https://vercel.com if you don't have one)

## Step 1: Deploy the Backend

1. Navigate to the backend directory:
```bash
cd backend
```

2. Deploy to Vercel:
```bash
vercel --prod
```

3. During the deployment, you'll be prompted to set environment variables:
   - `OPENAI_API_KEY` = your OpenAI API key
   - `QDRANT_API_KEY` = your Qdrant API key
   - `QDRANT_URL` = your Qdrant Cloud URL
   - `COHERE_API_KEY` = your Cohere API key
   - `DOCUSAURUS_BASE_URL` = https://hackathon1-five-taupe.vercel.app/ (or your frontend URL)

4. After deployment completes, note the deployment URL (e.g., `https://hackathon1-backend-username.vercel.app`)

## Step 2: Update Frontend Configuration

1. Navigate back to the project root:
```bash
cd ..
```

2. Update the API configuration with your actual backend URL:
```javascript
// In frontend/src/config/apiConfig.js
const DEFAULT_BASE_URLS = {
  development: 'http://127.0.0.1:8000',
  production: 'https://your-actual-backend-url.vercel.app',  // Replace with the URL from Step 1
  test: 'http://127.0.0.1:8000',
};
```

3. Update the Vercel rewrite configuration:
```json
// In vercel.json
{
  "rewrites": [
    {
      "source": "/api/:path*",
      "destination": "https://your-actual-backend-url.vercel.app/:path*"  // Replace with actual URL
    }
  ]
}
```

## Step 3: Rebuild and Deploy the Frontend

1. Rebuild the frontend:
```bash
npm run build
```

2. Deploy the frontend to Vercel:
```bash
vercel --prod
```

## Step 4: Verify the Deployment

1. Visit your deployed frontend URL
2. The RAGBot chat widget should now appear and be functional
3. Test by clicking the chat icon in the bottom-right corner

## Troubleshooting

- If the chat widget still doesn't appear, check the browser's developer console for errors
- Ensure all API keys are correctly set in the backend environment variables
- Verify that the backend URL in the frontend configuration matches the deployed backend URL
- Check that the Vercel rewrite rules are correctly routing /api requests to your backend