#!/bin/bash
# Deployment script for RAGBot frontend and backend

echo "RAGBot Deployment Script"
echo "========================"

echo "Step 1: Building frontend..."
npm run build

if [ $? -ne 0 ]; then
    echo "❌ Frontend build failed. Please fix the errors and try again."
    exit 1
fi

echo "✅ Frontend built successfully"

echo ""
echo "Step 2: Deploy backend to Vercel..."
echo "Please navigate to the backend directory and run: cd backend && vercel --prod"
echo "Make sure to set the required environment variables during deployment:"
echo "  - OPENAI_API_KEY"
echo "  - QDRANT_API_KEY" 
echo "  - QDRANT_URL"
echo "  - COHERE_API_KEY"
echo ""

echo "Step 3: After backend deployment, update the frontend configuration"
echo "with the deployed backend URL, then deploy the frontend:"
echo "  - Update frontend/src/config/apiConfig.js"
echo "  - Update vercel.json rewrites"
echo "  - Run: vercel --prod from the project root"
echo ""

echo "For detailed instructions, see DEPLOYMENT_GUIDE.md"