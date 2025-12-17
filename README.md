# Physical AI & Humanoid Robotics: Embodied Intelligence from Simulation to Reality

This is an academic textbook project on Physical AI, Robotics, and Vision-Language-Action systems with an integrated RAG (Retrieval-Augmented Generation) chatbot.

## Features

- Complete academic textbook on Physical AI and Humanoid Robotics
- Interactive RAGBot chatbot for asking questions about the content
- Integration with OpenAI, Qdrant, and Cohere for semantic search and AI responses
- Docusaurus-based documentation site

## RAGBot Chat Functionality

The RAGBot chatbot is integrated into the documentation, allowing users to ask questions about the textbook content and receive contextually relevant responses based on the book's content.

### How to Deploy the RAGBot

To make the RAGBot functionality work, you need to deploy both the frontend and backend services:

1. **Deploy the backend API** (required for RAGBot to function):
   - Navigate to the `backend` directory
   - Deploy using Vercel CLI: `vercel --prod`
   - Set required environment variables during deployment:
     - `OPENAI_API_KEY`
     - `QDRANT_API_KEY`
     - `QDRANT_URL`
     - `COHERE_API_KEY`

2. **Deploy the frontend**:
   - From the project root, run: `npm run build`
   - Deploy using Vercel CLI: `vercel --prod`

For complete deployment instructions, refer to the [DEPLOYMENT_GUIDE.md](./DEPLOYMENT_GUIDE.md) file.

## Development

To run the project locally:

1. Install dependencies:
```bash
npm install
```

2. Build the project:
```bash
npm run build
```

3. Serve the built project:
```bash
npm run serve
```

## License

MIT License