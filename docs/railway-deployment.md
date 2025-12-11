# Railway Deployment Environment Variables

To successfully deploy the RAG Chatbot backend to Railway, you need to set the following environment variables:

## Required Environment Variables

1. **OPENAI_API_KEY**
   - Description: Your OpenAI API key for accessing GPT models and embeddings
   - Format: A string starting with "sk-" followed by alphanumeric characters
   - Example: `sk-proj-youractualapikeyhere`

2. **QDRANT_URL**
   - Description: The URL of your Qdrant vector database instance
   - Format: A complete URL (e.g., https://your-cluster-url.qdrant.tech:6333)
   - Example: `https://your-cluster.qdrant.tech:6333`

3. **QDRANT_API_KEY**
   - Description: The API key for your Qdrant vector database
   - Format: A string of alphanumeric characters
   - Example: `youractualqdrantapikey`

## How to Set Environment Variables in Railway

1. Go to your Railway project dashboard
2. Click on your backend service
3. Navigate to the "Variables" tab
4. Add each variable with its corresponding value
5. Redeploy your service after adding the variables

## Troubleshooting

- If you get 502 errors after setting variables, check the deployment logs in Railway
- Make sure your Qdrant database is accessible and has the required collection
- The application creates a collection named "book_chunks" on startup
- Verify that your OpenAI API key has sufficient quota/credits

## Important Notes

- Never commit these keys to version control
- The Qdrant database must be pre-populated with book content for the RAG system to work
- The application will fail to start if these variables are not set