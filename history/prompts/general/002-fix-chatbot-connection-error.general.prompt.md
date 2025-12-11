---
id: "002"
title: "Fix Chatbot Connection Error"
stage: "general"
date: "2025-12-11"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "master"
user: "Shakir"
command: "fix chatbot connection error"
labels: ["bug-fix", "connection", "api", "railway"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "src/components/ChatWidget.js"
tests: []
---

## Summary
Fixed the "Error connecting to server" issue in the deployed chatbot by correcting the API URL configuration.

## Changes Made
1. Added proper protocol (`https://`) to the API_URL in ChatWidget.js
2. Improved error handling to provide more specific error messages
3. Ensured proper CORS configuration for Railway deployment

## Technical Details
- Fixed missing protocol in API_URL: `https://physical-ai-and-humanoid-robotics-textbook-by-sh-production.up.railway.app`
- Enhanced error handling with HTTP status checking
- Maintained existing CORS middleware in FastAPI backend

## Files Modified
- `src/components/ChatWidget.js` - Fixed API URL and improved error handling

## Expected Outcome
The chatbot should now successfully connect to the Railway-deployed backend and provide answers instead of connection errors.

## Validation
- API endpoint should be accessible via the corrected URL
- CORS should work properly between GitHub Pages frontend and Railway backend
- Error messages should be more descriptive if issues persist