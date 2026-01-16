# RAG Chatbot System - Requirements & Dependencies

## 1. System Requirements

### Hardware Requirements

**Minimum:**
- CPU: 4 cores
- RAM: 8 GB
- Storage: 10 GB (for project + dependencies)
- Network: 10 Mbps internet connection

**Recommended:**
- CPU: 8+ cores
- RAM: 16+ GB
- Storage: 20 GB SSD
- Network: 100+ Mbps

### Operating System

**Supported:**
- Windows 10/11
- macOS 10.15+
- Linux (Ubuntu 20.04+, Debian 11+)

### Python Version

**Required:** Python 3.8+
**Tested with:** Python 3.11, 3.12

---

## 2. Backend Dependencies

### Core Framework

| Package | Version | Purpose |
|---------|---------|---------|
| `fastapi` | >=0.100.0 | Web framework |
| `uvicorn[standard]` | >=0.23.0 | ASGI server |
| `pydantic` | >=2.0.0 | Data validation |
| `python-dotenv` | >=1.0.0 | Environment variables |

### LLM & Embeddings

| Package | Version | Purpose |
|---------|---------|---------|
| `openai` | >=1.0.0 | OpenAI GPT-4 API |
| `qdrant-client` | >=2.7.0 | Vector database client |
| `cohere` | >=4.0.0 | Cohere embeddings API |

### HTTP & Async

| Package | Version | Purpose |
|---------|---------|---------|
| `httpx` | >=0.24.0 | Async HTTP client |
| `aiohttp` | >=3.8.0 | Async HTTP library |

### Logging & Monitoring

| Package | Version | Purpose |
|---------|---------|---------|
| `python-json-logger` | >=2.0.7 | JSON logging formatter |

### Testing (Optional)

| Package | Version | Purpose |
|---------|---------|---------|
| `pytest` | >=7.4.0 | Testing framework |
| `pytest-asyncio` | >=0.21.0 | Async test support |
| `pytest-cov` | >=4.1.0 | Coverage reporting |

### Development (Optional)

| Package | Version | Purpose |
|---------|---------|---------|
| `black` | >=23.0.0 | Code formatter |
| `isort` | >=5.12.0 | Import sorting |
| `flake8` | >=6.0.0 | Linting |
| `mypy` | >=1.4.0 | Type checking |

### Complete Requirements File

```
# Core dependencies
fastapi>=0.100.0
uvicorn[standard]>=0.23.0
pydantic>=2.0.0
python-dotenv>=1.0.0

# Data processing
numpy>=1.24.0
pandas>=2.0.0

# Vector database and embeddings
qdrant-client>=2.7.0
openai>=1.0.0
cohere>=4.0.0

# HTTP and async
httpx>=0.24.0
aiohttp>=3.8.0

# Logging and monitoring
python-json-logger>=2.0.7

# Testing
pytest>=7.4.0
pytest-asyncio>=0.21.0
pytest-cov>=4.1.0

# Development
black>=23.0.0
isort>=5.12.0
flake8>=6.0.0
mypy>=1.4.0
```

**File Location:** `backend/requirements.txt`

**Installation:**
```bash
# Create and activate virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r backend/requirements.txt
```

---

## 3. Frontend Dependencies

### Node.js Environment

**Required:** Node.js 18+
**NPM:** 9+

### Frontend Packages

Installed via `npm install` in `frontend/` directory:

| Package | Version | Purpose |
|---------|---------|---------|
| `react` | ^18.2.0 | UI library |
| `react-dom` | ^18.2.0 | React DOM |
| `docusaurus` | ^3.6.3 | Static site generator |

### Development Dependencies

| Package | Purpose |
|---------|---------|
| `@docusaurus/preset-classic` | Docusaurus preset |
| `@docusaurus/module-ideal-image` | Image optimization |
| `mermaid` | Diagram support |

**Installation:**
```bash
cd frontend
npm install
npm start  # Start development server
```

---

## 4. External Services & APIs

### OpenAI API

**Service:** OpenAI GPT-4 API
**Endpoint:** `https://api.openai.com/v1`
**Authentication:** API key (environment variable)
**Requirements:**
- Active OpenAI account
- Paid plan with credits
- API key: `OPENAI_API_KEY`
- Rate limits: Standard tier

**Pricing:**
- GPT-4o-mini: ~$0.00015 per input token, $0.0006 per output token
- Estimated cost per query: ~$0.01-0.05

**Documentation:** https://platform.openai.com/docs

---

### Qdrant Vector Database

**Service:** Qdrant Cloud (or self-hosted)
**Endpoint:** Cloud-hosted (eu-west3-0.gcp.cloud.qdrant.io)
**Authentication:** API key (environment variable)
**Requirements:**
- Active Qdrant Cloud account
- Collection: `textbook_embeddings` (26 vectors)
- Vector dimension: 1024
- API key: `QDRANT_API_KEY`

**Pricing:**
- Cloud: Free tier available (small collections)
- Production: Tiered pricing based on storage/requests

**Documentation:** https://qdrant.tech/documentation/

---

### Cohere Embeddings API

**Service:** Cohere API
**Endpoint:** `https://api.cohere.com`
**Authentication:** API key (environment variable)
**Requirements:**
- Active Cohere account
- Model: `embed-english-v3.0`
- Input: Query text (≤2048 chars)
- Output: 1024-dimensional embedding
- API key: `COHERE_API_KEY`

**Pricing:**
- Free tier: Limited requests
- Production: $0.10 per 1,000 embeddings

**Documentation:** https://docs.cohere.com/

---

## 5. Environment Variables

### Required Variables

```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxxxx  # OpenAI API key (required)
OPENAI_MODEL=gpt-4o-mini                # Model name (required)

# Qdrant Configuration
QDRANT_URL=https://xxx.qdrant.io        # Qdrant endpoint (required)
QDRANT_API_KEY=xxxxxxxxxxxxxxxxxxxxxxxx # Qdrant API key (required)
QDRANT_COLLECTION=textbook_embeddings   # Collection name (required)

# Cohere Configuration
COHERE_API_KEY=xxxxxxxxxxxxxxxxxxxxxxxx # Cohere API key (required)
```

### Optional Variables

```bash
# FastAPI Settings
FASTAPI_HOST=0.0.0.0                    # Server host (default: 0.0.0.0)
FASTAPI_PORT=8000                       # Server port (default: 8000)
FASTAPI_ENV=development                 # Environment (default: development)

# Logging
LOG_LEVEL=INFO                           # Log level (default: INFO)

# Feature Flags
ENABLE_CONTEXT_RESTRICTED_MODE=true     # Restrict to context only
ENABLE_FACT_CHECKING_GRADE=true         # Enable fact checking
```

### Environment File Location

**File:** `.env` (root directory)

**Setup:**
```bash
# Copy example to create .env
cp .env.example .env

# Edit with your API keys
# DO NOT commit .env to git (add to .gitignore)
```

---

## 6. Development Tools (Optional)

### IDE/Editor

**Recommended:**
- Visual Studio Code (free)
- PyCharm Professional/Community (free)
- Vim/Neovim (advanced)

### VS Code Extensions

```
- Python (Microsoft)
- Pylance (Microsoft)
- FastAPI (Kevinjoseph)
- REST Client (Huachao Mao)
- Thunder Client (Thunder Client)
```

### Git

```bash
# Required for version control
git --version  # Verify installation
```

### Postman/Thunder Client

For API testing:
- Postman (free desktop app)
- Thunder Client (VS Code extension)
- curl (command-line)

### Browser

For frontend testing:
- Chrome/Chromium (latest)
- Firefox (latest)
- Safari (macOS)
- Edge (Windows)

---

## 7. Installation Guide

### Step 1: Prerequisites Check

```bash
# Check Python version
python --version  # Should be 3.8+

# Check Node.js version
node --version   # Should be 18+
npm --version    # Should be 9+

# Check git
git --version
```

### Step 2: Backend Setup

```bash
# Navigate to project root
cd E:\Physical-AI-and-Humanoid-Robotics

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r backend/requirements.txt
```

### Step 3: Environment Configuration

```bash
# Create .env file in project root
cat > .env << 'EOF'
OPENAI_API_KEY=your-api-key-here
OPENAI_MODEL=gpt-4o-mini
QDRANT_URL=https://your-qdrant-instance.qdrant.io
QDRANT_API_KEY=your-qdrant-key-here
QDRANT_COLLECTION=textbook_embeddings
COHERE_API_KEY=your-cohere-key-here
FASTAPI_HOST=0.0.0.0
FASTAPI_PORT=8000
LOG_LEVEL=INFO
EOF
```

### Step 4: Frontend Setup

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Start development server (optional)
npm start
```

### Step 5: Verify Installation

```bash
# Test Python imports
python -c "import fastapi; import openai; import qdrant_client; print('✓ All imports OK')"

# Test API startup (from project root with venv activated)
cd backend
python -m uvicorn src.api.main:app --reload
# Should see: "Uvicorn running on http://0.0.0.0:8000"
```

---

## 8. Troubleshooting

### Python Version Issues

**Problem:** `python` command not found
**Solution:**
```bash
# Use python3 instead
python3 --version
python3 -m venv venv
```

### Virtual Environment Issues

**Problem:** Cannot activate venv
**Solution:**
- Windows: Use `venv\Scripts\activate.bat` instead of `.ps1`
- macOS/Linux: Use `source venv/bin/activate`

### Dependency Installation Issues

**Problem:** `pip install` fails
**Solution:**
```bash
# Upgrade pip first
python -m pip install --upgrade pip

# Install with verbose output
pip install -r backend/requirements.txt -v
```

### API Key Issues

**Problem:** OpenAI API returns 401 (unauthorized)
**Solution:**
- Check `OPENAI_API_KEY` is correct and has no extra spaces
- Verify API key in `.env` file
- Ensure API key has active credits

**Problem:** Qdrant connection fails
**Solution:**
- Check `QDRANT_URL` is accessible
- Verify `QDRANT_API_KEY` is correct
- Check network connectivity

### Port Already in Use

**Problem:** `Address already in use` error on port 8000
**Solution:**
```bash
# Use different port
python -m uvicorn src.api.main:app --port 8001

# Or kill existing process:
# Windows: taskkill /F /IM python.exe
# macOS/Linux: kill -9 $(lsof -t -i :8000)
```

---

## 9. Dependency Compatibility

### Python Version Compatibility

| Python | Status | Notes |
|--------|--------|-------|
| 3.8 | ✅ Supported | Minimum version |
| 3.9 | ✅ Supported | Recommended for stability |
| 3.10 | ✅ Supported | |
| 3.11 | ✅ Supported | **Recommended** |
| 3.12 | ✅ Tested | Latest stable |
| 3.13 | ⚠️ Untested | May work |

### Known Compatibility Issues

None currently reported. All packages are actively maintained and compatible.

---

## 10. Security Considerations

### API Key Management

```bash
# NEVER commit .env to git
echo ".env" >> .gitignore

# NEVER log API keys
# Remove keys from any debug output

# NEVER hardcode keys in code
# Always use environment variables
```

### Dependency Security

```bash
# Check for security vulnerabilities
pip install safety
safety check

# Keep dependencies updated
pip list --outdated
pip install --upgrade package-name
```

### HTTPS/TLS (Production)

For production deployment:
- Use HTTPS with proper SSL certificates
- Configure CORS properly
- Implement rate limiting
- Add authentication if needed

---

## 11. Scalability Considerations

### For Production

1. **Backend:**
   - Use production ASGI server (gunicorn + uvicorn)
   - Run multiple worker processes
   - Use load balancer (nginx, HAProxy)
   - Monitor with APM tool

2. **Database:**
   - Use managed Qdrant Cloud
   - Configure backups
   - Monitor collection size
   - Plan for scaling

3. **Frontend:**
   - Static site hosting (GitHub Pages, Netlify, Vercel)
   - CDN for assets
   - Cache optimization

4. **API Keys:**
   - Use secrets manager (AWS Secrets Manager, HashiCorp Vault)
   - Rotate keys regularly
   - Implement API rate limiting

---

## 12. Update & Maintenance

### Regular Updates

```bash
# Check for outdated packages
pip list --outdated

# Update specific package
pip install --upgrade package-name

# Update all packages
pip install --upgrade -r backend/requirements.txt
```

### Breaking Changes

Monitor these packages for breaking changes:
- FastAPI (usually backwards compatible)
- Pydantic v2 (breaking changes from v1)
- OpenAI SDK (periodic breaking changes)

---

## 13. Support & Resources

### Documentation Links

- **FastAPI:** https://fastapi.tiangolo.com/
- **OpenAI:** https://platform.openai.com/docs/
- **Qdrant:** https://qdrant.tech/documentation/
- **Cohere:** https://docs.cohere.com/
- **React:** https://react.dev/

### Community Support

- Stack Overflow: Tag questions with `fastapi`, `openai`, `qdrant`
- GitHub Issues: Report bugs in respective repositories
- Discord Communities: FastAPI, OpenAI, Qdrant Discord servers

---

**Version:** 1.0
**Last Updated:** 2025-12-27
**Status:** CURRENT
