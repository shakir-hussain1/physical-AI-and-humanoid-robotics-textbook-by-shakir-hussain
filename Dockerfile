FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    gcc \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements
COPY backend/requirements.txt .

# Upgrade pip and install Python dependencies
RUN pip install --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY backend/src/ src/
COPY backend/.env* ./
COPY space_env.txt ./

# Expose port (7860 for Hugging Face, 8000 for others)
EXPOSE 7860

# Run the application
# For Hugging Face: use port 7860
# For local/other: change to 8000
CMD ["python", "-m", "uvicorn", "src.app:app", "--host", "0.0.0.0", "--port", "7860"]
