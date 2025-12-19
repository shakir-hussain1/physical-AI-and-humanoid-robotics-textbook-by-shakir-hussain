#!/bin/bash

# Load environment variables
set -a
[ -f space_env.txt ] && source space_env.txt
set +a

# Install dependencies
cd backend
pip install -r requirements.txt

# Start backend
python -m uvicorn src.app:app --host 0.0.0.0 --port 7860
