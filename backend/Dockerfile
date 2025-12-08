FROM python:3.12-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and app code
COPY requirements.txt .
COPY app ./app

# Install dependencies and clean up
RUN pip install --no-cache-dir -r requirements.txt && \
    rm -rf /root/.cache/pip

# Expose port (Railway will set PORT env variable)
EXPOSE $PORT

# Start command
CMD uvicorn app.main:app --host 0.0.0.0 --port $PORT
