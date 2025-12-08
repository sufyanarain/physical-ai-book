FROM python:3.12-slim

WORKDIR /app

# Copy requirements and app code
COPY requirements.txt .
COPY app ./app

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Expose port (Railway will set PORT env variable)
EXPOSE $PORT

# Start command
CMD uvicorn app.main:app --host 0.0.0.0 --port $PORT
