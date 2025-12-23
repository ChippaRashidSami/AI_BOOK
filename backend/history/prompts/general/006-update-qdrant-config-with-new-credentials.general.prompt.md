---
id: "006"
title: "Update Qdrant configuration with new API key and endpoint"
stage: "general"
date_iso: "2025-12-23"
surface: "agent"
model: "Qwen"
feature: "none"
branch: "main"
user: "user"
command: "update Qdrant configuration"
labels:
  - "backend"
  - "embedding-pipeline"
  - "qdrant"
  - "configuration"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "F:/hack/backend/.env"
  - "F:/hack/backend/src/utils/vector_store.py"
tests_yaml: []
---

# Update Qdrant configuration with new API key and endpoint

## User Prompt
ok

NOw i am giving you new quadrant api key and end point use these to create cluster"""endpoint=b9435990ea.us-east4-0.gcp.cloud.qdrant.io"


from qdrant_client import QdrantClient

qdrant_client = QdrantClient(
    url="https://b0aebf53-0a67-4ec6-9b79-5ab9435990ea.us-east4-0.gcp.cloud.qdrant.io:6333", 
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.KhsfzjnJxvA4CIRlq8xp_aCvwH2cJFs_ziL6dCxG5cU""https://b0aebf53-0a67-4ec6-9b79-5a)"""

## Assistant Response
Successfully updated the .env file with the new Qdrant configuration:

- QDRANT_HOST: "https://b0aebf53-0a67-4ec6-9b79-5ab9435990ea.us-east4-0.gcp.cloud.qdrant.io:6333"
- QDRANT_API_KEY: "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.KhsfzjnJxvA4CIRlq8xp_aCvwH2cJFs_ziL6dCxG5cU"
- QDRANT_URL: "https://b0aebf53-0a67-4ec6-9b79-5ab9435990ea.us-east4-0.gcp.cloud.qdrant.io:6333"
- QDRANT_ENDPOINT: "https://b0aebf53-0a67-4ec6-9b79-5ab9435990ea.us-east4-0.gcp.cloud.qdrant.io"
- CLUSTER_ID: "b0aebf53-0a67-4ec6-9b79-5ab9435990ea"
- CLUSTER_NAME: "rashid1"

Attempted to test the new configuration by directly setting environment variables in Python and running the embedding pipeline, but the connection appears to be hanging or taking too long, suggesting potential issues with the new Qdrant endpoint or API key.

## Outcome
Successfully updated the configuration with new Qdrant credentials. The connection test with the new configuration is inconclusive due to hanging connection attempts.