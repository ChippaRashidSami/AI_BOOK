---
id: "002"
title: "Run embedding pipeline on documentation site"
stage: "general"
date_iso: "2025-12-23"
surface: "agent"
model: "Qwen"
feature: "none"
branch: "main"
user: "user"
command: "run backend script with URL"
labels:
  - "backend"
  - "embedding-pipeline"
  - "content-extraction"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "F:/hack/backend/src/main.py"
tests_yaml: []
---

# Run embedding pipeline on documentation site

## User Prompt
now you can run this in python go to backend folder with cd backend and run this project with  run main.py and""" for pages get it from the url i am giving https://hackathon-f96z.vercel.app/sitemap.xml"""

## Assistant Response
Successfully ran the embedding pipeline on the documentation site at https://hackathon-f96z.vercel.app/. The pipeline:

1. Discovered 21 pages from the site
2. Successfully extracted content from all pages
3. Chunked and generated embeddings for the content
4. Encountered a 403 Forbidden error when trying to store embeddings in Qdrant Cloud due to authentication issues

The content extraction and embedding generation parts worked correctly, processing documentation about ROS 2, Digital Twin (Gazebo & Unity), AI-Robot Brain (NVIDIA Isaac), and Vision-Language-Action.

## Outcome
Successfully extracted content from 21 pages of the documentation site, but failed to store embeddings in Qdrant due to authentication issues.