# Embedding Pipeline Documentation

## Overview

The Embedding Pipeline is a Python-based tool that ingests content from Docusaurus-based GitHub Pages, generates semantic embeddings using Cohere models, and stores them in Qdrant Cloud for use in RAG (Retrieval-Augmented Generation) systems.

## Features

- Crawls Docusaurus-based GitHub Pages sites to discover all content
- Extracts clean text content while filtering out navigation and non-content elements
- Chunks content appropriately for embedding model limits
- Generates semantic embeddings using Cohere's embedding models
- Stores embeddings with metadata in Qdrant Cloud
- Provides API endpoints for processing and querying
- Implements error handling, retry mechanisms, and rate limiting

## Prerequisites

- Python 3.11+
- uv package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. Clone the repository
2. Install dependencies using uv:
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install -r requirements.txt
   ```
3. Create a `.env` file based on `.env.example`:
   ```bash
   cp .env.example .env
   ```
4. Update `.env` with your API keys

## Usage

### Command Line Interface

Process a Docusaurus-based GitHub Pages site:

```bash
python src/main.py --url https://your-book.github.io
```

With custom parameters:

```bash
python src/main.py --url https://your-book.github.io --collection-name my-book-embeddings --chunk-size 512
```

### API Endpoints

#### Process URL for Embeddings

`POST /process-url`

Process a Docusaurus-based GitHub Pages URL and generate embeddings.

Request:
```json
{
  "url": "https://your-book.github.io",
  "collection_name": "my-book-embeddings",
  "chunk_size": 1000,
  "batch_size": 10
}
```

Response:
```json
{
  "job_id": "unique-job-identifier",
  "status": "processing",
  "url": "https://your-book.github.io",
  "estimated_completion": "2023-10-05T14:48:00Z",
  "message": "Processing started for 45 documents"
}
```

#### Check Job Status

`GET /jobs/{job_id}`

Check the status of a processing job.

#### Query Embeddings

`POST /query`

Search for relevant content using semantic similarity.

Request:
```json
{
  "query_text": "How do I implement RAG with vector databases?",
  "collection_name": "my-book-embeddings",
  "limit": 5
}
```

#### List Collections

`GET /collections`

List all available embedding collections.

## Architecture

### Components

1. **Crawler** (`src/utils/crawler.py`): Discovers all pages on a Docusaurus site
2. **Content Extractor** (`src/utils/content_extractor.py`): Extracts clean text from web pages
3. **Chunker** (`src/utils/chunker.py`): Splits content into appropriately sized chunks
4. **Embedder** (`src/utils/embedder.py`): Generates semantic embeddings using Cohere
5. **Vector Store** (`src/utils/vector_store.py`): Stores embeddings in Qdrant Cloud
6. **API** (`src/api.py`): Provides REST endpoints for the service

### Data Flow

1. Crawl the provided URL to discover all pages
2. Extract text content from each page
3. Chunk the content to fit within token limits
4. Generate embeddings using Cohere
5. Store the embeddings in Qdrant with metadata
6. Provide a summary of the processing results

## Configuration

The pipeline can be configured through command-line arguments or environment variables:

- `--url`: The GitHub Pages URL containing the Docusaurus-based book
- `--collection-name`: Name of the Qdrant collection to store embeddings
- `--chunk-size`: Size of text chunks for embedding (in tokens)
- `--batch-size`: Number of chunks to process in each batch

Environment variables are loaded from `.env`:
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_HOST`: Your Qdrant Cloud endpoint

## Error Handling

The pipeline implements several error handling mechanisms:
- Retry logic for transient failures
- Rate limiting for API calls
- Content validation before embedding
- Checksum-based change detection for incremental updates
- Comprehensive logging for debugging

## Performance

The pipeline is designed to:
- Process 100 pages within 30 minutes
- Achieve 99% embedding generation success rate when Cohere API is available
- Complete content extraction for 100 pages within 5 minutes
- Achieve 95% URL ingestion success rate