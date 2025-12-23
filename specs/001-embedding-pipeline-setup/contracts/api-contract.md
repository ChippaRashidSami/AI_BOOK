# API Contract: Embedding Pipeline Service

## Endpoints

### Process URL for Embeddings

**POST** `/process-url`

Process a Docusaurus-based GitHub Pages URL and generate embeddings.

#### Request

```json
{
  "url": "https://your-book.github.io",
  "collection_name": "my-book-embeddings",
  "chunk_size": 1000,
  "batch_size": 10
}
```

#### Response

```json
{
  "job_id": "unique-job-identifier",
  "status": "processing",
  "url": "https://your-book.github.io",
  "estimated_completion": "2023-10-05T14:48:00Z",
  "message": "Processing started for 45 documents"
}
```

#### Error Response

```json
{
  "error": "invalid_url",
  "message": "The provided URL is not accessible or not a valid Docusaurus site"
}
```

### Check Job Status

**GET** `/jobs/{job_id}`

Check the status of a processing job.

#### Response

```json
{
  "job_id": "unique-job-identifier",
  "status": "completed",
  "url": "https://your-book.github.io",
  "documents_processed": 45,
  "documents_failed": 0,
  "embeddings_generated": 120,
  "started_at": "2023-10-05T14:45:00Z",
  "completed_at": "2023-10-05T14:48:00Z",
  "message": "Successfully processed 45 documents and generated 120 embeddings"
}
```

### Query Embeddings

**POST** `/query`

Search for relevant content using semantic similarity.

#### Request

```json
{
  "query_text": "How do I implement RAG with vector databases?",
  "collection_name": "my-book-embeddings",
  "limit": 5
}
```

#### Response

```json
{
  "query": "How do I implement RAG with vector databases?",
  "results": [
    {
      "document_id": "doc-123",
      "url": "https://your-book.github.io/rag-intro",
      "title": "Introduction to RAG Systems",
      "content_snippet": "Retrieval-Augmented Generation (RAG) combines...",
      "similarity_score": 0.87
    }
  ]
}
```

### List Collections

**GET** `/collections`

List all available embedding collections.

#### Response

```json
[
  {
    "name": "my-book-embeddings",
    "document_count": 120,
    "created_at": "2023-10-05T14:48:00Z"
  }
]
```