"""
API Endpoints for the Embedding Pipeline Service

This module implements the API endpoints for the embedding pipeline service.
"""
from fastapi import FastAPI, BackgroundTasks, HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from typing import List, Optional
import uuid
import asyncio
from datetime import datetime
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from src.utils.models import ProcessUrlResponse, JobStatusResponse, QueryResponse, CollectionsListResponse, ProcessUrlRequest, QueryRequest
from src.utils.crawler import get_all_urls
from src.utils.content_extractor import extract_text_from_url
from src.utils.chunker import chunk_text_by_tokens
from src.utils.embedder import embed
from src.utils.vector_store import create_collection, save_chunk_to_qdrant, execute_to_qdrant
from src.utils.config import validate_config


# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)
app = FastAPI(title="Embedding Pipeline API", version="1.0.0")
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)


@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler for API errors."""
    return JSONResponse(
        status_code=500,
        content={
            "error": "internal_error",
            "message": "An internal error occurred"
        }
    )


@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """HTTP exception handler for API errors."""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": "http_error",
            "message": exc.detail
        }
    )


# In-memory storage for job status (in production, use a database)
jobs = {}


class ProcessUrlRequest(BaseModel):
    url: str
    collection_name: str = "rag_embeddings"
    chunk_size: int = 1000
    batch_size: int = 10


@app.post("/process-url", response_model=ProcessUrlResponse)
@limiter.limit("5/minute")  # Limit to 5 requests per minute per IP
async def process_url(request: ProcessUrlRequest, background_tasks: BackgroundTasks):
    """Process a Docusaurus-based GitHub Pages URL and generate embeddings."""
    # Validate configuration
    if not validate_config():
        raise HTTPException(status_code=500, detail="Configuration validation failed")

    # Validate URL format
    if not request.url or not request.url.startswith(('http://', 'https://')):
        raise HTTPException(status_code=400, detail="Invalid URL provided")

    job_id = str(uuid.uuid4())

    # Create job record
    jobs[job_id] = {
        "id": job_id,
        "url": request.url,
        "status": "processing",
        "created_at": datetime.utcnow(),
        "documents_processed": 0,
        "documents_failed": 0,
        "embeddings_generated": 0
    }

    # Add background task to process the URL
    background_tasks.add_task(process_url_task, job_id, request.url, request.collection_name, request.chunk_size)

    response = ProcessUrlResponse(
        job_id=job_id,
        status="processing",
        url=request.url,
        estimated_completion=None,  # Would calculate based on content size in a real implementation
        message=f"Processing started for {request.url}"
    )

    return response


async def process_url_task(job_id: str, url: str, collection_name: str, chunk_size: int):
    """Background task to process the URL."""
    try:
        # Update job status
        jobs[job_id]["status"] = "in_progress"
        jobs[job_id]["started_at"] = datetime.utcnow()
        
        # Step 1: Get all URLs from the site
        urls = get_all_urls(url)
        jobs[job_id]["total_documents_count"] = len(urls)
        
        # Step 2: Create collection if it doesn't exist
        create_collection(collection_name)
        
        # Step 3: Process each URL
        for i, page_url in enumerate(urls):
            try:
                # Extract content from the page
                content_data = extract_text_from_url(page_url)
                
                if content_data:
                    # Chunk the content
                    chunks = chunk_text_by_tokens(content_data['content'], max_tokens=chunk_size)
                    
                    # Generate embeddings for chunks
                    chunk_texts = [chunk['content'] for chunk in chunks]
                    embeddings = embed(chunk_texts)
                    
                    # Save embeddings to Qdrant
                    for j, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                        chunk_data = {
                            **content_data,
                            'chunk_index': j,
                            'position_in_document': chunk['start_pos']
                        }
                        
                        save_chunk_to_qdrant(collection_name, chunk_data, embedding)
                    
                    jobs[job_id]["embeddings_generated"] += len(embeddings)
                    jobs[job_id]["documents_processed"] += 1
                else:
                    jobs[job_id]["documents_failed"] += 1
                    
            except Exception as e:
                jobs[job_id]["documents_failed"] += 1
                print(f"Error processing {page_url}: {str(e)}")
        
        # Update job status to completed
        jobs[job_id]["status"] = "completed"
        jobs[job_id]["completed_at"] = datetime.utcnow()
        
    except Exception as e:
        jobs[job_id]["status"] = "failed"
        jobs[job_id]["error_message"] = str(e)


@app.get("/jobs/{job_id}", response_model=JobStatusResponse)
@limiter.limit("10/minute")  # Limit to 10 requests per minute per IP
async def get_job_status(job_id: str):
    """Check the status of a processing job."""
    job = jobs.get(job_id)

    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    return JobStatusResponse(
        job_id=job["id"],
        status=job["status"],
        url=job["url"],
        documents_processed=job["documents_processed"],
        documents_failed=job["documents_failed"],
        embeddings_generated=job["embeddings_generated"],
        started_at=job.get("started_at"),
        completed_at=job.get("completed_at"),
        message=f"Job {job['status']}"
    )


class QueryRequest(BaseModel):
    query_text: str
    collection_name: str = "rag_embeddings"
    limit: int = 5


@app.post("/query", response_model=QueryResponse)
@limiter.limit("20/minute")  # Limit to 20 requests per minute per IP
async def query_embeddings(request: QueryRequest):
    """Search for relevant content using semantic similarity."""
    # This would actually implement the search functionality
    # For now, return a mock response
    results = [
        {
            "document_id": "doc-123",
            "url": "https://example.com/rag-intro",
            "title": "Introduction to RAG Systems",
            "content_snippet": "Retrieval-Augmented Generation (RAG) combines...",
            "similarity_score": 0.87
        }
    ]

    return QueryResponse(
        query=request.query_text,
        results=results[:request.limit]
    )


@app.get("/collections", response_model=CollectionsListResponse)
@limiter.limit("15/minute")  # Limit to 15 requests per minute per IP
async def list_collections():
    """List all available embedding collections."""
    # This would actually list collections from Qdrant
    # For now, return a mock response
    collections = [
        {
            "name": "my-book-embeddings",
            "document_count": 120,
            "created_at": datetime.utcnow()
        }
    ]

    return CollectionsListResponse(collections=collections)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)