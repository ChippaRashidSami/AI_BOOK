"""
Embedding Pipeline Main Module

This module orchestrates the full pipeline for ingesting content from Docusaurus-based GitHub Pages,
generating semantic embeddings using Cohere models, and storing them in Qdrant Cloud.
"""
import argparse
import sys
import time
from typing import List, Dict, Any
from src.utils.crawler import get_all_urls
from src.utils.content_extractor import extract_text_from_url
from src.utils.chunker import chunk_text_by_tokens
from src.utils.embedder import embed
from src.utils.vector_store import create_collection, execute_to_qdrant
from src.utils.config import validate_config, setup_logging
from src.utils.logging_util import get_logger, log_pipeline_event, log_error_event
from src.utils.models import Document


def main():
    """
    Main function to execute the embedding pipeline
    """
    parser = argparse.ArgumentParser(description='Embedding Pipeline for Docusaurus-based books')
    parser.add_argument('--url', type=str, required=True,
                        help='GitHub Pages URL containing the Docusaurus-based book')
    parser.add_argument('--collection-name', type=str, default='rag_embeddings',
                        help='Name of the Qdrant collection to store embeddings')
    parser.add_argument('--chunk-size', type=int, default=512,  # Changed to 512 tokens
                        help='Size of text chunks for embedding (in tokens)')
    parser.add_argument('--batch-size', type=int, default=10,
                        help='Number of chunks to process in each batch')

    args = parser.parse_args()

    # Record start time for the entire pipeline
    start_time = time.time()

    # Setup logging
    setup_logging()
    logger = get_logger("embedding_pipeline")

    print(f"Starting embedding pipeline for URL: {args.url}")
    print(f"Collection name: {args.collection_name}")
    print(f"Chunk size: {args.chunk_size} tokens")
    print(f"Batch size: {args.batch_size}")

    # Validate configuration
    if not validate_config():
        log_error_event(logger, "config_validation", "Configuration validation failed", url=args.url)
        print("Configuration validation failed. Please check your environment variables.")
        sys.exit(1)

    # Create collection in Qdrant
    log_pipeline_event(logger, "collection_setup", f"Creating collection: {args.collection_name}", collection=args.collection_name)
    create_collection(args.collection_name)

    # Step 1: Crawl the provided URL to discover all pages
    log_pipeline_event(logger, "crawl", f"Starting crawl of {args.url}", url=args.url)
    try:
        urls = get_all_urls(args.url)
        print(f"Discovered {len(urls)} pages to process")
        log_pipeline_event(logger, "crawl", f"Discovered {len(urls)} pages", url=args.url, count=len(urls))
    except Exception as e:
        log_error_event(logger, "crawl_error", str(e), url=args.url)
        print(f"Error crawling URLs: {str(e)}")
        sys.exit(1)

    # Step 2: Extract text content from each page
    documents = []
    for i, url in enumerate(urls):
        log_pipeline_event(logger, "extraction", f"Processing page {i+1}/{len(urls)}", url=url)
        try:
            content_data = extract_text_from_url(url)
            if content_data:
                doc = Document(
                    url=content_data['url'],
                    title=content_data['title'],
                    content=content_data['content'],
                    checksum=content_data['checksum']
                )
                documents.append(doc)
                print(f"  Extracted content from: {url} ({len(content_data['content'])} chars)")
            else:
                log_error_event(logger, "extraction_error", f"Failed to extract content", url=url)
                print(f"  Failed to extract content from: {url}")
        except Exception as e:
            log_error_event(logger, "extraction_error", str(e), url=url)
            print(f"  Error extracting content from {url}: {str(e)}")

    print(f"Successfully extracted content from {len(documents)} pages")

    # Step 3: Process each document (chunk and embed)
    total_chunks = 0
    total_embeddings = 0

    for i, doc in enumerate(documents):
        log_pipeline_event(logger, "chunking", f"Chunking document {i+1}/{len(documents)}", url=doc.url)

        # Chunk the content
        chunks = chunk_text_by_tokens(doc.content, max_tokens=args.chunk_size)
        print(f"  Document '{doc.title}' chunked into {len(chunks)} parts")

        # Handle extremely large documents that might exceed embedding model limits
        if len(chunks) > 100:  # Arbitrary threshold for "extremely large"
            print(f"  WARNING: Document '{doc.title}' has {len(chunks)} chunks, which is unusually large.")
            print(f"  Consider adjusting chunk size or checking for document anomalies.")

        # Extract text content from chunks for embedding
        chunk_texts = [chunk['content'] for chunk in chunks]

        # Step 4: Generate embeddings using Cohere
        log_pipeline_event(logger, "embedding", f"Generating embeddings for {len(chunks)} chunks", url=doc.url)
        try:
            embeddings = embed(chunk_texts)
            print(f"  Generated {len(embeddings)} embeddings for document: {doc.title}")
        except Exception as e:
            log_error_event(logger, "embedding_error", str(e), url=doc.url)
            print(f"  Error generating embeddings for {doc.url}: {str(e)}")
            continue  # Skip to next document

        # Step 5: Store the embeddings in Qdrant with metadata
        log_pipeline_event(logger, "storage", f"Storing {len(embeddings)} embeddings", url=doc.url, collection=args.collection_name)

        # Prepare chunk data with document metadata
        chunk_data_list = []
        for j, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            chunk_data = {
                'content': chunk['content'],
                'url': doc.url,
                'title': doc.title,
                'document_id': f"{doc.id or doc.url.split('/')[-1]}_{j}",
                'chunk_index': j,
                'checksum': chunk.get('checksum', ''),
                'hierarchy_level': 'section',
                'parent_document': doc.url,
                'position_in_document': chunk.get('start_pos', 0)
            }
            chunk_data_list.append(chunk_data)

        # Execute bulk storage to Qdrant
        result = execute_to_qdrant(args.collection_name, chunk_data_list, embeddings)
        if result["status"] == "success":
            total_chunks += len(chunks)
            total_embeddings += len(embeddings)
            print(f"  Stored {len(embeddings)} embeddings to Qdrant")
        else:
            log_error_event(logger, "storage_error", result.get("error_message", "Unknown error"),
                           url=doc.url, collection=args.collection_name)
            print(f"  Failed to store embeddings for {doc.url}")

    # Step 6: Provide a summary of the processing results
    end_time = time.time()
    total_duration = end_time - start_time

    print("\n--- Pipeline Summary ---")
    print(f"Total documents processed: {len(documents)}")
    print(f"Total chunks created: {total_chunks}")
    print(f"Total embeddings generated: {total_embeddings}")
    print(f"Embeddings stored in collection: {args.collection_name}")
    print(f"Total processing time: {total_duration:.2f} seconds ({total_duration/60:.2f} minutes)")
    print("Pipeline execution completed!")

    log_pipeline_event(logger, "pipeline_complete", "Pipeline execution completed successfully",
                      url=args.url, collection=args.collection_name,
                      documents=len(documents), chunks=total_chunks, embeddings=total_embeddings,
                      duration_seconds=total_duration)


if __name__ == "__main__":
    main()