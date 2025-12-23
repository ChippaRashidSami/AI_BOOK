"""
Vector store utility module for storing embeddings in Qdrant Cloud.

This module provides functionality to store, retrieve, and manage vector embeddings
in Qdrant Cloud with proper metadata indexing.
"""
import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
import logging
import uuid
from datetime import datetime


# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_host = os.getenv("QDRANT_HOST")

if not qdrant_api_key or not qdrant_host:
    raise ValueError("QDRANT_API_KEY and QDRANT_HOST environment variables are required")

client = QdrantClient(
    url=qdrant_host,
    api_key=qdrant_api_key,
)


def create_collection(collection_name: str = "rag_embeddings", vector_size: int = 1024) -> bool:
    """
    Create a new collection in Qdrant Cloud for storing embeddings.
    
    Args:
        collection_name: Name of the collection to create
        vector_size: Size of the embedding vectors (default: 1024 for Cohere models)
    
    Returns:
        True if collection was created successfully, False otherwise
    """
    try:
        # Check if collection already exists
        collections = client.get_collections().collections
        collection_names = [c.name for c in collections]
        
        if collection_name in collection_names:
            logging.info(f"Collection {collection_name} already exists")
            return True
        
        # Create the collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,
                distance=models.Distance.COSINE
            )
        )
        
        logging.info(f"Collection {collection_name} created successfully")
        return True
    
    except Exception as e:
        logging.error(f"Error creating collection {collection_name}: {str(e)}")
        return False


def save_chunk_to_qdrant(
    collection_name: str,
    chunk: Dict[str, Any],
    embedding: List[float],
    metadata: Optional[Dict[str, Any]] = None
) -> bool:
    """
    Save a text chunk with its embedding to Qdrant Cloud.

    Args:
        collection_name: Name of the collection to save to
        chunk: Dictionary containing chunk information (content, URL, etc.)
        embedding: Embedding vector for the chunk
        metadata: Additional metadata to store with the chunk

    Returns:
        True if saved successfully, False otherwise
    """
    try:
        # Prepare the payload with chunk data and metadata
        payload = {
            "content": chunk.get('content', ''),
            "url": chunk.get('url', ''),
            "title": chunk.get('title', ''),
            "section": chunk.get('section', ''),
            "document_id": chunk.get('document_id', str(uuid.uuid4())),
            "chunk_index": chunk.get('chunk_index', 0),
            "source_type": chunk.get('source_type', 'docusaurus'),
            "created_at": str(datetime.now()),  # Add timestamp for indexing
            "hierarchy_level": chunk.get('hierarchy_level', 'section'),  # For document structure indexing
            "parent_document": chunk.get('parent_document', ''),  # For document structure indexing
            "position_in_document": chunk.get('position_in_document', 0)  # For document structure indexing
        }

        # Add any additional metadata
        if metadata:
            payload.update(metadata)

        # Generate a unique ID for this record
        record_id = str(uuid.uuid4())

        # Upsert the record to Qdrant
        client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=record_id,
                    vector=embedding,
                    payload=payload
                )
            ]
        )

        logging.info(f"Chunk saved to Qdrant with ID: {record_id}")
        return True

    except Exception as e:
        logging.error(f"Error saving chunk to Qdrant: {str(e)}")
        # Implement retry mechanism for transient failures
        from .error_handling import retry_on_failure
        # Retry the operation up to 3 times with exponential backoff
        @retry_on_failure(max_retries=3, delay=1, backoff=2, exceptions=(Exception,))
        def retry_save_chunk():
            # Re-attempt the save operation
            client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=record_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )
            return True

        try:
            return retry_save_chunk()
        except Exception as retry_error:
            logging.error(f"Failed to save chunk to Qdrant after retries: {str(retry_error)}")
            return False


def check_content_changes_and_update(
    collection_name: str,
    document_id: str,
    new_chunks: List[Dict[str, Any]],
    new_embeddings: List[List[float]]
) -> Dict[str, Any]:
    """
    Check for content changes and perform incremental updates to Qdrant.

    Args:
        collection_name: Name of the collection to update
        document_id: ID of the document to update
        new_chunks: List of new chunk dictionaries
        new_embeddings: List of new embedding vectors corresponding to the chunks

    Returns:
        Dictionary with results of the update operation
    """
    try:
        # Get existing chunks for this document
        existing_points = client.scroll(
            collection_name=collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="document_id",
                        match=models.MatchValue(value=document_id)
                    )
                ]
            ),
            limit=10000  # Assuming a reasonable limit for document chunks
        )

        existing_chunks = {point.id: point for point in existing_points[0]}

        update_results = {
            "updated": 0,
            "added": 0,
            "deleted": 0,
            "unchanged": 0
        }

        for i, (chunk, embedding) in enumerate(zip(new_chunks, new_embeddings)):
            # Create a unique ID for this chunk based on document_id and chunk_index
            chunk_id = f"{document_id}-{chunk.get('chunk_index', i)}"

            # Check if this chunk exists in the collection
            if chunk_id in existing_chunks:
                existing_chunk = existing_chunks[chunk_id]

                # Compare checksums to determine if content has changed
                if chunk.get('checksum') != existing_chunk.payload.get('checksum'):
                    # Content has changed, update the chunk
                    success = save_chunk_to_qdrant(collection_name, chunk, embedding)
                    if success:
                        update_results["updated"] += 1
                    else:
                        logging.error(f"Failed to update chunk {chunk_id}")
                else:
                    # Content hasn't changed
                    update_results["unchanged"] += 1
            else:
                # This is a new chunk, add it
                success = save_chunk_to_qdrant(collection_name, chunk, embedding)
                if success:
                    update_results["added"] += 1
                else:
                    logging.error(f"Failed to add new chunk {chunk_id}")

        # Check for deleted chunks (chunks that exist in Qdrant but not in the new content)
        new_chunk_ids = {f"{document_id}-{chunk.get('chunk_index', i)}" for i, chunk in enumerate(new_chunks)}
        for existing_id in existing_chunks:
            if existing_id not in new_chunk_ids:
                # Delete the chunk that no longer exists
                try:
                    client.delete(
                        collection_name=collection_name,
                        points_selector=[existing_id]
                    )
                    update_results["deleted"] += 1
                    logging.info(f"Deleted outdated chunk {existing_id}")
                except Exception as e:
                    logging.error(f"Error deleting chunk {existing_id}: {str(e)}")

        return update_results

    except Exception as e:
        logging.error(f"Error during incremental update: {str(e)}")
        return {
            "error": str(e),
            "updated": 0,
            "added": 0,
            "deleted": 0,
            "unchanged": 0
        }


def execute_to_qdrant(
    collection_name: str,
    chunks: List[Dict[str, Any]],
    embeddings: List[List[float]],
    metadata_list: Optional[List[Dict[str, Any]]] = None
) -> Dict[str, Any]:
    """
    Execute bulk save of multiple chunks and embeddings to Qdrant.

    Args:
        collection_name: Name of the collection to save to
        chunks: List of chunk dictionaries
        embeddings: List of embedding vectors
        metadata_list: Optional list of metadata dictionaries for each chunk

    Returns:
        Dictionary with results of the operation
    """
    try:
        if len(chunks) != len(embeddings):
            raise ValueError("Chunks and embeddings lists must have the same length")

        points = []

        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            # Prepare the payload with chunk data
            payload = {
                "content": chunk.get('content', ''),
                "url": chunk.get('url', ''),
                "title": chunk.get('title', ''),
                "section": chunk.get('section', ''),
                "document_id": chunk.get('document_id', str(uuid.uuid4())),
                "chunk_index": chunk.get('chunk_index', i),
                "source_type": chunk.get('source_type', 'docusaurus'),
                "created_at": str(datetime.now()),  # Add timestamp for indexing
                "hierarchy_level": chunk.get('hierarchy_level', 'section'),  # For document structure indexing
                "parent_document": chunk.get('parent_document', ''),  # For document structure indexing
                "position_in_document": chunk.get('position_in_document', i)  # For document structure indexing
            }

            # Add metadata if provided
            if metadata_list and i < len(metadata_list):
                payload.update(metadata_list[i])

            # Generate a unique ID for this record
            record_id = str(uuid.uuid4())

            # Create point structure
            point = models.PointStruct(
                id=record_id,
                vector=embedding,
                payload=payload
            )

            points.append(point)

        # Upsert all points to Qdrant
        client.upsert(
            collection_name=collection_name,
            points=points
        )

        result = {
            "status": "success",
            "records_saved": len(points),
            "collection_name": collection_name
        }

        logging.info(f"Bulk save completed: {len(points)} records saved to {collection_name}")
        return result

    except Exception as e:
        logging.error(f"Error during bulk save to Qdrant: {str(e)}")
        # Implement retry mechanism for transient failures
        from .error_handling import retry_on_failure
        # Retry the operation up to 3 times with exponential backoff
        @retry_on_failure(max_retries=3, delay=1, backoff=2, exceptions=(Exception,))
        def retry_bulk_save():
            # Re-attempt the bulk save operation
            client.upsert(
                collection_name=collection_name,
                points=points
            )
            return {
                "status": "success",
                "records_saved": len(points),
                "collection_name": collection_name
            }

        try:
            result = retry_bulk_save()
            logging.info(f"Bulk save completed after retry: {len(points)} records saved to {collection_name}")
            return result
        except Exception as retry_error:
            logging.error(f"Failed to bulk save to Qdrant after retries: {str(retry_error)}")
            return {
                "status": "error",
                "error_message": str(retry_error),
                "records_saved": 0
            }


def search_similar(
    collection_name: str,
    query_embedding: List[float],
    limit: int = 5
) -> List[Dict[str, Any]]:
    """
    Search for similar content based on embedding similarity.
    
    Args:
        collection_name: Name of the collection to search in
        query_embedding: Embedding vector to search for similarity
        limit: Maximum number of results to return (default: 5)
    
    Returns:
        List of similar content with similarity scores
    """
    try:
        results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True
        )
        
        similar_chunks = []
        for result in results:
            chunk_info = {
                "id": result.id,
                "content": result.payload.get("content", ""),
                "url": result.payload.get("url", ""),
                "title": result.payload.get("title", ""),
                "section": result.payload.get("section", ""),
                "document_id": result.payload.get("document_id", ""),
                "similarity_score": result.score
            }
            similar_chunks.append(chunk_info)
        
        return similar_chunks
    
    except Exception as e:
        logging.error(f"Error searching similar content: {str(e)}")
        return []


def get_collection_info(collection_name: str) -> Dict[str, Any]:
    """
    Get information about a specific collection.
    
    Args:
        collection_name: Name of the collection to get info for
    
    Returns:
        Dictionary with collection information
    """
    try:
        info = client.get_collection(collection_name)
        return {
            "name": info.config.params.vectors.size,
            "vector_size": info.config.params.vectors.size,
            "point_count": info.point_count,
            "status": info.status
        }
    except Exception as e:
        logging.error(f"Error getting collection info: {str(e)}")
        return {}


if __name__ == "__main__":
    # Example usage
    collection_name = "test_embeddings"
    
    # Create collection
    create_collection(collection_name)
    
    # Example chunk and embedding
    sample_chunk = {
        "content": "This is a sample text chunk for embedding.",
        "url": "https://example.com/page",
        "title": "Sample Page",
        "section": "Introduction"
    }
    
    sample_embedding = [0.1] * 1024  # Placeholder embedding
    
    # Save to Qdrant
    success = save_chunk_to_qdrant(collection_name, sample_chunk, sample_embedding)
    print(f"Save successful: {success}")
    
    # Bulk save example
    sample_chunks = [sample_chunk, sample_chunk]
    sample_embeddings = [sample_embedding, sample_embedding]
    
    result = execute_to_qdrant(collection_name, sample_chunks, sample_embeddings)
    print(f"Bulk save result: {result}")