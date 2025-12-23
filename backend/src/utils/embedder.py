"""
Embedder utility module for generating semantic embeddings using Cohere models.

This module provides functionality to generate vector embeddings for text chunks
using Cohere's embedding API.
"""
import cohere
import os
from dotenv import load_dotenv
from typing import List, Dict, Any
import time
import logging
from .error_handling import retry_on_failure, rate_limit_handler


# Load environment variables
load_dotenv()

# Initialize Cohere client
cohere_api_key = os.getenv("COHERE_API_KEY")
if not cohere_api_key:
    raise ValueError("COHERE_API_KEY environment variable is required")

co = cohere.Client(cohere_api_key)


@retry_on_failure(max_retries=3, delay=1, backoff=2, exceptions=(Exception,))
@rate_limit_handler(max_calls=100, time_window=60)  # Conservative rate limit (100 calls per minute)
def embed(texts: List[str], model: str = "embed-english-v3.0", input_type: str = "search_document") -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere API.

    Args:
        texts: List of texts to embed
        model: Cohere model to use for embeddings (default: embed-english-v3.0)
        input_type: Type of input text (default: search_document)

    Returns:
        List of embedding vectors
    """
    # Validate content quality before embedding
    validated_texts = []
    for text in texts:
        if is_content_valid(text):
            validated_texts.append(text)
        else:
            logging.warning(f"Skipping low-quality content for embedding: {text[:50]}...")

    if not validated_texts:
        logging.warning("No valid content to embed after quality validation")
        return []

    try:
        # Cohere API has a limit on the number of texts per request
        # We'll process in batches if needed
        batch_size = 96  # Conservative batch size to stay within API limits
        all_embeddings = []

        for i in range(0, len(validated_texts), batch_size):
            batch = validated_texts[i:i + batch_size]

            response = co.embed(
                texts=batch,
                model=model,
                input_type=input_type
            )

            all_embeddings.extend(response.embeddings)

            # Be respectful to the API - add a small delay between batches if needed
            if i + batch_size < len(validated_texts):
                time.sleep(0.1)

        return all_embeddings

    except Exception as e:
        logging.error(f"Error generating embeddings: {str(e)}")
        raise


def is_content_valid(text: str, min_length: int = 10) -> bool:
    """
    Validate content quality before generating embeddings.

    Args:
        text: Text to validate
        min_length: Minimum length of text to be considered valid

    Returns:
        True if content is valid, False otherwise
    """
    if not text or len(text.strip()) < min_length:
        return False

    # Check for too many special characters which might indicate low quality
    special_chars = sum(1 for c in text if not c.isalnum() and not c.isspace())
    if special_chars > len(text) * 0.5:  # More than 50% special characters
        return False

    # Check for too many repeated characters which might indicate noise
    if has_excessive_repetition(text):
        return False

    return True


def has_excessive_repetition(text: str, max_repetition: int = 5) -> bool:
    """
    Check if text has excessive character repetition.

    Args:
        text: Text to check
        max_repetition: Maximum allowed consecutive repeated characters

    Returns:
        True if text has excessive repetition, False otherwise
    """
    if len(text) < max_repetition:
        return False

    for i in range(len(text) - max_repetition):
        if len(set(text[i:i+max_repetition])) == 1:  # All characters are the same
            return True

    return False


def embed_documents(documents: List[Dict], model: str = "embed-english-v3.0") -> List[Dict]:
    """
    Generate embeddings for a list of document dictionaries, preserving document metadata.

    Args:
        documents: List of document dictionaries with 'content', 'id', etc.
        model: Cohere model to use for embeddings

    Returns:
        List of dictionaries containing the original document info plus embeddings
    """
    if not documents:
        return []

    # Extract just the text content for embedding
    texts = [doc.get('content', '') for doc in documents]

    try:
        embeddings = embed(texts, model=model, input_type="search_document")

        # Combine the embeddings with the original document information
        result = []
        for doc, embedding in zip(documents, embeddings):
            doc_copy = doc.copy()
            doc_copy['embedding'] = embedding
            result.append(doc_copy)

        return result

    except Exception as e:
        logging.error(f"Error generating embeddings for documents: {str(e)}")
        raise


def embed_single(text: str, model: str = "embed-english-v3.0", input_type: str = "search_document") -> List[float]:
    """
    Generate embedding for a single text using Cohere API.
    
    Args:
        text: Text to embed
        model: Cohere model to use for embeddings (default: embed-english-v3.0)
        input_type: Type of input text (default: search_document)
    
    Returns:
        Embedding vector
    """
    try:
        response = co.embed(
            texts=[text],
            model=model,
            input_type=input_type
        )
        
        return response.embeddings[0]
    
    except Exception as e:
        logging.error(f"Error generating embedding for text: {str(e)}")
        raise


def get_embedding_dimensions(model: str = "embed-english-v3.0") -> int:
    """
    Get the expected dimensions for embeddings from a specific model.
    
    Args:
        model: Cohere model name
    
    Returns:
        Number of dimensions in the embedding
    """
    # As per Cohere documentation, embed-english-v3.0 and embed-multilingual-v3.0 return 1024 dimensions
    if "v3.0" in model:
        return 1024
    # Add other models as needed
    return 1024  # Default to 1024 for most Cohere models


if __name__ == "__main__":
    # Example usage
    sample_texts = [
        "This is the first text to embed.",
        "This is the second text to embed.",
        "And here's a third example."
    ]
    
    try:
        embeddings = embed(sample_texts)
        print(f"Generated {len(embeddings)} embeddings")
        print(f"Embedding dimensions: {len(embeddings[0])}")
        print(f"First embedding preview: {embeddings[0][:10]}...")  # First 10 dimensions
    except Exception as e:
        print(f"Error: {e}")