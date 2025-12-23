"""
Chunker utility module for splitting text into smaller pieces for embedding.

This module provides functionality to split text content into appropriately sized
chunks that fit within token limits for embedding models.
"""
import re
from typing import List, Dict
from math import ceil


def chunk_text(text: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[Dict]:
    """
    Split text into chunks of specified size with overlap.
    
    Args:
        text: The text to be chunked
        max_chunk_size: Maximum size of each chunk (default 1000 characters)
        overlap: Number of overlapping characters between chunks (default 100)
    
    Returns:
        List of dictionaries containing chunk information
    """
    if not text:
        return []
    
    # Split text into sentences to avoid breaking in the middle of sentences
    sentences = re.split(r'[.!?]+\s+', text)
    
    chunks = []
    current_chunk = ""
    chunk_start = 0
    chunk_index = 0
    
    for i, sentence in enumerate(sentences):
        # Check if adding this sentence would exceed the chunk size
        if len(current_chunk) + len(sentence) <= max_chunk_size:
            current_chunk += sentence
            if i < len(sentences) - 1:  # Add back the punctuation and space we split on
                current_chunk += ". "
        else:
            # If the current chunk is not empty, save it
            if current_chunk.strip():
                chunks.append({
                    'content': current_chunk.strip(),
                    'start_pos': chunk_start,
                    'end_pos': chunk_start + len(current_chunk),
                    'chunk_index': chunk_index
                })
                
                # Calculate the next starting position with overlap
                if overlap > 0:
                    # Find where to start the next chunk with overlap
                    overlap_start = max(0, len(current_chunk) - overlap)
                    current_chunk = current_chunk[overlap_start:]
                    chunk_start += overlap_start
                else:
                    current_chunk = ""
                
                chunk_index += 1
            
            # If the sentence is larger than max_chunk_size, split it
            if len(sentence) > max_chunk_size:
                # Split the large sentence into smaller parts
                sentence_chunks = [
                    sentence[i:i+max_chunk_size] 
                    for i in range(0, len(sentence), max_chunk_size)
                ]
                
                # Add the first part to current chunk and process the rest
                current_chunk = sentence_chunks[0]
                for j in range(1, len(sentence_chunks)):
                    chunks.append({
                        'content': current_chunk.strip(),
                        'start_pos': chunk_start,
                        'end_pos': chunk_start + len(current_chunk),
                        'chunk_index': chunk_index
                    })
                    
                    chunk_start += len(current_chunk)
                    current_chunk = sentence_chunks[j]
                    chunk_index += 1
            else:
                current_chunk = sentence + (". " if i < len(sentences) - 1 else "")
    
    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append({
            'content': current_chunk.strip(),
            'start_pos': chunk_start,
            'end_pos': chunk_start + len(current_chunk),
            'chunk_index': chunk_index
        })
    
    return chunks


def estimate_token_count(text: str) -> int:
    """
    Estimate the number of tokens in the text.
    This is a rough estimation where 1 token ≈ 4 characters.

    Args:
        text: The text to estimate tokens for

    Returns:
        Estimated number of tokens
    """
    # A common approximation: 1 token ≈ 4 characters for English text
    return ceil(len(text) / 4)


def count_tokens_accurate(text: str) -> int:
    """
    More accurate token counting using a tokenizer (tiktoken for OpenAI models,
    or a simple heuristic for Cohere).

    Args:
        text: The text to count tokens for

    Returns:
        Number of tokens
    """
    # For this implementation, we'll use the character-based estimation
    # In a production system, you might want to use a proper tokenizer
    # like tiktoken for more accurate results
    try:
        # Using the common approximation for English text
        # 1 token ≈ 4 characters
        return ceil(len(text) / 4)
    except Exception:
        # Fallback to character count if anything goes wrong
        return len(text)


def validate_chunk_size(chunk: str, max_tokens: int = 512) -> bool:
    """
    Validate if a chunk is within the token limit.

    Args:
        chunk: Text chunk to validate
        max_tokens: Maximum allowed tokens (default 512 for Cohere)

    Returns:
        True if chunk is within the limit, False otherwise
    """
    token_count = estimate_token_count(chunk)
    return token_count <= max_tokens


def chunk_text_by_tokens(text: str, max_tokens: int = 512) -> List[Dict]:
    """
    Split text into chunks based on token count rather than character count.
    
    Args:
        text: The text to be chunked
        max_tokens: Maximum number of tokens per chunk (default 512)
    
    Returns:
        List of dictionaries containing chunk information
    """
    if not text:
        return []
    
    # First, split into sentences to avoid breaking in the middle
    sentences = re.split(r'(?<=[.!?])\s+', text)
    
    chunks = []
    current_chunk = ""
    chunk_start = 0
    chunk_index = 0
    
    for i, sentence in enumerate(sentences):
        test_chunk = current_chunk + sentence + (" " if i < len(sentences) - 1 else "")
        test_token_count = estimate_token_count(test_chunk)
        
        # If adding this sentence would exceed token limit
        if test_token_count > max_tokens and current_chunk.strip():
            # Save the current chunk
            chunks.append({
                'content': current_chunk.strip(),
                'start_pos': chunk_start,
                'end_pos': chunk_start + len(current_chunk),
                'chunk_index': chunk_index,
                'token_count': estimate_token_count(current_chunk)
            })
            
            # Start a new chunk with the current sentence
            current_chunk = sentence + (" " if i < len(sentences) - 1 else "")
            chunk_start += len(current_chunk)
            chunk_index += 1
        else:
            # Add the sentence to the current chunk
            current_chunk = test_chunk
    
    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append({
            'content': current_chunk.strip(),
            'start_pos': chunk_start,
            'end_pos': chunk_start + len(current_chunk),
            'chunk_index': chunk_index,
            'token_count': estimate_token_count(current_chunk)
        })
    
    return chunks


if __name__ == "__main__":
    # Example usage
    sample_text = "This is a sample text. It contains multiple sentences. " * 50
    chunks = chunk_text(sample_text, max_chunk_size=100)
    
    print(f"Split text into {len(chunks)} chunks")
    for i, chunk in enumerate(chunks):
        print(f"Chunk {i}: {len(chunk['content'])} chars, starts at {chunk['start_pos']}")
        print(f"Content preview: {chunk['content'][:50]}...")
        print("---")