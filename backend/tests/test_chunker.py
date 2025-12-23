"""
Unit tests for chunking functionality.

This module contains unit tests for the chunker utility module.
"""
import unittest
from src.utils.chunker import chunk_text, chunk_text_by_tokens, estimate_token_count, validate_chunk_size


class TestChunker(unittest.TestCase):
    
    def test_chunk_text_by_size(self):
        """Test chunking text by character size."""
        text = "This is a sample text. " * 10  # Creates a text of about 220 characters
        chunks = chunk_text(text, max_chunk_size=50)
        
        # Should create multiple chunks since the text is larger than 50 characters
        self.assertGreater(len(chunks), 1)
        
        # Each chunk should be at most max_chunk_size characters
        for chunk in chunks:
            self.assertLessEqual(len(chunk['content']), 50)
    
    def test_chunk_text_by_tokens(self):
        """Test chunking text by token count."""
        text = "This is a sample text. " * 20  # Create a longer text
        chunks = chunk_text_by_tokens(text, max_tokens=10)
        
        # Should create multiple chunks since we're limiting to 10 tokens
        self.assertGreater(len(chunks), 1)
        
        # Each chunk should have at most max_tokens tokens (approximately)
        for chunk in chunks:
            token_count = estimate_token_count(chunk['content'])
            # Allow some flexibility as our token estimation is rough
            self.assertLessEqual(token_count, 15)  # Slightly more than 10 to account for estimation
    
    def test_estimate_token_count(self):
        """Test token estimation function."""
        text = "This is a test sentence."
        estimated_tokens = estimate_token_count(text)
        
        # Our estimation is len(text)/4, so for "This is a test sentence." (25 chars) it should be ~7
        expected = len(text) / 4
        self.assertAlmostEqual(estimated_tokens, expected, delta=1)
    
    def test_validate_chunk_size(self):
        """Test chunk size validation."""
        # Valid chunk (under 10 tokens)
        valid_chunk = "Short text"
        self.assertTrue(validate_chunk_size(valid_chunk, max_tokens=10))
        
        # Invalid chunk (over 2 tokens, assuming ~1 token per 4 chars)
        invalid_chunk = "This is definitely a much longer piece of text that should exceed the token limit"
        # This should be false since the text is much longer
        # Note: Our token estimation is rough, so we test with a very small limit
        self.assertFalse(validate_chunk_size(invalid_chunk, max_tokens=2))
    
    def test_empty_text(self):
        """Test chunking with empty text."""
        chunks = chunk_text("")
        self.assertEqual(len(chunks), 0)
        
        chunks_by_tokens = chunk_text_by_tokens("")
        self.assertEqual(len(chunks_by_tokens), 0)


if __name__ == '__main__':
    unittest.main()