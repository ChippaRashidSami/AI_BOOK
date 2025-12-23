"""
Integration test for embedding generation.

This module contains an integration test to verify that the embedding generation
works correctly from text input to vector output.
"""
import unittest
from unittest.mock import patch, Mock
from src.utils.chunker import chunk_text_by_tokens
from src.utils.embedder import embed, get_embedding_dimensions


class TestEmbeddingGenerationIntegration(unittest.TestCase):
    
    @patch('src.utils.embedder.co')
    def test_embedding_generation_workflow(self, mock_cohere):
        """Test the full workflow: text chunking and embedding generation."""
        # Mock Cohere response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3, 0.4]]  # Simulated embedding vector
        mock_cohere.embed.return_value = mock_response
        
        # Test text
        sample_text = "This is a sample text for embedding."
        
        # Chunk the text
        chunks = chunk_text_by_tokens(sample_text, max_tokens=50)
        
        # Verify we got chunks
        self.assertGreater(len(chunks), 0)
        
        # Extract content from chunks
        chunk_contents = [chunk['content'] for chunk in chunks]
        
        # Generate embeddings
        embeddings = embed(chunk_contents)
        
        # Verify embeddings were generated
        self.assertEqual(len(embeddings), len(chunk_contents))
        
        # Verify embedding dimensions (mocked to 4 dimensions)
        expected_dims = 4  # Based on our mock
        for embedding in embeddings:
            self.assertEqual(len(embedding), expected_dims)
    
    def test_embedding_dimensions(self):
        """Test that we get the expected embedding dimensions."""
        # This test would require actual Cohere API or better mocking
        # For now, we'll test the function that returns expected dimensions
        dims = get_embedding_dimensions()
        # Cohere v3 models return 1024 dimensions
        self.assertEqual(dims, 1024)


if __name__ == '__main__':
    unittest.main()