"""
Unit tests for vector storage functionality.

This module contains unit tests for the vector store utility module.
"""
import unittest
from unittest.mock import patch, Mock
from src.utils.vector_store import create_collection, save_chunk_to_qdrant


class TestVectorStore(unittest.TestCase):
    
    @patch('src.utils.vector_store.client')
    def test_create_collection(self, mock_client):
        """Test creating a collection in Qdrant."""
        # Mock the client methods
        mock_client.get_collections.return_value = Mock()
        mock_client.get_collections().collections = []
        mock_client.create_collection.return_value = True
        
        # Test
        result = create_collection("test_collection")
        
        # Assertions
        self.assertTrue(result)
        mock_client.create_collection.assert_called_once()
    
    @patch('src.utils.vector_store.client')
    def test_save_chunk_to_qdrant(self, mock_client):
        """Test saving a chunk to Qdrant."""
        # Mock the client upsert method
        mock_client.upsert.return_value = True
        
        # Test data
        test_chunk = {
            'content': 'Test content',
            'url': 'https://example.com',
            'title': 'Test Title',
            'section': 'Test Section',
            'document_id': 'test-doc-123'
        }
        test_embedding = [0.1, 0.2, 0.3, 0.4]
        
        # Test
        result = save_chunk_to_qdrant("test_collection", test_chunk, test_embedding)
        
        # Assertions
        self.assertTrue(result)
        mock_client.upsert.assert_called_once()
    
    @patch('src.utils.vector_store.client')
    def test_save_chunk_to_qdrant_with_error_handling(self, mock_client):
        """Test error handling in save_chunk_to_qdrant."""
        # Mock the client to raise an exception on first call, then succeed
        mock_client.upsert.side_effect = [Exception("Connection error"), True]
        
        # Test data
        test_chunk = {
            'content': 'Test content',
            'url': 'https://example.com',
            'title': 'Test Title',
            'section': 'Test Section',
            'document_id': 'test-doc-123'
        }
        test_embedding = [0.1, 0.2, 0.3, 0.4]
        
        # Test - this would test the retry mechanism, but since we're mocking
        # we can only test that the function handles the exception
        try:
            result = save_chunk_to_qdrant("test_collection", test_chunk, test_embedding)
            # If we reach this, the function handled the exception
            self.assertIsNotNone(result)  # Either True or False depending on final outcome
        except Exception:
            # If an exception is still raised, the error handling might not be working as expected
            self.fail("save_chunk_to_qdrant did not handle the exception properly")


if __name__ == '__main__':
    unittest.main()