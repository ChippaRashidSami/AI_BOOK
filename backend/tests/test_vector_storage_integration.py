"""
Integration test for vector storage in Qdrant Cloud.

This module contains an integration test to verify that the vector storage
works correctly from embedding input to storage in Qdrant.
"""
import unittest
from unittest.mock import patch, Mock
from src.utils.vector_store import create_collection, save_chunk_to_qdrant, execute_to_qdrant


class TestVectorStorageIntegration(unittest.TestCase):
    
    @patch('src.utils.vector_store.client')
    def test_full_storage_workflow(self, mock_client):
        """Test the full workflow: collection creation, chunk storage, and bulk storage."""
        # Mock the client methods
        mock_client.get_collections.return_value = Mock()
        mock_client.get_collections().collections = []
        mock_client.create_collection.return_value = True
        mock_client.upsert.return_value = True
        
        # Test collection creation
        collection_result = create_collection("integration_test_collection")
        self.assertTrue(collection_result)
        
        # Test single chunk storage
        test_chunk = {
            'content': 'Test content for integration',
            'url': 'https://example.com/test',
            'title': 'Integration Test Title',
            'section': 'Integration Test Section',
            'document_id': 'integration-test-doc-123'
        }
        test_embedding = [0.1, 0.2, 0.3, 0.4] * 256  # 1024-dimensional vector
        
        save_result = save_chunk_to_qdrant("integration_test_collection", test_chunk, test_embedding)
        self.assertTrue(save_result)
        
        # Test bulk storage
        chunks = [test_chunk, test_chunk]  # Two identical chunks
        embeddings = [test_embedding, test_embedding]  # Two identical embeddings
        
        bulk_result = execute_to_qdrant("integration_test_collection", chunks, embeddings)
        self.assertEqual(bulk_result["status"], "success")
        self.assertEqual(bulk_result["records_saved"], 2)
    
    @patch('src.utils.vector_store.client')
    def test_storage_with_metadata(self, mock_client):
        """Test storage with additional metadata."""
        # Mock the client methods
        mock_client.upsert.return_value = True
        
        # Test data with additional metadata
        test_chunk = {
            'content': 'Test content with metadata',
            'url': 'https://example.com/metadata',
            'title': 'Metadata Test Title',
            'section': 'Metadata Test Section',
            'document_id': 'metadata-test-doc-456',
            'checksum': 'abc123',
            'hierarchy_level': 'subsection',
            'parent_document': 'main-document'
        }
        test_embedding = [0.5, 0.6, 0.7, 0.8] * 256  # 1024-dimensional vector
        test_metadata = {'custom_field': 'custom_value', 'tags': ['test', 'integration']}
        
        result = save_chunk_to_qdrant(
            "integration_test_collection", 
            test_chunk, 
            test_embedding, 
            metadata=test_metadata
        )
        
        self.assertTrue(result)
        # Verify that upsert was called with the expected payload containing both chunk data and metadata
        mock_client.upsert.assert_called_once()
        # Get the payload from the call
        call_args = mock_client.upsert.call_args
        points = call_args[1]['points']
        payload = points[0].payload
        
        # Check that custom metadata was included
        self.assertEqual(payload['custom_field'], 'custom_value')
        self.assertIn('test', payload['tags'])


if __name__ == '__main__':
    unittest.main()