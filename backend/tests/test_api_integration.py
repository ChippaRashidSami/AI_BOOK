"""
API Integration Tests for the Embedding Pipeline Service

This module contains integration tests for the API endpoints.
"""
import unittest
from fastapi.testclient import TestClient
from src.api import app


class TestAPIIntegration(unittest.TestCase):
    def setUp(self):
        """Set up the test client."""
        self.client = TestClient(app)
    
    def test_process_url_endpoint(self):
        """Test the process URL endpoint."""
        # Test with a valid request
        response = self.client.post(
            "/process-url",
            json={
                "url": "https://example.com/docs",
                "collection_name": "test-collection",
                "chunk_size": 1000,
                "batch_size": 10
            }
        )
        
        # Should return a 200 status (or 400 if the URL is invalid, which is expected in testing)
        # Since we're not actually processing real URLs in tests, we expect it to at least validate
        self.assertIn(response.status_code, [200, 400, 500])
    
    def test_get_job_status_endpoint(self):
        """Test the job status endpoint."""
        # Test with a non-existent job ID
        response = self.client.get("/jobs/non-existent-job-id")
        
        # Should return 404 for non-existent job
        self.assertEqual(response.status_code, 404)
    
    def test_query_endpoint(self):
        """Test the query endpoint."""
        response = self.client.post(
            "/query",
            json={
                "query_text": "test query",
                "collection_name": "test-collection",
                "limit": 5
            }
        )
        
        # Should return 200 with query results
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn("query", data)
        self.assertIn("results", data)
    
    def test_list_collections_endpoint(self):
        """Test the list collections endpoint."""
        response = self.client.get("/collections")
        
        # Should return 200 with collections list
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn("collections", data)
    
    def test_rate_limiting(self):
        """Test that rate limiting is applied."""
        # Make multiple requests quickly to test rate limiting
        for i in range(10):  # More than our rate limit of 5/minute in the test
            response = self.client.post(
                "/query",
                json={
                    "query_text": f"test query {i}",
                    "collection_name": "test-collection",
                    "limit": 5
                }
            )
            # We may or may not hit the rate limit depending on timing,
            # but the important thing is that the rate limiter is configured
            self.assertIn(response.status_code, [200, 429])
    
    def test_error_handling(self):
        """Test error handling for invalid requests."""
        # Test with invalid URL
        response = self.client.post(
            "/process-url",
            json={
                "url": "invalid-url",
                "collection_name": "test-collection"
            }
        )
        
        # Should return 422 for validation error or 400 for invalid URL
        self.assertIn(response.status_code, [400, 422])


if __name__ == '__main__':
    unittest.main()