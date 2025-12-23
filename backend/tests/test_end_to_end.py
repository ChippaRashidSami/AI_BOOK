"""
End-to-End Test for the Embedding Pipeline

This module contains an end-to-end test for the embedding pipeline
using the target site: https://hackathon-l4pp.vercel.app/
"""
import unittest
import os
from src.utils.crawler import get_all_urls
from src.utils.content_extractor import extract_text_from_url
from src.utils.config import validate_config


class TestEndToEnd(unittest.TestCase):
    
    def setUp(self):
        """Set up the test with the target URL."""
        self.target_url = "https://hackathon-l4pp.vercel.app/"
        
        # Validate configuration
        if not validate_config():
            self.skipTest("Configuration validation failed. Check your environment variables.")
    
    def test_crawl_target_site(self):
        """Test crawling the target site."""
        # Test that we can crawl the site and get URLs
        urls = get_all_urls(self.target_url, max_pages=5)  # Limit to 5 pages for testing
        
        # We expect at least the main page
        self.assertGreaterEqual(len(urls), 1, f"Expected at least 1 URL, got {len(urls)}")
        
        # Print discovered URLs for verification
        print(f"Discovered {len(urls)} URLs:")
        for url in urls:
            print(f"  - {url}")
    
    def test_extract_content_from_target_site(self):
        """Test extracting content from the target site."""
        # Test that we can extract content from the main page
        content_data = extract_text_from_url(self.target_url)
        
        # We expect to get content data
        self.assertIsNotNone(content_data, "Expected content data, got None")
        
        if content_data:
            # Verify that we have content
            self.assertGreater(len(content_data['content']), 0, "Expected content, got empty string")
            self.assertIn('title', content_data, "Expected title in content data")

            print(f"Extracted content from {self.target_url}")
            print(f"Title: {content_data['title']}")
            print(f"Content length: {len(content_data['content'])} characters")


if __name__ == '__main__':
    unittest.main()