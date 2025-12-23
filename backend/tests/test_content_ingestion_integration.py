"""
Integration test for content ingestion from GitHub Pages URL.

This module contains an integration test to verify that the content ingestion
from GitHub Pages URLs works correctly.
"""
import unittest
from unittest.mock import patch, Mock
from src.utils.crawler import get_all_urls
from src.utils.content_extractor import extract_text_from_url


class TestContentIngestionIntegration(unittest.TestCase):
    
    @patch('src.utils.content_extractor.requests.get')
    @patch('src.utils.content_extractor.BeautifulSoup')
    @patch('src.utils.crawler.requests.get')
    @patch('src.utils.crawler.BeautifulSoup')
    def test_content_ingestion_workflow(self, mock_crawler_bs, mock_crawler_get, 
                                      mock_extractor_bs, mock_extractor_get):
        """Test the full workflow: crawling URLs and extracting content."""
        # Mock crawler response
        mock_crawler_response = Mock()
        mock_crawler_response.text = '<html><body><a href="/page1">Page 1</a><a href="/page2">Page 2</a></body></html>'
        mock_crawler_response.raise_for_status.return_value = None
        mock_crawler_get.return_value = mock_crawler_response
        
        # Mock crawler BeautifulSoup
        mock_crawler_soup = Mock()
        mock_link1 = Mock()
        mock_link1.__getitem__.return_value = '/page1'
        mock_link2 = Mock()
        mock_link2.__getitem__.return_value = '/page2'
        mock_crawler_soup.find_all.return_value = [mock_link1, mock_link2]
        mock_crawler_bs.return_value = mock_crawler_soup
        
        # Crawl URLs
        urls = get_all_urls('https://example.com/docs', max_pages=5)
        
        # Verify we got URLs
        self.assertGreater(len(urls), 0)
        
        # Mock content extraction responses
        mock_extractor_response = Mock()
        mock_extractor_response.text = '<html><head><title>Test Page</title></head><body><p>This is test content.</p></body></html>'
        mock_extractor_response.raise_for_status.return_value = None
        mock_extractor_get.return_value = mock_extractor_response
        
        # Mock extractor BeautifulSoup
        mock_extractor_soup = Mock()
        mock_body = Mock()
        mock_body.get_text.return_value = 'This is test content.'
        mock_extractor_soup.find.return_value = mock_body
        mock_extractor_bs.return_value = mock_extractor_soup
        
        # Extract content from the first URL
        if urls:
            content_result = extract_text_from_url(urls[0])
            
            # Verify content extraction
            self.assertIsNotNone(content_result)
            self.assertIn('test content', content_result['content'])
            self.assertEqual(content_result['title'], 'Test Page')


if __name__ == '__main__':
    unittest.main()