"""
Unit tests for crawler functionality.

This module contains unit tests for the crawler utility module.
"""
import unittest
from unittest.mock import patch, Mock
import requests
from src.utils.crawler import get_all_urls


class TestCrawler(unittest.TestCase):
    
    @patch('src.utils.crawler.requests.get')
    @patch('src.utils.crawler.BeautifulSoup')
    def test_get_all_urls_success(self, mock_bs, mock_get):
        """Test successful URL crawling."""
        # Mock response
        mock_response = Mock()
        mock_response.text = '<html><body><a href="/page1">Page 1</a><a href="/page2">Page 2</a></body></html>'
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response
        
        # Mock BeautifulSoup
        mock_soup = Mock()
        mock_link1 = Mock()
        mock_link1.__getitem__.return_value = '/page1'
        mock_link2 = Mock()
        mock_link2.__getitem__.return_value = '/page2'
        mock_soup.find_all.return_value = [mock_link1, mock_link2]
        mock_bs.return_value = mock_soup
        
        # Test
        urls = get_all_urls('https://example.com', max_pages=5)
        
        # Assertions
        self.assertEqual(len(urls), 1)  # Only the base URL is added initially
        mock_get.assert_called_once()
    
    @patch('src.utils.crawler.requests.get')
    def test_get_all_urls_request_exception(self, mock_get):
        """Test handling of request exceptions."""
        # Mock request to raise an exception
        mock_get.side_effect = requests.RequestException("Connection error")
        
        # Test
        urls = get_all_urls('https://example.com', max_pages=5)
        
        # Assertions
        self.assertEqual(urls, [])
    
    @patch('src.utils.crawler.requests.get')
    def test_get_all_urls_non_html_content(self, mock_get):
        """Test handling of non-HTML content."""
        # Mock response with non-HTML content type
        mock_response = Mock()
        mock_response.text = 'binary data'
        mock_response.headers = {'content-type': 'application/pdf'}
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response
        
        # Test
        urls = get_all_urls('https://example.com', max_pages=5)
        
        # Should return with just the base URL since non-HTML content is skipped
        self.assertEqual(len(urls), 0)  # The non-HTML page would be skipped


if __name__ == '__main__':
    unittest.main()