"""
Unit tests for content extraction functionality.

This module contains unit tests for the content extractor utility module.
"""
import unittest
from unittest.mock import patch, Mock
import requests
from src.utils.content_extractor import extract_text_from_url, clean_content


class TestContentExtractor(unittest.TestCase):
    
    @patch('src.utils.content_extractor.requests.get')
    @patch('src.utils.content_extractor.BeautifulSoup')
    def test_extract_text_from_url_success(self, mock_bs, mock_get):
        """Test successful content extraction from a URL."""
        # Mock response
        mock_response = Mock()
        mock_response.text = '<html><head><title>Test Page</title></head><body><p>This is test content.</p></body></html>'
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response
        
        # Mock BeautifulSoup
        mock_element = Mock()
        mock_element.get_text.return_value = 'This is test content.'
        mock_bs.return_value.find.return_value = mock_element
        
        # Test
        result = extract_text_from_url('https://example.com/page')
        
        # Assertions
        self.assertIsNotNone(result)
        self.assertEqual(result['title'], 'Test Page')
        self.assertIn('test content', result['content'])
        mock_get.assert_called_once()
    
    @patch('src.utils.content_extractor.requests.get')
    def test_extract_text_from_url_request_exception(self, mock_get):
        """Test handling of request exceptions."""
        # Mock request to raise an exception
        mock_get.side_effect = requests.RequestException("Connection error")
        
        # Test
        result = extract_text_from_url('https://example.com/page')
        
        # Assertions
        self.assertIsNone(result)
    
    def test_clean_content_removes_extra_whitespace(self):
        """Test that clean_content removes extra whitespace."""
        dirty_content = "  This   is  \n\n\n  a  test  \n\n  content  "
        cleaned = clean_content(dirty_content)
        
        # Should have normalized whitespace
        self.assertIn('This is a test content', cleaned)
        self.assertNotIn('\n\n\n', cleaned)
    
    def test_clean_content_removes_duplicates(self):
        """Test that clean_content removes duplicate paragraphs."""
        content_with_duplicates = "First paragraph.\n\nSecond paragraph.\n\nFirst paragraph."
        cleaned = clean_content(content_with_duplicates)
        
        # Count occurrences of "First paragraph"
        occurrences = cleaned.lower().count('first paragraph')
        self.assertEqual(occurrences, 1, "Duplicate paragraphs should be removed")
    
    @patch('src.utils.content_extractor.requests.get')
    @patch('src.utils.content_extractor.BeautifulSoup')
    def test_extract_text_with_code_blocks(self, mock_bs, mock_get):
        """Test content extraction with code blocks."""
        html_content = '''
        <html>
            <head><title>Test Page</title></head>
            <body>
                <p>This is regular text.</p>
                <pre><code>def hello_world():
    print("Hello, World!")
</code></pre>
                <p>More regular text.</p>
            </body>
        </html>
        '''
        
        # Mock response
        mock_response = Mock()
        mock_response.text = html_content
        mock_response.raise_for_status.return_value = None
        mock_get.return_value = mock_response
        
        # Mock BeautifulSoup
        mock_body = Mock()
        mock_body.find_all.side_effect = lambda tags: [
            Mock(get_text=lambda separator, strip: 'This is regular text.'),
            Mock(get_text=lambda separator, strip: 'def hello_world():\n    print("Hello, World!")'),
            Mock(get_text=lambda separator, strip: 'More regular text.')
        ] if 'p' in tags else [
            Mock(get_text=lambda separator, strip: 'def hello_world():\n    print("Hello, World!")')
        ] if 'code' in tags else []
        mock_body.find.return_value = mock_body
        mock_bs.return_value.find.return_value = mock_body
        
        # Test
        result = extract_text_from_url('https://example.com/page')
        
        # Assertions
        self.assertIsNotNone(result)
        # The content should include the code block
        self.assertIn('def hello_world', result['content'])


if __name__ == '__main__':
    unittest.main()