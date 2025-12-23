"""
Unit tests for embedding functionality.

This module contains unit tests for the embedder utility module.
"""
import unittest
from unittest.mock import patch, Mock
from src.utils.embedder import is_content_valid, has_excessive_repetition


class TestEmbedder(unittest.TestCase):
    
    def test_is_content_valid_with_good_content(self):
        """Test content validation with good content."""
        good_content = "This is a valid piece of content with meaningful text."
        self.assertTrue(is_content_valid(good_content))
    
    def test_is_content_valid_with_short_content(self):
        """Test content validation with short content."""
        short_content = "Hi"
        self.assertFalse(is_content_valid(short_content))
    
    def test_is_content_valid_with_excessive_special_chars(self):
        """Test content validation with excessive special characters."""
        bad_content = "!@#$%^&*()" * 10  # Lots of special characters
        self.assertFalse(is_content_valid(bad_content))
    
    def test_is_content_valid_with_excessive_repetition(self):
        """Test content validation with excessive character repetition."""
        repetitive_content = "aaaaabbbbbccccc" * 10  # Repetitive content
        self.assertFalse(is_content_valid(repetitive_content))
    
    def test_has_excessive_repetition_true(self):
        """Test excessive repetition detection (should return True)."""
        repetitive_text = "aaaaabbbbbc"
        self.assertTrue(has_excessive_repetition(repetitive_text, max_repetition=5))
    
    def test_has_excessive_repetition_false(self):
        """Test excessive repetition detection (should return False)."""
        normal_text = "This is normal text with no repetition."
        self.assertFalse(has_excessive_repetition(normal_text, max_repetition=5))
    
    @patch('src.utils.embedder.co')
    def test_embed_with_quality_filtering(self, mock_cohere):
        """Test that embedding filters out low-quality content."""
        # This test would require more complex mocking of the Cohere client
        # For now, we'll just test the validation functions
        good_texts = ["Good content here.", "Another good piece of content."]
        bad_texts = ["", "hi", "aaaaabbbbccccc"]  # Short, repetitive content
        mixed_texts = good_texts + bad_texts
        
        # The embed function should filter out bad content
        # This test would be more comprehensive with full mocking of Cohere API
        valid_count = 0
        for text in mixed_texts:
            if is_content_valid(text):
                valid_count += 1
        
        self.assertEqual(valid_count, 2)  # Only 2 good texts


if __name__ == '__main__':
    unittest.main()