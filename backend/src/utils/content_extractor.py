"""
Content extractor utility module for extracting text from Docusaurus-based pages.

This module provides functionality to extract clean text content from individual
web pages, removing navigation, headers, and other non-content elements.
"""
import requests
from bs4 import BeautifulSoup
import logging
from typing import Optional
import hashlib


def extract_text_from_url(url: str) -> Optional[dict]:
    """
    Extract clean text content from a single URL, with specific handling for Docusaurus sites.

    Args:
        url: The URL to extract content from

    Returns:
        Dictionary containing the extracted content and metadata, or None if extraction fails
    """
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()

        soup = BeautifulSoup(response.text, 'html.parser')

        # Remove navigation and other non-content elements
        for element in soup.find_all(['nav', 'header', 'footer', 'aside', '.menu', '.nav', '.sidebar']):
            element.decompose()

        # Remove script and style elements
        for element in soup.find_all(['script', 'style', 'meta', 'link', 'noscript']):
            element.decompose()

        # Remove common non-content elements in Docusaurus sites
        for element in soup.select('.theme-edit-this-page, .theme-last-updated, .pagination-nav'):
            element.decompose()

        # Try to find the main content area (common selectors for Docusaurus)
        content_selectors = [
            'main',
            '.main-wrapper',
            '.container',
            '.theme-doc-markdown',
            '.docItemContainer',
            '.markdown',
            'article',
            '.post-content',
            '.doc-content',
            '.container.padding-vert--lg'
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        # If no specific content area found, use the body
        if not content_element:
            content_element = soup.find('body')

        if content_element:
            # Process different content formats separately to preserve structure
            processed_content = []

            # Process paragraphs and text content
            paragraphs = content_element.find_all(['p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li'])
            for p in paragraphs:
                text = p.get_text(separator=' ', strip=True)
                if text:
                    processed_content.append(text)

            # Process code blocks separately to preserve formatting
            code_blocks = content_element.find_all(['code', 'pre'])
            for code in code_blocks:
                code_text = code.get_text(separator=' ', strip=True)
                if code_text:
                    # Mark code blocks to preserve their nature
                    processed_content.append(f"CODE_BLOCK: {code_text}")

            # Process tables separately
            tables = content_element.find_all('table')
            for table in tables:
                table_text = []
                rows = table.find_all(['tr', 'th', 'td'])
                for row in rows:
                    cell_text = row.get_text(separator=' ', strip=True)
                    if cell_text:
                        table_text.append(cell_text)
                if table_text:
                    processed_content.append(f"TABLE: {' | '.join(table_text)}")

            # Join all processed content
            text = '\n'.join(processed_content)

            # Clean up extra whitespace
            lines = [line.strip() for line in text.splitlines() if line.strip()]
            clean_text = '\n'.join(lines)

            # Extract title
            title = soup.find('title')
            title_text = title.get_text().strip() if title else ""

            # Try to get additional metadata from meta tags
            description_tag = soup.find('meta', attrs={'name': 'description'})
            description = description_tag.get('content', '') if description_tag else ''

            # Generate content checksum
            checksum = hashlib.md5(clean_text.encode()).hexdigest()

            return {
                'url': url,
                'title': title_text,
                'content': clean_text,
                'description': description,
                'checksum': checksum
            }
        else:
            logging.warning(f"No content found for URL: {url}")
            return None

    except requests.RequestException as e:
        logging.error(f"Failed to fetch content from {url}: {str(e)}")
        return None
    except Exception as e:
        logging.error(f"Error extracting content from {url}: {str(e)}")
        return None


def clean_content(content: str) -> str:
    """
    Additional cleaning of extracted content with more sophisticated processing.

    Args:
        content: Raw extracted content

    Returns:
        Cleaned content
    """
    if not content:
        return content

    # Remove extra whitespace and normalize line breaks
    lines = [line.strip() for line in content.splitlines() if line.strip()]
    cleaned = '\n'.join(lines)

    # Remove duplicate paragraphs (heuristic)
    paragraphs = cleaned.split('\n\n')
    unique_paragraphs = []
    seen = set()

    for para in paragraphs:
        # Create a simplified version for comparison
        simplified = ' '.join(para.lower().split())
        if simplified not in seen and simplified:
            seen.add(simplified)
            unique_paragraphs.append(para)

    result = '\n\n'.join(unique_paragraphs)

    # Remove common navigation elements that might have slipped through
    patterns_to_remove = [
        r'next\s+page.*',  # "next page" links
        r'previous\s+page.*',  # "previous page" links
        r'jump to.*',  # "jump to" links
        r'back to.*',  # "back to" links
        r'©\s*\d{4}.*',  # Copyright notices
        r'.*table of contents.*',  # Table of contents
        r'.*on this page.*',  # "on this page" navigation
    ]

    import re
    for pattern in patterns_to_remove:
        result = re.sub(pattern, '', result, flags=re.IGNORECASE)

    # Clean up any empty lines created by the removals
    lines = [line.strip() for line in result.splitlines() if line.strip()]
    result = '\n'.join(lines)

    return result


if __name__ == "__main__":
    # Example usage
    result = extract_text_from_url("https://example.com/page")
    if result:
        print(f"Title: {result['title']}")
        print(f"Content length: {len(result['content'])} characters")
        print(f"Checksum: {result['checksum']}")
    else:
        print("Failed to extract content")