"""
Crawler utility module for discovering and collecting URLs from Docusaurus-based sites.

This module provides functionality to crawl a GitHub Pages URL and discover all pages
within a Docusaurus-based documentation site.
"""
import requests
from urllib.parse import urljoin, urlparse
from bs4 import BeautifulSoup
import time
import logging
from typing import List, Set


def get_all_urls(base_url: str, max_pages: int = 100) -> List[str]:
    """
    Discover and return all valid URLs from a Docusaurus-based GitHub Pages site.

    Args:
        base_url: The base URL of the Docusaurus site
        max_pages: Maximum number of pages to crawl (default 100)

    Returns:
        List of discovered URLs
    """
    visited_urls: Set[str] = set()
    urls_to_visit = [base_url]
    all_urls: List[str] = []

    # Parse the base domain to ensure we only crawl the same domain
    base_domain = urlparse(base_url).netloc

    # Session to reuse connections for better performance
    session = requests.Session()

    # Headers to make requests look more like a real browser
    session.headers.update({
        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
    })

    while urls_to_visit and len(visited_urls) < max_pages:
        current_url = urls_to_visit.pop(0)

        if current_url in visited_urls:
            continue

        visited_urls.add(current_url)

        try:
            # Use the session for connection reuse
            response = session.get(current_url, timeout=10)
            response.raise_for_status()

            # Check if the response is HTML before parsing
            content_type = response.headers.get('content-type', '')
            if 'text/html' not in content_type:
                logging.info(f"Skipping non-HTML content at {current_url}: {content_type}")
                continue

            soup = BeautifulSoup(response.text, 'html.parser')

            # Add the current URL to our collection if it's valid
            all_urls.append(current_url)

            # Find all links on the page, focusing on navigation elements typical in Docusaurus
            # Look for links in nav, sidebar, and main content areas
            link_containers = soup.find_all(['nav', 'div', 'ul', 'ol'], recursive=True)
            for container in link_containers:
                try:
                    links = container.find_all('a', href=True)
                    for link in links:
                        href = link['href']
                        absolute_url = urljoin(current_url, href)

                        # Only follow links within the same domain and exclude external links
                        parsed_url = urlparse(absolute_url)
                        if parsed_url.netloc == base_domain and parsed_url.scheme in ['http', 'https']:
                            # Normalize the URL by removing fragments
                            normalized_url = absolute_url.split('#')[0]

                            # Only add if it's a valid URL we haven't seen before
                            # and it's not a file download link
                            if (normalized_url not in visited_urls and
                                normalized_url not in urls_to_visit and
                                not any(normalized_url.endswith(ext) for ext in ['.pdf', '.zip', '.doc', '.docx', '.xls', '.xlsx'])):
                                urls_to_visit.append(normalized_url)
                except Exception as e:
                    logging.warning(f"Error processing links in container on page {current_url}: {str(e)}")
                    continue

        except requests.RequestException as e:
            logging.warning(f"Failed to crawl {current_url}: {str(e)}")
            continue
        except Exception as e:
            logging.error(f"Unexpected error while crawling {current_url}: {str(e)}")
            continue

    # Close the session to free up resources
    session.close()

    return all_urls


if __name__ == "__main__":
    # Example usage
    urls = get_all_urls("https://example-docusaurus-site.com")
    print(f"Found {len(urls)} URLs")
    for url in urls:
        print(url)