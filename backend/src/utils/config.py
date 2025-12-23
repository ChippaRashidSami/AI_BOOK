"""
Configuration utility module for loading and managing environment variables.

This module provides functionality to load environment variables and manage
configuration settings for the embedding pipeline.
"""
import os
from dotenv import load_dotenv
import logging
from typing import Optional


def load_config():
    """
    Load environment variables from .env file.
    """
    load_dotenv()


def get_cohere_api_key() -> Optional[str]:
    """
    Get the Cohere API key from environment variables.
    
    Returns:
        Cohere API key or None if not set
    """
    return os.getenv("COHERE_API_KEY")


def get_qdrant_api_key() -> Optional[str]:
    """
    Get the Qdrant API key from environment variables.
    
    Returns:
        Qdrant API key or None if not set
    """
    return os.getenv("QDRANT_API_KEY")


def get_qdrant_host() -> Optional[str]:
    """
    Get the Qdrant host from environment variables.
    
    Returns:
        Qdrant host URL or None if not set
    """
    return os.getenv("QDRANT_HOST")


def get_config_value(key: str, default: str = None) -> str:
    """
    Get a configuration value from environment variables.
    
    Args:
        key: The environment variable key
        default: Default value if key is not found
    
    Returns:
        The configuration value or default
    """
    return os.getenv(key, default)


def validate_config():
    """
    Validate that all required configuration values are present.
    
    Returns:
        True if all required values are present, False otherwise
    """
    required_vars = ["COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_HOST"]
    missing_vars = []
    
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)
    
    if missing_vars:
        logging.error(f"Missing required environment variables: {', '.join(missing_vars)}")
        return False
    
    return True


def setup_logging():
    """
    Set up basic logging configuration.
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('embedding_pipeline.log'),
            logging.StreamHandler()
        ]
    )


if __name__ == "__main__":
    # Example usage
    load_config()
    print(f"Cohere API Key exists: {'Yes' if get_cohere_api_key() else 'No'}")
    print(f"Qdrant API Key exists: {'Yes' if get_qdrant_api_key() else 'No'}")
    print(f"Qdrant Host: {get_qdrant_host()}")
    print(f"Configuration valid: {validate_config()}")
    
    setup_logging()
    logging.info("Configuration loaded successfully")