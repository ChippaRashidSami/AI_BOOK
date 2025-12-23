"""
Logging utility module for the embedding pipeline.

This module provides centralized logging configuration and utilities
for the entire embedding pipeline application.
"""
import logging
import os
from datetime import datetime
from typing import Optional


def setup_logging(log_level: str = "INFO", log_file: Optional[str] = None):
    """
    Set up centralized logging configuration for the pipeline.
    
    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional file path to write logs to
    """
    # Create logs directory if it doesn't exist
    if log_file:
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
    
    # Configure logging format
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
    
    # Set up the root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, log_level.upper()))
    
    # Clear any existing handlers
    root_logger.handlers.clear()
    
    # Create formatter
    formatter = logging.Formatter(log_format)
    
    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(getattr(logging, log_level.upper()))
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)
    
    # Create file handler if specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(getattr(logging, log_level.upper()))
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)


def get_logger(name: str) -> logging.Logger:
    """
    Get a named logger instance.
    
    Args:
        name: Name of the logger
    
    Returns:
        Configured logger instance
    """
    return logging.getLogger(name)


def log_pipeline_event(
    logger: logging.Logger,
    event_type: str,
    message: str,
    url: str = None,
    collection: str = None,
    **kwargs
):
    """
    Log a standardized pipeline event.
    
    Args:
        logger: Logger instance to use
        event_type: Type of event (e.g., 'crawl', 'extract', 'embed', 'store')
        message: Event message
        url: Associated URL (if applicable)
        collection: Associated collection (if applicable)
        **kwargs: Additional context information
    """
    context = {
        'event_type': event_type,
        'message': message
    }
    
    if url:
        context['url'] = url
    if collection:
        context['collection'] = collection
    
    context.update(kwargs)
    
    log_message = f"[{event_type}] {message}"
    for key, value in context.items():
        if key not in ['event_type', 'message']:
            log_message += f" | {key}: {value}"
    
    logger.info(log_message)


def log_error_event(
    logger: logging.Logger,
    error_type: str,
    error_message: str,
    url: str = None,
    collection: str = None,
    **kwargs
):
    """
    Log a standardized error event.
    
    Args:
        logger: Logger instance to use
        error_type: Type of error
        error_message: Error message
        url: Associated URL (if applicable)
        collection: Associated collection (if applicable)
        **kwargs: Additional context information
    """
    context = {
        'error_type': error_type,
        'error_message': error_message
    }
    
    if url:
        context['url'] = url
    if collection:
        context['collection'] = collection
    
    context.update(kwargs)
    
    log_message = f"[ERROR - {error_type}] {error_message}"
    for key, value in context.items():
        if key not in ['error_type', 'error_message']:
            log_message += f" | {key}: {value}"
    
    logger.error(log_message)


if __name__ == "__main__":
    # Example usage
    setup_logging(log_level="INFO", log_file="logs/embedding_pipeline.log")
    
    logger = get_logger("embedding_pipeline")
    
    logger.info("Logging setup completed")
    log_pipeline_event(
        logger, 
        "test_event", 
        "This is a test pipeline event", 
        url="https://example.com",
        collection="test_collection"
    )
    
    try:
        # Simulate an error
        raise ValueError("This is a test error")
    except Exception as e:
        log_error_event(
            logger,
            "test_error",
            str(e),
            url="https://example.com",
            error_code=500
        )