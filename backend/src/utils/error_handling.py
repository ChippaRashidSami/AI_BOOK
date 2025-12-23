"""
Error handling and retry mechanisms for the embedding pipeline.

This module provides utilities for handling transient failures with
appropriate retry strategies and backoff mechanisms.
"""
import time
import random
from functools import wraps
from typing import Callable, Type, Any
import logging


def retry_on_failure(
    max_retries: int = 3,
    delay: float = 1.0,
    backoff: float = 2.0,
    exceptions: tuple = (Exception,)
):
    """
    Decorator to retry a function on failure with exponential backoff.
    
    Args:
        max_retries: Maximum number of retry attempts
        delay: Initial delay between retries in seconds
        backoff: Multiplier for delay after each retry
        exceptions: Tuple of exceptions to catch and retry on
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            current_delay = delay
            last_exception = None
            
            for attempt in range(max_retries + 1):  # +1 to include the initial attempt
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    if attempt == max_retries:
                        # Final attempt, re-raise the exception
                        logging.error(f"All {max_retries} retry attempts failed for {func.__name__}: {str(e)}")
                        raise e
                    
                    logging.warning(
                        f"Attempt {attempt + 1} failed for {func.__name__}: {str(e)}. "
                        f"Retrying in {current_delay} seconds..."
                    )
                    
                    # Add jitter to prevent thundering herd
                    time.sleep(current_delay + random.uniform(0, 0.1))
                    current_delay *= backoff
            
            # This line should never be reached, but included for type checking
            raise last_exception
        
        return wrapper
    return decorator


def rate_limit_handler(max_calls: int, time_window: int = 60):
    """
    Decorator to enforce rate limiting.
    
    Args:
        max_calls: Maximum number of calls allowed
        time_window: Time window in seconds
    """
    def decorator(func: Callable) -> Callable:
        calls = []
        
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            nonlocal calls
            now = time.time()
            
            # Remove calls that are outside the time window
            calls = [call_time for call_time in calls if now - call_time < time_window]
            
            if len(calls) >= max_calls:
                sleep_time = time_window - (now - calls[0])
                if sleep_time > 0:
                    logging.info(f"Rate limit reached, sleeping for {sleep_time:.2f} seconds")
                    time.sleep(sleep_time)
                    # After sleeping, we should have space for a new call
                    calls = [call_time for call_time in calls if now - call_time < time_window]
            
            calls.append(now)
            return func(*args, **kwargs)
        
        return wrapper
    return decorator


def handle_api_errors(func: Callable) -> Callable:
    """
    Decorator to handle common API errors with specific responses.
    """
    @wraps(func)
    def wrapper(*args, **kwargs) -> Any:
        try:
            return func(*args, **kwargs)
        except Exception as e:
            error_msg = str(e).lower()
            
            # Handle common API errors
            if "rate limit" in error_msg or "429" in error_msg:
                logging.warning(f"Rate limit error for {func.__name__}, consider implementing backoff")
                raise e
            elif "timeout" in error_msg or "408" in error_msg:
                logging.warning(f"Timeout error for {func.__name__}, consider increasing timeout")
                raise e
            elif "not found" in error_msg or "404" in error_msg:
                logging.error(f"Resource not found in {func.__name__}: {str(e)}")
                raise e
            elif "unauthorized" in error_msg or "401" in error_msg or "403" in error_msg:
                logging.error(f"Authentication error in {func.__name__}: {str(e)}")
                raise e
            else:
                logging.error(f"API error in {func.__name__}: {str(e)}")
                raise e
    
    return wrapper


def circuit_breaker(failure_threshold: int = 5, recovery_timeout: int = 60):
    """
    Decorator implementing circuit breaker pattern.
    
    Args:
        failure_threshold: Number of failures before opening the circuit
        recovery_timeout: Time in seconds to wait before attempting to close the circuit
    """
    def decorator(func: Callable) -> Callable:
        class CircuitState:
            def __init__(self):
                self.failure_count = 0
                self.last_failure_time = None
                self.is_open = False
        
        circuit_state = CircuitState()
        
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            if circuit_state.is_open:
                if time.time() - circuit_state.last_failure_time >= recovery_timeout:
                    # Attempt to close the circuit and make one test call
                    logging.info(f"Circuit breaker attempting reset for {func.__name__}")
                    try:
                        result = func(*args, **kwargs)
                        circuit_state.failure_count = 0
                        circuit_state.is_open = False
                        return result
                    except Exception:
                        circuit_state.last_failure_time = time.time()
                        circuit_state.failure_count += 1
                        raise
                else:
                    logging.error(f"Circuit breaker open for {func.__name__}, failing fast")
                    raise Exception(f"Circuit breaker open for {func.__name__}, failing fast")
            else:
                try:
                    result = func(*args, **kwargs)
                    circuit_state.failure_count = 0  # Reset on success
                    return result
                except Exception as e:
                    circuit_state.failure_count += 1
                    circuit_state.last_failure_time = time.time()
                    
                    if circuit_state.failure_count >= failure_threshold:
                        circuit_state.is_open = True
                        logging.error(f"Circuit breaker opened for {func.__name__} after {failure_threshold} failures")
                    
                    raise e
        
        return wrapper
    return decorator


# Example usage functions
if __name__ == "__main__":
    # Example of using the retry decorator
    @retry_on_failure(max_retries=3, delay=1, backoff=2, exceptions=(ConnectionError, TimeoutError))
    def unstable_api_call():
        import random
        if random.random() < 0.7:  # 70% chance of failure for testing
            raise ConnectionError("Simulated connection error")
        return "Success!"
    
    # Example of using rate limiting
    @rate_limit_handler(max_calls=2, time_window=5)  # 2 calls per 5 seconds
    def api_call():
        print(f"API call made at {time.time()}")
        return "API response"
    
    # Example of using circuit breaker
    @circuit_breaker(failure_threshold=3, recovery_timeout=10)
    def fragile_service_call():
        import random
        if random.random() < 0.6:  # 60% chance of failure for testing
            raise Exception("Service temporarily unavailable")
        return "Service response"
    
    # Test the decorators
    setup_logging = lambda: logging.basicConfig(level=logging.INFO)
    setup_logging()
    
    try:
        result = unstable_api_call()
        print(f"Result: {result}")
    except Exception as e:
        print(f"Failed after retries: {e}")
    
    for i in range(5):
        try:
            api_call()
            time.sleep(1)
        except Exception as e:
            print(f"API call failed: {e}")