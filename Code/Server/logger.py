# -*- coding: utf-8 -*-
"""
Structured logging framework for Freenove Hexapod Robot Server.
Provides thread-safe logging with file rotation and configurable output.
"""

import logging
import logging.handlers
import sys
from pathlib import Path
from typing import Optional
import threading


class ThreadSafeLogger:
    """
    Thread-safe logging wrapper with structured output.
    Supports both console and file logging with rotation.
    """

    _instance: Optional['ThreadSafeLogger'] = None
    _lock: threading.Lock = threading.Lock()

    def __new__(cls):
        """Singleton pattern for logger instance."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """Initialize logger (only once due to singleton)."""
        if hasattr(self, '_initialized'):
            return
        self._initialized = True
        self._logger: Optional[logging.Logger] = None
        self._config_lock = threading.Lock()

    def setup(
        self,
        name: str = 'hexapod_server',
        log_level: str = 'INFO',
        log_to_console: bool = True,
        log_to_file: bool = True,
        log_file_path: str = '/tmp/hexapod_server.log',
        log_max_bytes: int = 10485760,  # 10 MB
        log_backup_count: int = 3,
        log_format: Optional[str] = None
    ) -> logging.Logger:
        """
        Setup logging configuration.

        Args:
            name: Logger name
            log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            log_to_console: Enable console output
            log_to_file: Enable file output
            log_file_path: Path to log file
            log_max_bytes: Max file size before rotation
            log_backup_count: Number of backup files to keep
            log_format: Custom log format string

        Returns:
            Configured logger instance
        """
        with self._config_lock:
            if self._logger is not None:
                return self._logger

            # Create logger
            self._logger = logging.getLogger(name)
            self._logger.setLevel(getattr(logging, log_level.upper()))
            self._logger.propagate = False

            # Clear any existing handlers
            self._logger.handlers.clear()

            # Create formatter
            if log_format is None:
                log_format = (
                    '%(asctime)s - [%(levelname)s] - '
                    '[%(threadName)-10s] - %(name)s - '
                    '%(filename)s:%(lineno)d - %(message)s'
                )
            formatter = logging.Formatter(
                log_format,
                datefmt='%Y-%m-%d %H:%M:%S'
            )

            # Console handler
            if log_to_console:
                console_handler = logging.StreamHandler(sys.stdout)
                console_handler.setLevel(getattr(logging, log_level.upper()))
                console_handler.setFormatter(formatter)
                self._logger.addHandler(console_handler)

            # File handler with rotation
            if log_to_file:
                # Create log directory if it doesn't exist
                log_path = Path(log_file_path)
                log_path.parent.mkdir(parents=True, exist_ok=True)

                file_handler = logging.handlers.RotatingFileHandler(
                    log_file_path,
                    maxBytes=log_max_bytes,
                    backupCount=log_backup_count,
                    encoding='utf-8'
                )
                file_handler.setLevel(getattr(logging, log_level.upper()))
                file_handler.setFormatter(formatter)
                self._logger.addHandler(file_handler)

            return self._logger

    def get_logger(self) -> logging.Logger:
        """
        Get the configured logger instance.

        Returns:
            Logger instance

        Raises:
            RuntimeError: If logger hasn't been setup yet
        """
        if self._logger is None:
            raise RuntimeError("Logger not setup. Call setup() first.")
        return self._logger


# Global logger instance
_logger_wrapper = ThreadSafeLogger()


def setup_logger(
    name: str = 'hexapod_server',
    log_level: str = 'INFO',
    log_to_console: bool = True,
    log_to_file: bool = True,
    log_file_path: str = '/tmp/hexapod_server.log',
    log_max_bytes: int = 10485760,
    log_backup_count: int = 3
) -> logging.Logger:
    """
    Setup the global logger instance.

    Args:
        name: Logger name
        log_level: Logging level
        log_to_console: Enable console output
        log_to_file: Enable file output
        log_file_path: Path to log file
        log_max_bytes: Max file size before rotation
        log_backup_count: Number of backup files

    Returns:
        Configured logger instance
    """
    return _logger_wrapper.setup(
        name=name,
        log_level=log_level,
        log_to_console=log_to_console,
        log_to_file=log_to_file,
        log_file_path=log_file_path,
        log_max_bytes=log_max_bytes,
        log_backup_count=log_backup_count
    )


def get_logger() -> logging.Logger:
    """
    Get the global logger instance.

    Returns:
        Logger instance
    """
    try:
        return _logger_wrapper.get_logger()
    except RuntimeError:
        # Setup with defaults if not yet configured
        return setup_logger()


def setup_from_config(config):
    """
    Setup logger from configuration object.

    Args:
        config: Configuration object with logging attribute

    Returns:
        Configured logger instance
    """
    return setup_logger(
        log_level=config.logging.log_level,
        log_to_console=config.logging.log_to_console,
        log_to_file=config.logging.log_to_file,
        log_file_path=config.logging.log_file_path,
        log_max_bytes=config.logging.log_max_bytes,
        log_backup_count=config.logging.log_backup_count
    )


# Convenience functions for common log levels
def debug(msg: str, *args, **kwargs):
    """Log debug message."""
    get_logger().debug(msg, *args, **kwargs)


def info(msg: str, *args, **kwargs):
    """Log info message."""
    get_logger().info(msg, *args, **kwargs)


def warning(msg: str, *args, **kwargs):
    """Log warning message."""
    get_logger().warning(msg, *args, **kwargs)


def error(msg: str, *args, **kwargs):
    """Log error message."""
    get_logger().error(msg, *args, **kwargs)


def critical(msg: str, *args, **kwargs):
    """Log critical message."""
    get_logger().critical(msg, *args, **kwargs)


def exception(msg: str, *args, **kwargs):
    """Log exception with traceback."""
    get_logger().exception(msg, *args, **kwargs)


if __name__ == '__main__':
    # Example usage
    logger = setup_logger(
        log_level='DEBUG',
        log_to_console=True,
        log_to_file=True,
        log_file_path='/tmp/hexapod_test.log'
    )

    logger.debug("This is a debug message")
    logger.info("This is an info message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")

    try:
        raise ValueError("Test exception")
    except ValueError:
        logger.exception("Exception occurred during testing")

    print("Logger test complete. Check /tmp/hexapod_test.log")
