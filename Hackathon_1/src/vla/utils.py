"""
Utility functions for the VLA system including error handling and logging infrastructure.
"""

import logging
import sys
from datetime import datetime
from typing import Dict, Any, Optional
from enum import Enum
import traceback
import json


class LogLevel(str, Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class VLALogger:
    """
    Custom logger for the VLA system with structured logging capabilities.
    """
    def __init__(self, name: str = "VLA", level: LogLevel = LogLevel.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.value))

        # Create formatter with structured format
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

        # Add console handler if not already present
        if not self.logger.handlers:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

    def _log_structured(self, level: LogLevel, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log a structured message with optional extra data."""
        log_data = {
            "timestamp": datetime.now().isoformat(),
            "level": level.value,
            "message": message,
            "extra_data": extra_data or {}
        }

        log_message = json.dumps(log_data)
        getattr(self.logger, level.value.lower())(log_message)

    def debug(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log a debug message."""
        self._log_structured(LogLevel.DEBUG, message, extra_data)

    def info(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log an info message."""
        self._log_structured(LogLevel.INFO, message, extra_data)

    def warning(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log a warning message."""
        self._log_structured(LogLevel.WARNING, message, extra_data)

    def error(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log an error message."""
        self._log_structured(LogLevel.ERROR, message, extra_data)

    def critical(self, message: str, extra_data: Optional[Dict[str, Any]] = None):
        """Log a critical message."""
        self._log_structured(LogLevel.CRITICAL, message, extra_data)


class VLAException(Exception):
    """
    Base exception class for the VLA system.
    """
    def __init__(self, message: str, error_code: Optional[str] = None, details: Optional[Dict[str, Any]] = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code or "VLA_ERROR"
        self.details = details or {}
        self.timestamp = datetime.now().isoformat()
        self.traceback = traceback.format_exc()

    def to_dict(self) -> Dict[str, Any]:
        """Convert the exception to a dictionary for structured logging."""
        return {
            "error_type": self.__class__.__name__,
            "message": self.message,
            "error_code": self.error_code,
            "details": self.details,
            "timestamp": self.timestamp,
            "traceback": self.traceback
        }


class ValidationError(VLAException):
    """Exception raised for validation errors."""
    def __init__(self, message: str, field: Optional[str] = None, value: Any = None):
        details = {"field": field, "value": value} if field or value else {}
        super().__init__(message, "VALIDATION_ERROR", details)


class ProcessingError(VLAException):
    """Exception raised for processing errors."""
    def __init__(self, message: str, component: Optional[str] = None):
        details = {"component": component} if component else {}
        super().__init__(message, "PROCESSING_ERROR", details)


class CommunicationError(VLAException):
    """Exception raised for communication errors (e.g., with ROS2)."""
    def __init__(self, message: str, service: Optional[str] = None):
        details = {"service": service} if service else {}
        super().__init__(message, "COMMUNICATION_ERROR", details)


class ConfigurationError(VLAException):
    """Exception raised for configuration errors."""
    def __init__(self, message: str, config_key: Optional[str] = None):
        details = {"config_key": config_key} if config_key else {}
        super().__init__(message, "CONFIGURATION_ERROR", details)


def handle_vla_exception(exc: VLAException, logger: Optional[VLALogger] = None) -> Dict[str, Any]:
    """
    Handle a VLA exception by logging it and returning a structured response.
    """
    if logger:
        logger.error(
            f"VLA Exception: {exc.message}",
            extra_data=exc.to_dict()
        )
    else:
        # Fallback logging if no logger provided
        logging.error(f"VLA Exception: {exc.message}", extra=exc.to_dict())

    return {
        "error": exc.error_code,
        "message": exc.message,
        "details": exc.details,
        "timestamp": exc.timestamp
    }


def validate_required_fields(data: Dict[str, Any], required_fields: list) -> None:
    """
    Validate that required fields are present in the data.

    Args:
        data: Dictionary to validate
        required_fields: List of required field names

    Raises:
        ValidationError: If any required field is missing
    """
    missing_fields = []
    for field in required_fields:
        if field not in data or data[field] is None:
            missing_fields.append(field)

    if missing_fields:
        raise ValidationError(
            f"Missing required fields: {', '.join(missing_fields)}",
            field=", ".join(missing_fields)
        )


def safe_execute(func, logger: Optional[VLALogger] = None, *args, **kwargs):
    """
    Safely execute a function, catching and handling exceptions.

    Args:
        func: Function to execute
        logger: Optional logger to use for error logging
        *args: Arguments to pass to the function
        **kwargs: Keyword arguments to pass to the function

    Returns:
        Result of the function execution or None if an exception occurred
    """
    try:
        return func(*args, **kwargs)
    except VLAException as e:
        handle_vla_exception(e, logger)
        return None
    except Exception as e:
        # Wrap unexpected exceptions in a VLAException
        vla_exc = VLAException(f"Unexpected error: {str(e)}", "UNEXPECTED_ERROR")
        handle_vla_exception(vla_exc, logger)
        return None


# Global logger instance
vla_logger = VLALogger()


def get_logger(name: str = "VLA") -> VLALogger:
    """
    Get a logger instance for the VLA system.

    Args:
        name: Name for the logger (default: "VLA")

    Returns:
        VLALogger instance
    """
    return VLALogger(name=name)