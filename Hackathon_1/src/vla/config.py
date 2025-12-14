"""
Configuration management for the VLA system.

This module handles environment configuration and settings management
using python-dotenv for environment variable management.
"""

import os
from typing import Optional, Dict, Any
from dotenv import load_dotenv
try:
    from pydantic import Field
    from pydantic_settings import BaseSettings
except ImportError:
    from pydantic import BaseSettings, Field
import logging


# Load environment variables from .env file
load_dotenv()


class VLASettings(BaseSettings):
    """
    Settings class for the VLA system using Pydantic BaseSettings.
    """
    # Voice processing settings
    whisper_model: str = Field(default="base", env="VLA_WHISPER_MODEL")
    whisper_language: str = Field(default="en", env="VLA_WHISPER_LANGUAGE")
    whisper_sample_rate: int = Field(default=16000, env="VLA_WHISPER_SAMPLE_RATE")

    # LLM settings
    llm_provider: str = Field(default="openai", env="VLA_LLM_PROVIDER")
    llm_model: str = Field(default="gpt-4-turbo", env="VLA_LLM_MODEL")
    llm_temperature: float = Field(default=0.3, env="VLA_LLM_TEMPERATURE")
    openai_api_key: Optional[str] = Field(default=None, env="OPENAI_API_KEY")

    # Vision settings
    vision_detection_model: str = Field(default="yolov8n.pt", env="VLA_VISION_DETECTION_MODEL")
    vision_confidence_threshold: float = Field(default=0.5, env="VLA_VISION_CONFIDENCE_THRESHOLD")
    vision_image_topic: str = Field(default="/camera/rgb/image_raw", env="VLA_VISION_IMAGE_TOPIC")

    # ROS2 settings
    ros2_navigation_action: str = Field(default="/navigate_to_pose", env="VLA_ROS2_NAVIGATION_ACTION")
    ros2_manipulation_action: str = Field(default="/manipulation_controller", env="VLA_ROS2_MANIPULATION_ACTION")
    ros2_timeout: float = Field(default=30.0, env="VLA_ROS2_TIMEOUT")
    ros2_domain_id: int = Field(default=42, env="ROS_DOMAIN_ID")

    # System settings
    log_level: str = Field(default="INFO", env="VLA_LOG_LEVEL")
    debug_mode: bool = Field(default=False, env="VLA_DEBUG_MODE")
    simulation_speed: float = Field(default=1.0, env="VLA_SIMULATION_SPEED")

    # API settings
    api_host: str = Field(default="0.0.0.0", env="VLA_API_HOST")
    api_port: int = Field(default=8000, env="VLA_API_PORT")
    cors_origins: str = Field(default="*", env="VLA_CORS_ORIGINS")

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False


class ConfigManager:
    """
    Configuration manager for the VLA system.
    """
    def __init__(self):
        self.settings = VLASettings()
        self._setup_logging()

    def _setup_logging(self):
        """
        Setup logging based on configuration.
        """
        level = getattr(logging, self.settings.log_level.upper(), logging.INFO)
        logging.basicConfig(
            level=level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

    def get_voice_settings(self) -> Dict[str, Any]:
        """
        Get voice processing settings.
        """
        return {
            "model": self.settings.whisper_model,
            "language": self.settings.whisper_language,
            "sample_rate": self.settings.whisper_sample_rate
        }

    def get_llm_settings(self) -> Dict[str, Any]:
        """
        Get LLM settings.
        """
        return {
            "provider": self.settings.llm_provider,
            "model": self.settings.llm_model,
            "temperature": self.settings.llm_temperature,
            "api_key": self.settings.openai_api_key
        }

    def get_vision_settings(self) -> Dict[str, Any]:
        """
        Get vision processing settings.
        """
        return {
            "detection_model": self.settings.vision_detection_model,
            "confidence_threshold": self.settings.vision_confidence_threshold,
            "image_topic": self.settings.vision_image_topic
        }

    def get_ros2_settings(self) -> Dict[str, Any]:
        """
        Get ROS2 settings.
        """
        return {
            "navigation_action": self.settings.ros2_navigation_action,
            "manipulation_action": self.settings.ros2_manipulation_action,
            "timeout": self.settings.ros2_timeout,
            "domain_id": self.settings.ros2_domain_id
        }

    def get_system_settings(self) -> Dict[str, Any]:
        """
        Get system settings.
        """
        return {
            "log_level": self.settings.log_level,
            "debug_mode": self.settings.debug_mode,
            "simulation_speed": self.settings.simulation_speed
        }

    def get_api_settings(self) -> Dict[str, Any]:
        """
        Get API settings.
        """
        return {
            "host": self.settings.api_host,
            "port": self.settings.api_port,
            "cors_origins": self.settings.cors_origins.split(",") if self.settings.cors_origins else ["*"]
        }

    def validate_config(self) -> bool:
        """
        Validate the configuration.
        """
        # Check if required API keys are present
        if self.settings.llm_provider == "openai" and not self.settings.openai_api_key:
            raise ValueError("OPENAI_API_KEY is required when using OpenAI as LLM provider")

        # Validate log level
        try:
            getattr(logging, self.settings.log_level.upper())
        except AttributeError:
            raise ValueError(f"Invalid log level: {self.settings.log_level}")

        return True

    def reload_config(self):
        """
        Reload configuration from environment variables.
        """
        load_dotenv(override=True)
        self.settings = VLASettings()
        self._setup_logging()


# Global configuration instance
config_manager = ConfigManager()


def get_config() -> ConfigManager:
    """
    Get the global configuration manager instance.
    """
    return config_manager


def get_settings() -> VLASettings:
    """
    Get the current settings.
    """
    return config_manager.settings


# Create a default .env file template if it doesn't exist
def create_default_env_file():
    """
    Create a default .env file with template values.
    """
    env_content = """# VLA System Configuration

# Voice processing settings
VLA_WHISPER_MODEL=base
VLA_WHISPER_LANGUAGE=en
VLA_WHISPER_SAMPLE_RATE=16000

# LLM settings
VLA_LLM_PROVIDER=openai
VLA_LLM_MODEL=gpt-4-turbo
VLA_LLM_TEMPERATURE=0.3
OPENAI_API_KEY=your_openai_api_key_here

# Vision settings
VLA_VISION_DETECTION_MODEL=yolov8n.pt
VLA_VISION_CONFIDENCE_THRESHOLD=0.5
VLA_VISION_IMAGE_TOPIC=/camera/rgb/image_raw

# ROS2 settings
VLA_ROS2_NAVIGATION_ACTION=/navigate_to_pose
VLA_ROS2_MANIPULATION_ACTION=/manipulation_controller
VLA_ROS2_TIMEOUT=30.0
ROS_DOMAIN_ID=42

# System settings
VLA_LOG_LEVEL=INFO
VLA_DEBUG_MODE=false
VLA_SIMULATION_SPEED=1.0

# API settings
VLA_API_HOST=0.0.0.0
VLA_API_PORT=8000
VLA_CORS_ORIGINS=*
"""

    if not os.path.exists(".env"):
        with open(".env", "w") as f:
            f.write(env_content)
        print("Created default .env file with template values")
    else:
        print(".env file already exists")


# Create default .env file if needed
create_default_env_file()