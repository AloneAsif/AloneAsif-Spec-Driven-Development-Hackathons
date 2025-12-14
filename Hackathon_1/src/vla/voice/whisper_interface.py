"""
Whisper interface for the VLA system.

This module provides an interface to OpenAI's Whisper API for speech-to-text conversion.
"""

import asyncio
import io
import tempfile
import wave
from typing import Optional, Dict, Any
import aiohttp
import openai
from pydantic import BaseModel
from ..utils import VLALogger, VLAException, get_logger


class WhisperResponse(BaseModel):
    """
    Response model for Whisper transcription results.
    """
    text: str
    confidence: float
    language: str
    duration: float
    segments: Optional[list] = None


class WhisperInterface:
    """
    Interface for interacting with OpenAI Whisper API.
    """
    def __init__(self, api_key: Optional[str] = None, model: str = "whisper-1"):
        """
        Initialize the Whisper interface.

        Args:
            api_key: OpenAI API key (if not provided, will use environment variable)
            model: Whisper model to use (default: whisper-1)
        """
        self.logger = get_logger("WhisperInterface")

        if api_key:
            openai.api_key = api_key
        elif not openai.api_key:
            raise VLAException(
                "OpenAI API key is required for Whisper interface",
                "MISSING_API_KEY"
            )

        self.model = model

    async def transcribe_audio(
        self,
        audio_data: bytes,
        language: Optional[str] = None,
        response_format: str = "verbose_json",
        temperature: float = 0.0
    ) -> WhisperResponse:
        """
        Transcribe audio data using OpenAI Whisper API.

        Args:
            audio_data: Raw audio bytes
            language: Language of the audio (optional, Whisper will auto-detect if not provided)
            response_format: Format of the response (default: verbose_json)
            temperature: Temperature for the transcription (default: 0.0)

        Returns:
            WhisperResponse containing the transcription and metadata
        """
        try:
            # Create a temporary file to save the audio data
            with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as temp_audio:
                temp_audio.write(audio_data)
                temp_audio_path = temp_audio.name

            try:
                # Transcribe using OpenAI API
                with open(temp_audio_path, "rb") as audio_file:
                    transcript = await openai.Audio.atranscribe(
                        model=self.model,
                        file=audio_file,
                        language=language,
                        response_format=response_format,
                        temperature=temperature,
                        timestamp_granularities=["segment"] if response_format == "verbose_json" else None
                    )

                # Calculate confidence from segments if available
                confidence = self._calculate_confidence(transcript)

                # Create response object
                response = WhisperResponse(
                    text=transcript.text if hasattr(transcript, 'text') else str(transcript),
                    confidence=confidence,
                    language=getattr(transcript, 'language', language or 'unknown'),
                    duration=getattr(transcript, 'duration', 0.0),
                    segments=getattr(transcript, 'segments', None)
                )

                self.logger.info(
                    f"Audio transcribed successfully",
                    extra_data={
                        "duration": response.duration,
                        "confidence": response.confidence,
                        "text_length": len(response.text)
                    }
                )

                return response

            finally:
                # Clean up temporary file
                import os
                os.unlink(temp_audio_path)

        except Exception as e:
            self.logger.error(
                f"Error transcribing audio: {str(e)}",
                extra_data={"error_type": type(e).__name__}
            )
            raise VLAException(
                f"Failed to transcribe audio: {str(e)}",
                "WHISPER_TRANSCRIPTION_ERROR",
                {"error_type": type(e).__name__}
            )

    def _calculate_confidence(self, transcript: Any) -> float:
        """
        Calculate confidence score from transcript segments.

        Args:
            transcript: Transcript object from Whisper API

        Returns:
            Confidence score between 0.0 and 1.0
        """
        try:
            if hasattr(transcript, 'segments') and transcript.segments:
                # Calculate average confidence from segments
                confidences = []
                for segment in transcript.segments:
                    if isinstance(segment, dict):
                        # Check for various confidence indicators
                        if 'confidence' in segment:
                            confidences.append(segment['confidence'])
                        elif 'avg_logprob' in segment:
                            # Convert log probability to confidence (rough approximation)
                            avg_logprob = segment['avg_logprob']
                            # Normalize logprob to 0-1 range (roughly)
                            confidence = max(0, min(1, (avg_logprob + 5) / 10))
                            confidences.append(confidence)
                        elif 'temperature' in segment:
                            # Lower temperature generally means higher confidence
                            temp = segment['temperature']
                            confidence = max(0, min(1, (1 - temp)))
                            confidences.append(confidence)

                if confidences:
                    return sum(confidences) / len(confidences)

            # Default confidence if no segment data available
            return 0.8

        except Exception as e:
            self.logger.warning(
                f"Error calculating confidence: {str(e)}, using default",
                extra_data={"default_confidence": 0.8}
            )
            return 0.8

    async def transcribe_audio_file(self, file_path: str) -> WhisperResponse:
        """
        Transcribe an audio file using OpenAI Whisper API.

        Args:
            file_path: Path to the audio file

        Returns:
            WhisperResponse containing the transcription and metadata
        """
        try:
            with open(file_path, "rb") as audio_file:
                transcript = await openai.Audio.atranscribe(
                    model=self.model,
                    file=audio_file,
                    response_format="verbose_json",
                    timestamp_granularities=["segment"]
                )

            confidence = self._calculate_confidence(transcript)

            response = WhisperResponse(
                text=transcript.text,
                confidence=confidence,
                language=transcript.language,
                duration=transcript.duration,
                segments=transcript.segments
            )

            self.logger.info(
                f"Audio file transcribed successfully: {file_path}",
                extra_data={
                    "duration": response.duration,
                    "confidence": response.confidence,
                    "text_length": len(response.text)
                }
            )

            return response

        except Exception as e:
            self.logger.error(
                f"Error transcribing audio file {file_path}: {str(e)}",
                extra_data={"error_type": type(e).__name__, "file_path": file_path}
            )
            raise VLAException(
                f"Failed to transcribe audio file {file_path}: {str(e)}",
                "WHISPER_TRANSCRIPTION_ERROR",
                {"file_path": file_path, "error_type": type(e).__name__}
            )


# For testing purposes
if __name__ == "__main__":
    import asyncio

    async def test_whisper():
        # This is just for testing - in real usage, provide actual API key
        try:
            whisper = WhisperInterface(api_key="test-key")
            # Test with empty audio - this will fail but shows the structure
            print("Whisper interface initialized successfully")
        except Exception as e:
            print(f"Error initializing whisper: {e}")

    # asyncio.run(test_whisper())