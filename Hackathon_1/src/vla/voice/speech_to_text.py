"""
Speech-to-text service for the VLA system.

This module provides a comprehensive speech-to-text service that handles
audio preprocessing, Whisper integration, and text processing.
"""

import io
import wave
import numpy as np
import librosa
from typing import Optional, Dict, Any, Tuple
from pydantic import BaseModel
from ..models.voice_command import VoiceCommand
from ..utils import VLALogger, VLAException, get_logger
from .whisper_interface import WhisperInterface, WhisperResponse
from ..config import get_config


class AudioProcessingResult(BaseModel):
    """
    Result model for audio processing operations.
    """
    processed_audio: bytes
    sample_rate: int
    duration: float
    channels: int


class SpeechToTextService:
    """
    Service for converting speech to text with preprocessing and validation.
    """
    def __init__(self, whisper_interface: Optional[WhisperInterface] = None):
        """
        Initialize the speech-to-text service.

        Args:
            whisper_interface: Optional pre-configured Whisper interface
        """
        self.logger = get_logger("SpeechToTextService")
        self.config = get_config()

        # Initialize Whisper interface if not provided
        if whisper_interface:
            self.whisper = whisper_interface
        else:
            api_key = self.config.settings.openai_api_key
            model = self.config.settings.whisper_model
            self.whisper = WhisperInterface(api_key=api_key, model=model)

        self.default_sample_rate = self.config.settings.whisper_sample_rate

    async def process_audio(
        self,
        audio_data: bytes,
        sample_rate: Optional[int] = None,
        language: Optional[str] = None
    ) -> Tuple[str, float]:
        """
        Process audio data and convert to text.

        Args:
            audio_data: Raw audio bytes
            sample_rate: Sample rate of the audio (will be detected if not provided)
            language: Language of the audio (optional)

        Returns:
            Tuple of (transcribed_text, confidence_score)
        """
        try:
            # Preprocess the audio
            processed_result = self._preprocess_audio(audio_data, sample_rate)
            self.logger.info(
                f"Audio preprocessed successfully",
                extra_data={
                    "duration": processed_result.duration,
                    "sample_rate": processed_result.sample_rate,
                    "channels": processed_result.channels
                }
            )

            # Transcribe the audio
            whisper_response = await self.whisper.transcribe_audio(
                processed_result.processed_audio,
                language=language
            )

            self.logger.info(
                f"Speech-to-text conversion completed",
                extra_data={
                    "text_length": len(whisper_response.text),
                    "confidence": whisper_response.confidence
                }
            )

            return whisper_response.text, whisper_response.confidence

        except Exception as e:
            self.logger.error(
                f"Error in speech-to-text processing: {str(e)}",
                extra_data={"error_type": type(e).__name__}
            )
            raise VLAException(
                f"Speech-to-text processing failed: {str(e)}",
                "SPEECH_TO_TEXT_ERROR",
                {"error_type": type(e).__name__}
            )

    def _preprocess_audio(self, audio_data: bytes, sample_rate: Optional[int] = None) -> AudioProcessingResult:
        """
        Preprocess audio data for optimal Whisper performance.

        Args:
            audio_data: Raw audio bytes
            sample_rate: Target sample rate (will be detected if not provided)

        Returns:
            AudioProcessingResult with processed audio and metadata
        """
        try:
            # Load audio with librosa
            audio, detected_sr = librosa.load(io.BytesIO(audio_data), sr=sample_rate)

            # Resample if needed
            target_sr = sample_rate or self.default_sample_rate
            if detected_sr != target_sr:
                audio = librosa.resample(audio, orig_sr=detected_sr, target_sr=target_sr)

            # Apply noise reduction (simplified)
            audio = self._reduce_noise(audio)

            # Normalize volume
            audio = librosa.util.normalize(audio)

            # Convert back to bytes for Whisper API
            processed_audio_bytes = self._audio_to_bytes(audio, target_sr)

            duration = len(audio) / target_sr

            result = AudioProcessingResult(
                processed_audio=processed_audio_bytes,
                sample_rate=target_sr,
                duration=duration,
                channels=1  # Assuming mono for simplicity
            )

            return result

        except Exception as e:
            self.logger.error(
                f"Error preprocessing audio: {str(e)}",
                extra_data={"error_type": type(e).__name__}
            )
            raise VLAException(
                f"Audio preprocessing failed: {str(e)}",
                "AUDIO_PREPROCESSING_ERROR",
                {"error_type": type(e).__name__}
            )

    def _reduce_noise(self, audio: np.ndarray) -> np.ndarray:
        """
        Apply basic noise reduction to audio.

        Args:
            audio: Audio signal as numpy array

        Returns:
            Noise-reduced audio signal
        """
        # Simple noise reduction using spectral gating
        # In a real implementation, you might use more sophisticated techniques

        # Estimate noise during quiet periods (first 0.5 seconds)
        noise_sample_size = int(0.5 * self.default_sample_rate)
        if len(audio) > noise_sample_size:
            noise_sample = audio[:noise_sample_size]
            noise_mean = np.mean(np.abs(noise_sample))
        else:
            noise_mean = 0.0

        # Apply thresholding based on noise estimate
        threshold = max(0.01, noise_mean * 2)  # Minimum threshold
        audio_clean = np.where(np.abs(audio) > threshold, audio, audio * 0.1)  # Attenuate quiet parts

        return audio_clean

    def _audio_to_bytes(self, audio: np.ndarray, sample_rate: int) -> bytes:
        """
        Convert numpy audio array to bytes suitable for Whisper API.

        Args:
            audio: Audio signal as numpy array
            sample_rate: Sample rate of the audio

        Returns:
            Audio data as bytes
        """
        # Normalize to 16-bit range
        audio_normalized = np.int16(audio / np.max(np.abs(audio)) * 32767)

        # Create WAV file in memory
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio_normalized.tobytes())

        return wav_buffer.getvalue()

    async def create_voice_command(self, audio_data: bytes) -> VoiceCommand:
        """
        Create a VoiceCommand object from audio data.

        Args:
            audio_data: Raw audio bytes

        Returns:
            VoiceCommand object with transcription
        """
        try:
            # Process the audio to get text and confidence
            text, confidence = await self.process_audio(audio_data)

            # Validate confidence threshold
            min_confidence = 0.3  # Configurable threshold
            if confidence < min_confidence:
                self.logger.warning(
                    f"Low confidence transcription",
                    extra_data={
                        "confidence": confidence,
                        "threshold": min_confidence,
                        "text": text
                    }
                )

            # Create and return VoiceCommand object
            voice_command = VoiceCommand.create_from_audio(
                audio_data=audio_data,
                text=text,
                confidence=confidence
            )

            self.logger.info(
                f"Voice command created successfully",
                extra_data={
                    "command_id": voice_command.id,
                    "text_length": len(text),
                    "confidence": confidence
                }
            )

            return voice_command

        except Exception as e:
            self.logger.error(
                f"Error creating voice command: {str(e)}",
                extra_data={"error_type": type(e).__name__}
            )
            raise VLAException(
                f"Failed to create voice command: {str(e)}",
                "VOICE_COMMAND_CREATION_ERROR",
                {"error_type": type(e).__name__}
            )

    def validate_audio_format(self, audio_data: bytes) -> bool:
        """
        Validate that the audio data is in a supported format.

        Args:
            audio_data: Raw audio bytes

        Returns:
            True if format is supported, False otherwise
        """
        try:
            # Try to load the audio with librosa
            audio, sr = librosa.load(io.BytesIO(audio_data))
            return len(audio) > 0 and sr > 0
        except Exception:
            return False

    async def process_voice_command(self, audio_data: bytes) -> Dict[str, Any]:
        """
        Process a voice command and return structured results.

        Args:
            audio_data: Raw audio bytes

        Returns:
            Dictionary with processing results
        """
        try:
            # Validate audio format first
            if not self.validate_audio_format(audio_data):
                raise VLAException(
                    "Unsupported audio format",
                    "UNSUPPORTED_AUDIO_FORMAT"
                )

            # Create voice command
            voice_command = await self.create_voice_command(audio_data)

            # Process the command further if needed
            result = {
                "voice_command": voice_command.dict(),
                "processing_status": "success",
                "timestamp": voice_command.timestamp.isoformat()
            }

            self.logger.info(
                f"Voice command processed successfully",
                extra_data={
                    "command_id": voice_command.id,
                    "text": voice_command.text
                }
            )

            return result

        except Exception as e:
            self.logger.error(
                f"Error processing voice command: {str(e)}",
                extra_data={"error_type": type(e).__name__}
            )
            raise VLAException(
                f"Voice command processing failed: {str(e)}",
                "VOICE_COMMAND_PROCESSING_ERROR",
                {"error_type": type(e).__name__}
            )


# For testing purposes
if __name__ == "__main__":
    import asyncio

    async def test_speech_to_text():
        # This is just for testing - would need actual Whisper credentials
        try:
            service = SpeechToTextService()
            print("Speech-to-text service initialized successfully")
        except Exception as e:
            print(f"Error initializing service: {e}")

    # asyncio.run(test_speech_to_text())