---
sidebar_position: 3
---

# Voice-to-Text Processing with OpenAI Whisper

## Overview

This section covers the voice-to-text component of the VLA system, which uses OpenAI Whisper to convert spoken commands into structured text that can be processed by the planning system.

## Introduction to Whisper

OpenAI Whisper is a state-of-the-art speech recognition model that provides robust transcription capabilities across multiple languages and audio conditions. In the VLA system, Whisper serves as the primary interface for receiving natural language commands from users.

### Key Features
- High accuracy across diverse audio conditions
- Support for multiple languages
- Robust to background noise and audio quality variations
- Both API and self-hosted deployment options

## System Architecture

The voice-to-text pipeline consists of several components:

```
Microphone → Audio Preprocessing → Whisper → Text Processing → Intent Extraction
```

### Audio Input and Preprocessing
The system captures audio through various input methods:
- Direct microphone input
- Audio file upload
- Streaming audio from simulation environment

Audio preprocessing includes:
- Noise reduction
- Audio format normalization
- Volume normalization
- Silence detection and removal

### Whisper Integration
Whisper processes the audio and generates:
- Transcribed text
- Confidence scores
- Timestamp information
- Language detection

## Implementation Details

### Audio Handling

The audio input system supports multiple formats and provides preprocessing capabilities:

```python
import librosa
import numpy as np
from typing import Optional

class AudioPreprocessor:
    def __init__(self, sample_rate: int = 16000):
        self.sample_rate = sample_rate

    def preprocess_audio(self, audio_data: bytes) -> np.ndarray:
        """Preprocess raw audio data for Whisper processing."""
        # Load audio with librosa
        audio, sr = librosa.load(io.BytesIO(audio_data), sr=self.sample_rate)

        # Apply noise reduction
        audio = self.reduce_noise(audio)

        # Normalize volume
        audio = librosa.util.normalize(audio)

        return audio

    def reduce_noise(self, audio: np.ndarray) -> np.ndarray:
        """Apply basic noise reduction to audio."""
        # Implement noise reduction algorithm
        return audio
```

### Whisper Interface

The Whisper interface handles communication with the Whisper model:

```python
import openai
from pydantic import BaseModel
from typing import Optional

class WhisperResponse(BaseModel):
    text: str
    confidence: float
    language: str
    duration: float

class WhisperInterface:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.model = "whisper-1"

    async def transcribe_audio(self, audio_file_path: str) -> WhisperResponse:
        """Transcribe audio file using OpenAI Whisper API."""
        with open(audio_file_path, "rb") as audio_file:
            transcript = await openai.Audio.atranscribe(
                "whisper-1",
                audio_file,
                response_format="verbose_json",
                timestamp_granularities=["segment"]
            )

        # Calculate confidence based on segment confidence scores
        confidence = self._calculate_confidence(transcript)

        return WhisperResponse(
            text=transcript.text,
            confidence=confidence,
            language=transcript.language,
            duration=transcript.duration
        )

    def _calculate_confidence(self, transcript) -> float:
        """Calculate overall confidence from segment confidences."""
        if hasattr(transcript, 'segments'):
            confidences = [seg.get('avg_logprob', 0) for seg in transcript.segments]
            if confidences:
                # Normalize logprob to 0-1 confidence range
                avg_logprob = sum(confidences) / len(confidences)
                # Convert logprob to confidence (rough approximation)
                confidence = max(0, min(1, (avg_logprob + 5) / 10))
                return confidence

        return 0.8  # Default confidence if no segment data
```

## Voice Command Processing

Once audio is transcribed, the system processes the text to extract actionable commands:

### Command Validation
- Grammar and syntax validation
- Confidence threshold checking
- Context relevance assessment

### Intent Extraction
The system identifies user intent from transcribed text:
- Navigation commands ("Go to the kitchen")
- Manipulation commands ("Pick up the red cup")
- Complex multi-step commands ("Clean the room")

## Error Handling and Recovery

### Low Confidence Detection
When Whisper returns low-confidence transcriptions:
- Request user to repeat the command
- Provide alternative interpretations
- Use context to disambiguate

### Audio Quality Issues
For poor audio quality:
- Prompt user for clearer audio
- Provide audio quality feedback
- Offer text-based alternative

## Integration with VLA System

The voice-to-text component integrates with the broader VLA system:

```python
class VoiceCommandProcessor:
    def __init__(self, whisper_interface: WhisperInterface):
        self.whisper = whisper_interface

    async def process_voice_command(self, audio_data: bytes) -> dict:
        """Process voice command and return structured intent."""
        # Preprocess audio
        processed_audio = self.preprocess_audio(audio_data)

        # Transcribe using Whisper
        transcription = await self.whisper.transcribe_audio(processed_audio)

        # Validate confidence
        if transcription.confidence < 0.5:
            return {
                "status": "low_confidence",
                "message": "Could not understand command clearly. Please repeat.",
                "confidence": transcription.confidence
            }

        # Extract intent and return structured command
        structured_command = self.extract_intent(transcription.text)

        return {
            "status": "success",
            "command": structured_command,
            "text": transcription.text,
            "confidence": transcription.confidence
        }
```

## Performance Considerations

### Latency Optimization
- Use appropriate Whisper model size based on latency requirements
- Implement audio buffering for continuous listening
- Cache frequently used commands

### Resource Management
- Manage concurrent transcription requests
- Implement rate limiting for API calls
- Provide local Whisper model option for offline use

## Testing and Validation

### Unit Tests
Test individual components:
- Audio preprocessing functions
- Whisper API integration
- Confidence calculation

### Integration Tests
Validate end-to-end functionality:
- Audio input to text output
- Confidence threshold validation
- Error handling scenarios

## Security Considerations

- Secure API key storage and access
- Validate and sanitize all audio inputs
- Implement proper authentication for voice commands
- Protect against audio-based attacks