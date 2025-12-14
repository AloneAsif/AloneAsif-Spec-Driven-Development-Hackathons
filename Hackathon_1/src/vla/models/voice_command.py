from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional
import uuid


class VoiceCommand(BaseModel):
    """
    Natural language input from user that specifies desired robot behavior
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    text: str = Field(..., min_length=1)
    timestamp: datetime = Field(default_factory=datetime.now)
    confidence: float = Field(ge=0.0, le=1.0, default=1.0)
    raw_audio: Optional[bytes] = None
    status: str = Field(default="pending", regex=r"^(pending|processing|completed|failed)$")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    @classmethod
    def create_from_audio(cls, audio_data: bytes, text: str = None, confidence: float = 1.0):
        """Create a VoiceCommand from audio data with optional transcription."""
        return cls(
            raw_audio=audio_data,
            text=text or "",
            confidence=confidence,
            status="pending"
        )

    def mark_as_processed(self, transcribed_text: str, confidence: float):
        """Update the command with transcription results."""
        self.text = transcribed_text
        self.confidence = confidence
        self.status = "processing"

    def mark_as_completed(self):
        """Mark the command as completed."""
        self.status = "completed"

    def mark_as_failed(self):
        """Mark the command as failed."""
        self.status = "failed"