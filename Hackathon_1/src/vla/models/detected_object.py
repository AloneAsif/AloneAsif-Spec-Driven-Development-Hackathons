from pydantic import BaseModel, Field
from typing import Dict, Any
import uuid


class DetectedObject(BaseModel):
    """
    Object detected by the vision system
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str = Field(..., min_length=1)
    confidence: float = Field(ge=0.0, le=1.0, default=1.0)
    position: Dict[str, float] = Field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})  # 3D coordinates
    bounding_box: Dict[str, int] = Field(default_factory=lambda: {"x": 0, "y": 0, "width": 0, "height": 0})  # 2D coordinates
    properties: Dict[str, Any] = Field(default_factory=dict)

    def is_valid_detection(self) -> bool:
        """Check if this is a valid detection based on confidence."""
        return self.confidence >= 0.5  # Threshold can be adjusted

    def distance_from(self, other_position: Dict[str, float]) -> float:
        """Calculate 3D distance from another position."""
        dx = self.position["x"] - other_position["x"]
        dy = self.position["y"] - other_position["y"]
        dz = self.position["z"] - other_position["z"]
        return (dx**2 + dy**2 + dz**2)**0.5

    def is_similar_to(self, other: 'DetectedObject', threshold: float = 0.1) -> bool:
        """Check if this object is similar to another object."""
        if self.name != other.name:
            return False

        # Check if positions are close enough
        distance = self.distance_from(other.position)
        return distance <= threshold

    def update_position(self, new_position: Dict[str, float]):
        """Update the object's position."""
        self.position.update(new_position)