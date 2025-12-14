from pydantic import BaseModel, Field
from datetime import datetime
from typing import List, Dict, Any, Optional
import uuid
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .detected_object import DetectedObject


class PerceptionData(BaseModel):
    """
    Visual and sensory information from the environment processed by computer vision
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    timestamp: datetime = Field(default_factory=datetime.now)
    sensor_type: str = Field(..., regex=r"^(camera|lidar|depth|other)$")
    data: Dict[str, Any] = Field(default_factory=dict)
    objects: List['DetectedObject'] = Field(default_factory=list)
    environment_state: Dict[str, Any] = Field(default_factory=dict)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    def add_detected_object(self, detected_object: 'DetectedObject'):
        """Add a detected object to the perception data."""
        self.objects.append(detected_object)
        self.update_environment_state()

    def update_environment_state(self):
        """Update the environment state based on detected objects."""
        self.environment_state = {
            "object_count": len(self.objects),
            "object_types": list(set(obj.name for obj in self.objects)),
            "timestamp": self.timestamp.isoformat()
        }

    def get_object_by_name(self, name: str) -> Optional['DetectedObject']:
        """Get a detected object by its name."""
        for obj in self.objects:
            if obj.name == name:
                return obj
        return None

    def get_objects_by_type(self, obj_type: str) -> List['DetectedObject']:
        """Get all objects of a specific type."""
        return [obj for obj in self.objects if obj.name == obj_type]