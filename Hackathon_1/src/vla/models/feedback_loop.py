from pydantic import BaseModel, Field
from datetime import datetime
from typing import Dict, Any
import uuid


class FeedbackLoop(BaseModel):
    """
    Process where perception results are fed back to update the plan or confirm completion
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    plan_id: str
    perception_data_id: str
    feedback_type: str = Field(..., regex=r"^(validation|correction|replanning|status_update)$")
    result: Dict[str, Any] = Field(default_factory=dict)
    timestamp: datetime = Field(default_factory=datetime.now)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    def is_validation_feedback(self) -> bool:
        """Check if this is a validation feedback."""
        return self.feedback_type == "validation"

    def is_correction_feedback(self) -> bool:
        """Check if this is a correction feedback."""
        return self.feedback_type == "correction"

    def is_replanning_feedback(self) -> bool:
        """Check if this is a replanning feedback."""
        return self.feedback_type == "replanning"

    def is_status_update(self) -> bool:
        """Check if this is a status update feedback."""
        return self.feedback_type == "status_update"

    def requires_replanning(self) -> bool:
        """Check if this feedback requires replanning."""
        return self.feedback_type in ["correction", "replanning"]

    def get_action_recommendation(self) -> str:
        """Get the recommended action based on this feedback."""
        if self.feedback_type == "validation":
            if self.result.get("success", False):
                return "continue"
            else:
                return "retry"
        elif self.feedback_type == "correction":
            return "correct"
        elif self.feedback_type == "replanning":
            return "replan"
        else:
            return "continue"

    def update_result(self, new_result: Dict[str, Any]):
        """Update the feedback result."""
        self.result.update(new_result)
        self.timestamp = datetime.now()