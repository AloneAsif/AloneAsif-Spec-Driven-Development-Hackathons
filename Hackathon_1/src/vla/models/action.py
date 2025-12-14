from pydantic import BaseModel, Field
from typing import Dict, Any
import uuid


class Action(BaseModel):
    """
    Individual ROS 2 action to be executed
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    action_type: str = Field(..., regex=r"^(navigation|manipulation|service_call|topic_publish)$")
    parameters: Dict[str, Any] = Field(default_factory=dict)
    timeout: int = Field(gt=0, default=30)  # Maximum time to complete in seconds
    success_criteria: Dict[str, Any] = Field(default_factory=dict)  # Conditions that define successful completion
    status: str = Field(default="pending", regex=r"^(pending|executing|completed|failed)$")

    def mark_as_executing(self):
        """Mark the action as currently executing."""
        self.status = "executing"

    def mark_as_completed(self):
        """Mark the action as completed."""
        self.status = "completed"

    def mark_as_failed(self):
        """Mark the action as failed."""
        self.status = "failed"

    def is_successful(self, result: Dict[str, Any]) -> bool:
        """Check if the action result meets the success criteria."""
        if not self.success_criteria:
            return True  # If no criteria specified, consider successful

        # Simple implementation - can be extended based on specific criteria
        for key, expected_value in self.success_criteria.items():
            actual_value = result.get(key)
            if actual_value != expected_value:
                return False
        return True

    def can_execute_in_parallel(self, other_action: 'Action') -> bool:
        """Check if this action can execute in parallel with another action."""
        # For now, assume actions that don't use the same robot resources can run in parallel
        # This could be enhanced based on specific robot capabilities
        return True