from pydantic import BaseModel, Field
from datetime import datetime
from typing import List, Dict, Any, Optional
import uuid
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .action import Action


class ActionSequence(BaseModel):
    """
    Ordered list of ROS 2 commands that execute the planned behavior
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    plan_id: str
    actions: List['Action'] = Field(..., min_items=1)
    status: str = Field(default="pending", regex=r"^(pending|executing|completed|failed|interrupted)$")
    current_action_index: int = Field(ge=0, default=0)
    execution_log: List[Dict[str, Any]] = Field(default_factory=list)  # Log of executed actions with timestamps
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    def update_status(self, new_status: str):
        """Update the sequence status and timestamp."""
        self.status = new_status
        self.updated_at = datetime.now()

    def get_current_action(self) -> Optional['Action']:
        """Get the current action being executed."""
        if 0 <= self.current_action_index < len(self.actions):
            return self.actions[self.current_action_index]
        return None

    def mark_action_completed(self, action_result: Dict[str, Any] = None):
        """Mark the current action as completed and move to the next one."""
        if self.current_action_index < len(self.actions):
            # Log the completed action
            log_entry = {
                "action_id": self.actions[self.current_action_index].id,
                "status": "completed",
                "timestamp": datetime.now().isoformat(),
                "result": action_result
            }
            self.execution_log.append(log_entry)

            # Move to the next action
            self.current_action_index += 1
            self.updated_at = datetime.now()

    def mark_action_failed(self, error: str = None):
        """Mark the current action as failed."""
        if self.current_action_index < len(self.actions):
            # Log the failed action
            log_entry = {
                "action_id": self.actions[self.current_action_index].id,
                "status": "failed",
                "timestamp": datetime.now().isoformat(),
                "error": error
            }
            self.execution_log.append(log_entry)

            self.status = "failed"
            self.updated_at = datetime.now()

    def is_complete(self) -> bool:
        """Check if all actions in the sequence have been executed."""
        return self.current_action_index >= len(self.actions)

    def get_progress(self) -> float:
        """Get the completion progress as a percentage."""
        if not self.actions:
            return 0.0
        return min(1.0, self.current_action_index / len(self.actions))