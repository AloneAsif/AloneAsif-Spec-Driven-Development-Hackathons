from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
import uuid


class Task(BaseModel):
    """
    Individual unit of work within a task plan
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    type: str = Field(..., regex=r"^(navigation|manipulation|perception|other)$")
    description: str = Field(..., min_length=1)
    parameters: Dict[str, Any] = Field(default_factory=dict)
    dependencies: List[str] = Field(default_factory=list)  # List of task IDs that must complete first
    timeout: int = Field(gt=0, default=30)  # Maximum time to complete in seconds
    status: str = Field(default="pending", regex=r"^(pending|executing|completed|failed)$")

    def mark_as_executing(self):
        """Mark the task as currently executing."""
        self.status = "executing"

    def mark_as_completed(self):
        """Mark the task as completed."""
        self.status = "completed"

    def mark_as_failed(self):
        """Mark the task as failed."""
        self.status = "failed"

    def is_ready_to_execute(self, completed_tasks: List[str]) -> bool:
        """Check if all dependencies are met."""
        return all(dep_id in completed_tasks for dep_id in self.dependencies)

    def can_execute_in_parallel(self, other_task: 'Task') -> bool:
        """Check if this task can execute in parallel with another task."""
        # Tasks can run in parallel if they don't have conflicting dependencies
        # and don't require the same exclusive resources
        return not (set(self.dependencies) & set(other_task.dependencies))