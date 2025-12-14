from pydantic import BaseModel, Field
from datetime import datetime
from typing import List, Optional
import uuid
from .task import Task


class TaskPlan(BaseModel):
    """
    Structured sequence of sub-goals and actions generated from natural language commands
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    command_id: str
    tasks: List[Task] = Field(..., min_items=1)
    status: str = Field(default="pending", regex=r"^(pending|executing|completed|failed|cancelled)$")
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    estimated_duration: int = Field(gt=0, default=30)  # in seconds

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }

    def update_status(self, new_status: str):
        """Update the plan status and timestamp."""
        self.status = new_status
        self.updated_at = datetime.now()

    def add_task(self, task: Task):
        """Add a task to the plan."""
        self.tasks.append(task)
        self.updated_at = datetime.now()

    def get_next_task(self) -> Optional[Task]:
        """Get the next task to execute based on dependencies."""
        for task in self.tasks:
            if task.status == "pending":
                # Check if all dependencies are completed
                all_deps_met = True
                for dep_id in task.dependencies:
                    dep_task = next((t for t in self.tasks if t.id == dep_id), None)
                    if dep_task and dep_task.status != "completed":
                        all_deps_met = False
                        break

                if all_deps_met:
                    return task
        return None

    def is_complete(self) -> bool:
        """Check if all tasks in the plan are completed."""
        return all(task.status == "completed" for task in self.tasks)

    def is_failed(self) -> bool:
        """Check if any task in the plan has failed."""
        return any(task.status == "failed" for task in self.tasks)