"""Todo data model with validation."""

from dataclasses import dataclass, field
from enum import Enum


class Priority(Enum):
    """Priority levels for todo items."""
    HIGH = "High"
    MEDIUM = "Medium"
    LOW = "Low"


@dataclass
class Todo:
    """Represents a single task item in the todo list."""

    title: str
    description: str = ""
    completed: bool = False
    priority: Priority = Priority.MEDIUM
    tags: list[str] = field(default_factory=list)
    id: int = field(default=0, init=False)

    TITLE_MIN_LENGTH = 1
    TITLE_MAX_LENGTH = 200
    DESCRIPTION_MAX_LENGTH = 1000

    def __post_init__(self) -> None:
        """Validate and normalize fields after initialization."""
        self._validate_title()
        self._validate_description()

    def _validate_title(self) -> None:
        """Validate title length."""
        stripped = self.title.strip()
        if not self.TITLE_MIN_LENGTH <= len(stripped) <= self.TITLE_MAX_LENGTH:
            raise ValueError(
                f"Title must be between {self.TITLE_MIN_LENGTH} and "
                f"{self.TITLE_MAX_LENGTH} characters."
            )

    def _validate_description(self) -> None:
        """Validate description length if provided."""
        if self.description and len(self.description) > self.DESCRIPTION_MAX_LENGTH:
            raise ValueError(
                f"Description must not exceed {self.DESCRIPTION_MAX_LENGTH} characters."
            )

    def update_title(self, new_title: str) -> None:
        """Update the title with validation."""
        self.title = new_title
        self._validate_title()

    def update_description(self, new_description: str) -> None:
        """Update the description with validation."""
        self.description = new_description
        self._validate_description()

    def toggle_completed(self) -> None:
        """Toggle the completion status."""
        self.completed = not self.completed

    def update_priority(self, new_priority: Priority) -> None:
        """Update the priority level.

        Args:
            new_priority: Must be a valid Priority enum value

        Raises:
            TypeError: If new_priority is not a Priority enum
        """
        if not isinstance(new_priority, Priority):
            raise TypeError("Priority must be a Priority enum value")
        self.priority = new_priority

    def add_tag(self, tag: str) -> None:
        """Add a tag if not already present.

        Args:
            tag: Non-empty string tag

        Raises:
            ValueError: If tag is empty after stripping
        """
        tag = tag.strip()
        if not tag:
            raise ValueError("Tag cannot be empty")
        if tag not in self.tags:
            self.tags.append(tag)

    def remove_tag(self, tag: str) -> None:
        """Remove a tag if present (case-sensitive).

        Args:
            tag: Tag to remove
        """
        if tag in self.tags:
            self.tags.remove(tag)

    def set_tags(self, tags: list[str]) -> None:
        """Replace all tags with new list.

        Args:
            tags: List of tags (duplicates will be removed)

        Raises:
            ValueError: If any tag is empty after stripping
        """
        cleaned = []
        for tag in tags:
            tag_stripped = tag.strip()
            if not tag_stripped:
                raise ValueError("Tags cannot be empty")
            if tag_stripped not in cleaned:
                cleaned.append(tag_stripped)
        self.tags = cleaned

    def has_tag(self, tag: str, case_sensitive: bool = False) -> bool:
        """Check if todo has a specific tag.

        Args:
            tag: Tag to search for
            case_sensitive: If False, perform case-insensitive match

        Returns:
            True if tag is present, False otherwise
        """
        if case_sensitive:
            return tag in self.tags
        else:
            tag_lower = tag.lower()
            return any(t.lower() == tag_lower for t in self.tags)

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            "id": self.id,
            "title": self.title,
            "description": self.description,
            "completed": self.completed,
            "priority": self.priority.value,
            "tags": self.tags.copy(),
        }
