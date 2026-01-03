# Data Model: Todo CLI Intermediate Extensions

**Feature**: 002-todo-cli-intermediate
**Date**: 2025-12-31
**Status**: Design Complete

## Overview

This document defines the extensions to the existing Todo entity and TodoStore service to support priority, tags, search, filter, and sort operations. All changes are backward compatible with Phase I Basic implementation.

## Entity Extensions

### Priority Enum

**Purpose**: Type-safe representation of task priority levels

```python
from enum import Enum

class Priority(Enum):
    """Priority levels for todo items."""
    HIGH = "High"
    MEDIUM = "Medium"
    LOW = "Low"
```

**Properties**:
- Three distinct values: HIGH, MEDIUM, LOW
- String values for human-readable display
- Ordered for sorting: HIGH > MEDIUM > LOW

### Todo Model Extensions

**Existing Model** (from 001-todo-cli-app):
```python
@dataclass
class Todo:
    title: str
    description: str = ""
    completed: bool = False
    id: int = field(default=0, init=False)
```

**Extended Model** (002-todo-cli-intermediate):
```python
from enum import Enum
from dataclasses import dataclass, field

class Priority(Enum):
    HIGH = "High"
    MEDIUM = "Medium"
    LOW = "Low"

@dataclass
class Todo:
    title: str
    description: str = ""
    completed: bool = False
    priority: Priority = Priority.MEDIUM  # NEW: Default Medium
    tags: list[str] = field(default_factory=list)  # NEW: Empty list default
    id: int = field(default=0, init=False)
```

**New Fields**:

| Field | Type | Default | Description | Validation |
|-------|------|---------|-------------|------------|
| `priority` | `Priority` | `Priority.MEDIUM` | Task priority level | Must be valid Priority enum value |
| `tags` | `list[str]` | `[]` (empty list) | Category labels | Each tag must be non-empty string after stripping |

**Backward Compatibility**:
- Default values ensure existing code works without modification
- Existing fields unchanged
- Existing methods (toggle_completed, update_title, update_description) unchanged

**New Methods**:

```python
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
    """Convert to dictionary representation (extended).

    Returns:
        Dictionary with all fields including priority and tags
    """
    return {
        "id": self.id,
        "title": self.title,
        "description": self.description,
        "completed": self.completed,
        "priority": self.priority.value,  # NEW: String value of enum
        "tags": self.tags.copy(),  # NEW: Copy to prevent mutation
    }
```

## Service Extensions

### TodoStore Extensions

**Existing Methods** (unchanged):
- `add(title: str, description: str = "") -> Todo`
- `get(todo_id: int) -> Optional[Todo]`
- `get_all() -> list[Todo]`
- `update(todo_id: int, title: str = None, description: str = None) -> Optional[Todo]`
- `delete(todo_id: int) -> bool`
- `toggle_completed(todo_id: int) -> Optional[Todo]`

**New Methods**:

```python
def update_priority(self, todo_id: int, priority: Priority) -> Optional[Todo]:
    """Update a todo's priority.

    Args:
        todo_id: ID of todo to update
        priority: New priority level

    Returns:
        Updated Todo if found, None otherwise
    """
    todo = self.get(todo_id)
    if todo:
        todo.update_priority(priority)
    return todo

def update_tags(self, todo_id: int, tags: list[str]) -> Optional[Todo]:
    """Replace all tags on a todo.

    Args:
        todo_id: ID of todo to update
        tags: New list of tags

    Returns:
        Updated Todo if found, None otherwise

    Raises:
        ValueError: If any tag is invalid
    """
    todo = self.get(todo_id)
    if todo:
        todo.set_tags(tags)
    return todo

def search(self, keyword: str) -> list[Todo]:
    """Search todos by keyword (case-insensitive).

    Matches against title OR description.

    Args:
        keyword: Search term (case-insensitive)

    Returns:
        List of matching todos, sorted by ID
    """
    if not keyword:
        return self.get_all()

    keyword_lower = keyword.lower()
    matches = []
    for todo in self._todos.values():
        if (keyword_lower in todo.title.lower() or
            keyword_lower in todo.description.lower()):
            matches.append(todo)

    return sorted(matches, key=lambda t: t.id)

def filter_by_status(self, completed: bool) -> list[Todo]:
    """Filter todos by completion status.

    Args:
        completed: True for complete, False for incomplete

    Returns:
        List of matching todos, sorted by ID
    """
    matches = [t for t in self._todos.values() if t.completed == completed]
    return sorted(matches, key=lambda t: t.id)

def filter_by_priority(self, priority: Priority) -> list[Todo]:
    """Filter todos by priority level.

    Args:
        priority: Priority level to match

    Returns:
        List of matching todos, sorted by ID
    """
    matches = [t for t in self._todos.values() if t.priority == priority]
    return sorted(matches, key=lambda t: t.id)

def filter_by_tag(self, tag: str) -> list[Todo]:
    """Filter todos by tag (case-insensitive).

    Args:
        tag: Tag to match

    Returns:
        List of matching todos, sorted by ID
    """
    matches = [t for t in self._todos.values() if t.has_tag(tag, case_sensitive=False)]
    return sorted(matches, key=lambda t: t.id)

def sort_by_title(self) -> list[Todo]:
    """Get all todos sorted by title (case-insensitive A-Z).

    Returns:
        List of todos sorted alphabetically by title
    """
    return sorted(self._todos.values(), key=lambda t: t.title.lower())

def sort_by_priority(self) -> list[Todo]:
    """Get all todos sorted by priority (High -> Medium -> Low).

    Returns:
        List of todos sorted by priority level
    """
    priority_order = {
        Priority.HIGH: 0,
        Priority.MEDIUM: 1,
        Priority.LOW: 2,
    }
    return sorted(self._todos.values(), key=lambda t: priority_order[t.priority])
```

## Data Relationships

### Todo → Priority
- **Cardinality**: Many-to-One (many todos can have the same priority)
- **Constraint**: Priority must be one of three enum values
- **Default**: MEDIUM

### Todo → Tags
- **Cardinality**: Many-to-Many (todo has many tags, tag appears on many todos)
- **Constraint**: Tags are strings, case-sensitive storage, case-insensitive matching
- **Default**: Empty list

## Validation Rules

### Priority
- **Input**: Must be a Priority enum value or parseable string ("high", "medium", "low" - case-insensitive)
- **Storage**: Always stored as Priority enum
- **Display**: Use `.value` property for string representation

### Tags
- **Input**: Comma-separated string or list of strings
- **Storage**: List of non-empty strings after stripping whitespace
- **Validation**: Each tag must have length >= 1 after stripping
- **Duplicates**: Automatically removed during set_tags()
- **Case**: Stored as provided, matched case-insensitively for filtering

## State Transitions

### Priority Transitions
```
Any Priority → Any Priority (unrestricted)
```

### Tag Transitions
```
[] → [tag1] (add first tag)
[tag1] → [tag1, tag2] (add tag)
[tag1, tag2] → [tag1] (remove tag)
[tag1, tag2] → [tag3, tag4] (replace all)
```

## Performance Considerations

### Search
- **Complexity**: O(n * m) where n = number of todos, m = average text length
- **Acceptable Range**: <1000 todos
- **Optimization**: Early return on empty keyword

### Filter
- **Complexity**: O(n) for status and priority filters
- **Complexity**: O(n * t) for tag filter where t = average tags per todo
- **Acceptable Range**: <1000 todos

### Sort
- **Complexity**: O(n log n) for both title and priority sorts
- **Memory**: Creates new sorted list, does not mutate storage

## Testing Strategy

### Unit Tests (models/todo.py)
- Priority validation (valid enum, invalid type)
- Tag operations (add, remove, set, has_tag)
- Case-insensitive tag matching
- to_dict() includes new fields

### Unit Tests (services/todo_service.py)
- update_priority (valid, invalid ID)
- update_tags (valid, invalid tags, empty strings)
- search (case-insensitive, title match, description match, no match)
- filter_by_status (complete, incomplete, empty list)
- filter_by_priority (each level, empty list)
- filter_by_tag (case-insensitive, no match)
- sort_by_title (alphabetical, case-insensitive)
- sort_by_priority (High-Medium-Low order)

### Integration Tests
- Create todo with priority and tags
- Update existing todo with new priority and tags
- Search and display results
- Filter and display results
- Sort and display results
- Verify no mutations to storage order

## Backward Compatibility Verification

### Regression Tests
- [ ] All Phase I Basic tests pass without modification
- [ ] Creating todo without priority uses MEDIUM default
- [ ] Creating todo without tags uses empty list
- [ ] Existing methods (add, update, delete, toggle_completed) work unchanged
- [ ] to_dict() still includes all original fields

## Migration Notes

**From Phase I Basic to Intermediate**:
- No data migration needed (in-memory only)
- Existing Todo instances will need priority and tags fields added when loading from persistent storage in future phases
- Default values ensure seamless upgrade path
