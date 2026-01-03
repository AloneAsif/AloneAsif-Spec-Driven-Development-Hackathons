# Todo CLI Application

A feature-rich in-memory todo management application built with Python 3.13+. This is Phase I Intermediate of "The Evolution of Todo" project, demonstrating spec-driven development using Spec-Kit Plus.

## Features

### Core Features (Phase I Basic)
- **Add Todo**: Create tasks with title and optional description
- **View Todos**: Display all tasks with their status
- **Update Todo**: Modify existing task details by ID
- **Delete Todo**: Remove tasks by ID
- **Mark Complete/Incomplete**: Toggle task completion status

### New Features (Phase I Intermediate)
- **Priority Management**: Assign High/Medium/Low priority levels to tasks
- **Tags System**: Categorize tasks with multiple tags
- **Search**: Find tasks by keyword (searches title and description)
- **Filter**: View filtered subsets by status, priority, or tag
- **Sort**: Display tasks sorted by title (A-Z) or priority (High to Low)

## Prerequisites

- Python 3.13 or higher
- UV package manager (recommended) or pip

## Installation

### Option 1: Using UV (Recommended)

```bash
cd todo-cli
uv sync
```

### Option 2: Manual Setup

```bash
cd todo-cli
python -m venv .venv
source .venv/bin/activate  # Linux/macOS
# OR
.venv\Scripts\activate    # Windows
pip install pytest
```

## Running the Application

```bash
# Method 1: Using Python Module
python -m src.cli.menu

# Method 2: Direct execution
python src/cli/menu.py
```

## Usage Guide

### Main Menu

```
=== Todo Manager ===
1. Add todo
2. View todos
3. Update todo
4. Delete todo
5. Mark complete/incomplete
6. Search todos
7. Filter todos
8. Sort todos
9. Exit
```

### Quick Start Examples

**Add a todo with priority and tags:**
```
Choice: 1
Title: Complete documentation
Description: Update README
Priority: H (High/Medium/Low)
Tags: work, urgent
```

**Search todos:**
```
Choice: 6
Keyword: documentation
```

**Filter by priority:**
```
Choice: 7
Filter by: 2 (Priority)
Priority: H
```

**Sort by priority:**
```
Choice: 8
Sort by: 2 (Priority)
```

## Project Structure

```
todo-cli/
├── src/
│   ├── models/
│   │   └── todo.py          # Todo model with Priority enum
│   ├── services/
│   │   └── todo_service.py  # Business logic + search/filter/sort
│   └── cli/
│       └── menu.py          # CLI interface (9 options)
├── tests/
│   ├── unit/
│   └── integration/
├── pyproject.toml
├── .gitignore
└── README.md
```

## Technical Details

### Data Model

```python
class Priority(Enum):
    HIGH = "High"
    MEDIUM = "Medium"
    LOW = "Low"

@dataclass
class Todo:
    title: str
    description: str = ""
    completed: bool = False
    priority: Priority = Priority.MEDIUM
    tags: list[str] = field(default_factory=list)
    id: int = field(default=0, init=False)
```

### New Service Methods

- `add(title, description, priority, tags)` - Create todo with priority/tags
- `update_priority(todo_id, priority)` - Update priority
- `update_tags(todo_id, tags)` - Update tags
- `search(keyword)` - Case-insensitive search
- `filter_by_status/priority/tag()` - Filter todos
- `sort_by_title/priority()` - Sort todos (non-mutating)

## Running Tests

```bash
pytest              # Run all tests
pytest -v           # Verbose output
pytest tests/unit/  # Unit tests only
```

## Limitations

- In-memory storage only (data lost on exit)
- Single user session
- No persistence layer
- Single filter at a time
- Sorting is display-only

## Version History

- **v0.2.0** (Phase I Intermediate) - Priority, tags, search, filter, sort
- **v0.1.0** (Phase I Basic) - Core CRUD operations

## License

Part of "The Evolution of Todo" hackathon project.
