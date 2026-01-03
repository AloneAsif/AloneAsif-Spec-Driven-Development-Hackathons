# Quickstart: Todo CLI Application

**Feature**: 001-todo-cli-app
**Date**: 2025-12-31

## Prerequisites

- Python 3.13 or higher
- UV package manager

## Setup

1. **Initialize project with UV**

   ```bash
   uv init todo-cli --python 3.13
   cd todo-cli
   ```

2. **Install dependencies** (none required beyond Python standard library)

   ```bash
   uv add pytest --dev  # For testing
   ```

3. **Verify installation**

   ```bash
   python --version  # Should show 3.13+
   uv --version      # Should show UV version
   ```

## Project Structure

```
todo-cli/
├── src/
│   ├── models/
│   │   └── todo.py
│   ├── services/
│   │   └── todo_service.py
│   ├── cli/
│   │   └── menu.py
│   └── __init__.py
├── tests/
│   ├── unit/
│   └── integration/
├── pyproject.toml
└── README.md
```

## Running the Application

```bash
python -m src.cli.menu
```

Or add a script to `pyproject.toml`:

```toml
[project.scripts]
todo = "src.cli.menu:main"
```

Then run:

```bash
todo
```

## Running Tests

```bash
# Run all tests
pytest

# Run with verbose output
pytest -v

# Run specific test file
pytest tests/unit/test_todo_model.py
```

## Usage Workflow

1. **Add a todo**: Select option 1, enter title (and optional description)
2. **View todos**: Select option 2 to see all tasks with status
3. **Update a todo**: Select option 3, provide ID and new values
4. **Delete a todo**: Select option 4, provide ID to remove
5. **Toggle status**: Select option 5, choose to mark complete or incomplete
6. **Exit**: Select option 6 to close the application

## Verification Checklist

After implementation, verify:

- [ ] Can add todo with title only
- [ ] Can add todo with title and description
- [ ] View shows all todos with correct information
- [ ] Empty state displays when no todos exist
- [ ] Can update todo title
- [ ] Can update todo description
- [ ] Invalid ID shows error message
- [ ] Can delete todo
- [ ] Invalid delete ID shows error message
- [ ] Can mark todo complete
- [ ] Can mark todo incomplete
- [ ] All operations complete in under 1 second
- [ ] Application continues after error messages
