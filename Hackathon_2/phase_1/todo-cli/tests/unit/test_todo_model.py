"""Unit tests for Todo model."""

import pytest

from src.models.todo import Todo


class TestTodoModel:
    """Test cases for Todo dataclass."""

    def test_create_todo_with_title_only(self):
        """Test creating a todo with only a title."""
        todo = Todo(title="Buy groceries")
        assert todo.title == "Buy groceries"
        assert todo.description == ""
        assert todo.completed is False
        assert todo.id == 0  # ID set externally

    def test_create_todo_with_title_and_description(self):
        """Test creating a todo with title and description."""
        todo = Todo(
            title="Buy groceries",
            description="Milk, eggs, bread, butter",
        )
        assert todo.title == "Buy groceries"
        assert todo.description == "Milk, eggs, bread, butter"
        assert todo.completed is False

    def test_create_todo_with_completed_status(self):
        """Test creating a completed todo."""
        todo = Todo(title="Task", completed=True)
        assert todo.completed is True

    def test_todo_title_validation_empty_title(self):
        """Test that empty title raises ValueError."""
        with pytest.raises(ValueError, match="Title must be between"):
            Todo(title="")

    def test_todo_title_validation_whitespace_only(self):
        """Test that whitespace-only title raises ValueError."""
        with pytest.raises(ValueError, match="Title must be between"):
            Todo(title="   ")

    def test_todo_title_validation_max_length(self):
        """Test that title exceeding max length raises ValueError."""
        long_title = "x" * 201
        with pytest.raises(ValueError, match="Title must be between"):
            Todo(title=long_title)

    def test_todo_title_validation_valid_max_length(self):
        """Test that title at max length is valid."""
        long_title = "x" * 200
        todo = Todo(title=long_title)
        assert len(todo.title) == 200

    def test_todo_description_validation_max_length(self):
        """Test that description exceeding max length raises ValueError."""
        long_desc = "x" * 1001
        with pytest.raises(ValueError, match="Description must not exceed"):
            Todo(title="Task", description=long_desc)

    def test_update_title(self):
        """Test updating title."""
        todo = Todo(title="Original")
        todo.update_title("Updated")
        assert todo.title == "Updated"

    def test_update_description(self):
        """Test updating description."""
        todo = Todo(title="Task", description="Original")
        todo.update_description("Updated")
        assert todo.description == "Updated"

    def test_toggle_completed(self):
        """Test toggling completion status."""
        todo = Todo(title="Task", completed=False)
        assert todo.completed is False
        todo.toggle_completed()
        assert todo.completed is True
        todo.toggle_completed()
        assert todo.completed is False

    def test_to_dict(self):
        """Test converting todo to dictionary."""
        todo = Todo(title="Task", description="Desc", completed=True)
        todo.id = 1  # ID is set externally
        result = todo.to_dict()
        assert result == {
            "id": 1,
            "title": "Task",
            "description": "Desc",
            "completed": True,
            "priority": "Medium",
            "tags": [],
        }
