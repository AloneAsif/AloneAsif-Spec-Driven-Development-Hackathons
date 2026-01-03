"""Integration tests for CLI workflow."""

import pytest
from io import StringIO
from unittest.mock import patch

from src.cli.menu import main
from src.services.todo_service import TodoStore


class TestCLIWorkflow:
    """Integration tests for the complete CLI workflow."""

    def test_full_add_and_view_workflow(self):
        """Test adding todos and viewing them."""
        store = TodoStore()

        # Add a todo
        todo = store.add("Buy groceries", "Milk, eggs, bread")
        assert todo.id == 1
        assert todo.title == "Buy groceries"

        # Add another todo
        todo2 = store.add("Call mom")
        assert todo2.id == 2

        # View todos
        todos = store.get_all()
        assert len(todos) == 2
        assert todos[0].title == "Buy groceries"
        assert todos[1].title == "Call mom"

    def test_update_workflow(self):
        """Test updating a todo."""
        store = TodoStore()
        todo = store.add("Original title")

        # Update the todo
        updated = store.update(todo.id, title="New title", description="New desc")
        assert updated.title == "New title"
        assert updated.description == "New desc"

    def test_delete_workflow(self):
        """Test deleting a todo."""
        store = TodoStore()
        todo1 = store.add("Keep this")
        todo2 = store.add("Delete this")

        # Delete one
        store.delete(todo2.id)

        # Verify only one remains
        todos = store.get_all()
        assert len(todos) == 1
        assert todos[0].title == "Keep this"

    def test_toggle_completion_workflow(self):
        """Test toggling completion status."""
        store = TodoStore()
        todo = store.add("Task to complete")

        assert todo.completed is False

        # Toggle to complete
        store.toggle_complete(todo.id)
        assert todo.completed is True

        # Toggle back to incomplete
        store.toggle_complete(todo.id)
        assert todo.completed is False

    def test_error_handling_invalid_id(self):
        """Test error handling for invalid IDs."""
        store = TodoStore()

        # Try to get non-existent todo
        assert store.get(999) is None

        # Try to update non-existent todo
        with pytest.raises(ValueError, match="Todo with ID 999 not found"):
            store.update(999, title="New")

        # Try to delete non-existent todo
        with pytest.raises(ValueError, match="Todo with ID 999 not found"):
            store.delete(999)

        # Try to toggle non-existent todo
        with pytest.raises(ValueError, match="Todo with ID 999 not found"):
            store.toggle_complete(999)

    def test_error_handling_empty_title(self):
        """Test error handling for empty title."""
        store = TodoStore()

        with pytest.raises(ValueError, match="Title cannot be empty"):
            store.add("")

    def test_empty_view_displays_message(self):
        """Test that empty view displays appropriate message."""
        store = TodoStore()
        todos = store.get_all()
        assert todos == []

    def test_all_operations_in_sequence(self):
        """Test complete workflow: add, view, update, toggle, delete."""
        store = TodoStore()

        # Add
        todo = store.add("Test task", "Test description")
        todo_id = todo.id
        assert todo_id == 1

        # View
        todos = store.get_all()
        assert len(todos) == 1

        # Update
        store.update(todo_id, title="Updated task")
        assert store.get(todo_id).title == "Updated task"

        # Toggle complete
        store.toggle_complete(todo_id)
        assert store.get(todo_id).completed is True

        # Delete
        store.delete(todo_id)

        # Verify empty
        assert store.count() == 0
