"""Unit tests for TodoStore service."""

import pytest

from src.services.todo_service import TodoStore


class TestTodoStore:
    """Test cases for TodoStore class."""

    def setup_method(self):
        """Create a fresh store for each test."""
        self.store = TodoStore()

    def test_add_todo_with_title_only(self):
        """Test adding a todo with only a title."""
        todo = self.store.add("Buy groceries")
        assert todo.id == 1
        assert todo.title == "Buy groceries"
        assert todo.description == ""
        assert todo.completed is False

    def test_add_todo_with_title_and_description(self):
        """Test adding a todo with title and description."""
        todo = self.store.add(
            title="Buy groceries",
            description="Milk, eggs, bread",
        )
        assert todo.id == 1
        assert todo.title == "Buy groceries"
        assert todo.description == "Milk, eggs, bread"

    def test_add_multiple_todos_increments_ids(self):
        """Test that IDs increment correctly."""
        todo1 = self.store.add("Task 1")
        todo2 = self.store.add("Task 2")
        todo3 = self.store.add("Task 3")
        assert todo1.id == 1
        assert todo2.id == 2
        assert todo3.id == 3

    def test_add_todo_strips_whitespace(self):
        """Test that whitespace is stripped from inputs."""
        todo = self.store.add("  Task  ", "  Description  ")
        assert todo.title == "Task"
        assert todo.description == "Description"

    def test_add_todo_empty_title_raises_error(self):
        """Test that empty title raises ValueError."""
        with pytest.raises(ValueError, match="Title cannot be empty"):
            self.store.add("")

    def test_add_todo_whitespace_only_title_raises_error(self):
        """Test that whitespace-only title raises ValueError."""
        with pytest.raises(ValueError, match="Title cannot be empty"):
            self.store.add("   ")

    def test_get_existing_todo(self):
        """Test getting an existing todo."""
        added = self.store.add("Task")
        retrieved = self.store.get(added.id)
        assert retrieved is added
        assert retrieved.title == "Task"

    def test_get_nonexistent_todo(self):
        """Test getting a non-existent todo returns None."""
        result = self.store.get(999)
        assert result is None

    def test_get_all_returns_sorted_todos(self):
        """Test that get_all returns todos sorted by ID."""
        self.store.add("Third")
        self.store.add("First")
        self.store.add("Second")
        all_todos = self.store.get_all()
        assert len(all_todos) == 3
        assert all_todos[0].id == 1
        assert all_todos[1].id == 2
        assert all_todos[2].id == 3
        assert all_todos[0].title == "Third"
        assert all_todos[1].title == "First"
        assert all_todos[2].title == "Second"

    def test_get_all_empty_store(self):
        """Test get_all on empty store."""
        todos = self.store.get_all()
        assert todos == []

    def test_update_title_only(self):
        """Test updating just the title."""
        todo = self.store.add("Original")
        updated = self.store.update(todo.id, title="Updated")
        assert updated.title == "Updated"
        assert updated.description == ""

    def test_update_description_only(self):
        """Test updating just the description."""
        todo = self.store.add("Task", "Original")
        updated = self.store.update(todo.id, description="Updated")
        assert updated.title == "Task"
        assert updated.description == "Updated"

    def test_update_both_title_and_description(self):
        """Test updating both title and description."""
        todo = self.store.add("Original", "Original desc")
        updated = self.store.update(
            todo.id,
            title="New title",
            description="New desc",
        )
        assert updated.title == "New title"
        assert updated.description == "New desc"

    def test_update_partial_empty_title_raises_error(self):
        """Test that empty title during partial update raises error."""
        todo = self.store.add("Task")
        with pytest.raises(ValueError, match="Title cannot be empty"):
            self.store.update(todo.id, title="")

    def test_update_nonexistent_todo_raises_error(self):
        """Test updating non-existent todo raises ValueError."""
        with pytest.raises(ValueError, match="Todo with ID 999 not found"):
            self.store.update(999, title="New")

    def test_delete_existing_todo(self):
        """Test deleting an existing todo."""
        todo = self.store.add("Task")
        self.store.delete(todo.id)
        assert self.store.get(todo.id) is None
        assert self.store.count() == 0

    def test_delete_nonexistent_todo_raises_error(self):
        """Test deleting non-existent todo raises ValueError."""
        with pytest.raises(ValueError, match="Todo with ID 999 not found"):
            self.store.delete(999)

    def test_toggle_complete_incomplete_to_complete(self):
        """Test toggling incomplete to complete."""
        todo = self.store.add("Task")
        assert todo.completed is False
        toggled = self.store.toggle_complete(todo.id)
        assert toggled.completed is True

    def test_toggle_complete_complete_to_incomplete(self):
        """Test toggling complete to incomplete."""
        todo = self.store.add("Task")
        todo.completed = True  # Manually set to complete first
        toggled = self.store.toggle_complete(todo.id)
        assert toggled.completed is False

    def test_toggle_complete_nonexistent_raises_error(self):
        """Test toggling non-existent todo raises ValueError."""
        with pytest.raises(ValueError, match="Todo with ID 999 not found"):
            self.store.toggle_complete(999)

    def test_count_empty_store(self):
        """Test count on empty store."""
        assert self.store.count() == 0

    def test_count_with_todos(self):
        """Test count with multiple todos."""
        self.store.add("Task 1")
        self.store.add("Task 2")
        self.store.add("Task 3")
        assert self.store.count() == 3

    def test_count_after_delete(self):
        """Test count after deleting a todo."""
        self.store.add("Task 1")
        self.store.add("Task 2")
        self.store.delete(1)
        assert self.store.count() == 1
