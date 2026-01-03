"""Todo storage and business logic service."""

from typing import Optional

from src.models.todo import Todo, Priority


class TodoStore:
    """In-memory storage for Todo items with auto-incrementing IDs."""

    def __init__(self) -> None:
        """Initialize an empty todo store."""
        self._todos: dict[int, Todo] = {}
        self._next_id: int = 1

    def add(
        self,
        title: str,
        description: str = "",
        priority: Optional[Priority] = None,
        tags: Optional[list[str]] = None,
    ) -> Todo:
        """Add a new todo with auto-assigned ID.

        Args:
            title: Required todo title.
            description: Optional todo description.
            priority: Optional priority level (defaults to MEDIUM if not provided).
            tags: Optional list of tags.

        Returns:
            The created Todo instance.

        Raises:
            ValueError: If title is empty.
        """
        if not title or not title.strip():
            raise ValueError("Title cannot be empty.")

        # Create todo with priority and tags if provided
        if priority is None:
            priority = Priority.MEDIUM
        if tags is None:
            tags = []

        todo = Todo(
            title=title.strip(),
            description=description.strip(),
            priority=priority,
            tags=tags,
        )
        todo.id = self._next_id
        self._todos[self._next_id] = todo
        self._next_id += 1
        return todo

    def get(self, todo_id: int) -> Optional[Todo]:
        """Get a todo by ID.

        Args:
            todo_id: The todo ID to look up.

        Returns:
            The Todo if found, None otherwise.
        """
        return self._todos.get(todo_id)

    def get_all(self) -> list[Todo]:
        """Get all todos sorted by ID.

        Returns:
            List of all todos in ID order.
        """
        return [self._todos[id_] for id_ in sorted(self._todos.keys())]

    def update(
        self,
        todo_id: int,
        title: Optional[str] = None,
        description: Optional[str] = None,
    ) -> Todo:
        """Update a todo's title and/or description.

        Args:
            todo_id: The todo ID to update.
            title: New title (if provided, must be non-empty).
            description: New description (if provided, must be valid).

        Returns:
            The updated Todo.

        Raises:
            ValueError: If todo not found or title is empty.
        """
        todo = self._get_or_raise(todo_id)

        if title is not None:
            if not title.strip():
                raise ValueError("Title cannot be empty.")
            todo.update_title(title.strip())

        if description is not None:
            todo.update_description(description.strip())

        return todo

    def update_priority(self, todo_id: int, priority: Priority) -> Todo:
        """Update a todo's priority.

        Args:
            todo_id: ID of todo to update
            priority: New priority level

        Returns:
            Updated Todo

        Raises:
            ValueError: If todo not found
        """
        todo = self._get_or_raise(todo_id)
        todo.update_priority(priority)
        return todo

    def update_tags(self, todo_id: int, tags: list[str]) -> Todo:
        """Replace all tags on a todo.

        Args:
            todo_id: ID of todo to update
            tags: New list of tags

        Returns:
            Updated Todo

        Raises:
            ValueError: If todo not found or any tag is invalid
        """
        todo = self._get_or_raise(todo_id)
        todo.set_tags(tags)
        return todo

    def delete(self, todo_id: int) -> None:
        """Delete a todo by ID.

        Args:
            todo_id: The todo ID to delete.

        Raises:
            ValueError: If todo not found.
        """
        if todo_id not in self._todos:
            raise ValueError(f"Todo with ID {todo_id} not found.")
        del self._todos[todo_id]

    def toggle_complete(self, todo_id: int) -> Todo:
        """Toggle a todo's completion status.

        Args:
            todo_id: The todo ID to toggle.

        Returns:
            The updated Todo.

        Raises:
            ValueError: If todo not found.
        """
        todo = self._get_or_raise(todo_id)
        todo.toggle_completed()
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
            if (
                keyword_lower in todo.title.lower()
                or keyword_lower in todo.description.lower()
            ):
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

    def count(self) -> int:
        """Get the total number of todos.

        Returns:
            Number of todos in the store.
        """
        return len(self._todos)

    def _get_or_raise(self, todo_id: int) -> Todo:
        """Get a todo or raise ValueError if not found.

        Args:
            todo_id: The todo ID to look up.

        Returns:
            The Todo instance.

        Raises:
            ValueError: If todo not found.
        """
        todo = self._todos.get(todo_id)
        if todo is None:
            raise ValueError(f"Todo with ID {todo_id} not found.")
        return todo
