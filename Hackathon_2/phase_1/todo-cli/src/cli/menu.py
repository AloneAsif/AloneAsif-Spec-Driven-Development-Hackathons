"""CLI menu interface for Todo application."""

from src.services.todo_service import TodoStore
from src.models.todo import Priority


def get_input(prompt: str) -> str:
    """Get user input with a prompt."""
    return input(prompt).strip()


def parse_priority(input_str: str) -> Priority:
    """Parse priority from user input."""
    input_lower = input_str.lower().strip()
    if input_lower in ("h", "high"):
        return Priority.HIGH
    elif input_lower in ("m", "medium"):
        return Priority.MEDIUM
    elif input_lower in ("l", "low"):
        return Priority.LOW
    else:
        raise ValueError(f"Invalid priority. Use H/M/L or High/Medium/Low.")


def parse_tags(input_str: str) -> list[str]:
    """Parse tags from comma-separated input."""
    if not input_str:
        return []
    tags = []
    for tag in input_str.split(","):
        tag_stripped = tag.strip()
        if tag_stripped and tag_stripped not in tags:
            tags.append(tag_stripped)
    return tags


def priority_indicator(priority: Priority) -> str:
    """Get visual indicator for priority level."""
    if priority == Priority.HIGH:
        return "HIGH"
    elif priority == Priority.MEDIUM:
        return "MED"
    else:
        return "LOW"


def add_todo(store: TodoStore) -> None:
    """Handle the Add Todo menu option."""
    title = get_input("Enter title: ")
    if not title:
        print("Error: Title cannot be empty.")
        return
    description = get_input("Enter description (optional): ")
    priority_input = get_input("Enter priority (H/M/L, default: Medium): ")
    if not priority_input:
        priority = Priority.MEDIUM
    else:
        try:
            priority = parse_priority(priority_input)
        except ValueError as e:
            print(f"Error: {e}")
            return
    tags_input = get_input("Enter tags (comma-separated, optional): ")
    tags = parse_tags(tags_input)
    try:
        todo = store.add(title, description, priority, tags)
        print(f"Todo added successfully with ID {todo.id}.")
    except ValueError as e:
        print(f"Error: {e}")


def view_todos(store: TodoStore) -> None:
    """Handle the View Todos menu option."""
    todos = store.get_all()
    if not todos:
        print("No todos yet. Add your first todo!")
        return
    print("\n=== Your Todos ===")
    print(f"{'ID':<3} | {'Status':<12} | {'Priority':<8} | {'Title'}")
    print("-" * 60)
    for todo in todos:
        status = "[x] Complete" if todo.completed else "[ ] Incomplete"
        priority_str = priority_indicator(todo.priority)
        print(f"{todo.id:<3} | {status:<12} | {priority_str:<8} | {todo.title}")
        if todo.description:
            print(f"     Description: {todo.description}")
        if todo.tags:
            tags_str = ", ".join(todo.tags)
            print(f"     Tags: {tags_str}")
    print(f"\n{len(todos)} todo(s) displayed.")


def update_todo(store: TodoStore) -> None:
    """Handle the Update Todo menu option."""
    try:
        todo_id = int(get_input("Enter todo ID: "))
    except ValueError:
        print("Error: Please enter a valid number.")
        return
    print("\nWhat would you like to update?")
    print("1. Title and/or Description")
    print("2. Priority")
    print("3. Tags")
    print("4. Cancel")
    choice = get_input("Enter choice: ")
    try:
        if choice == "1":
            new_title = get_input("Enter new title (leave empty to keep current): ")
            new_desc = get_input("Enter new description (leave empty to keep current): ")
            store.update(
                todo_id,
                title=new_title if new_title else None,
                description=new_desc if new_desc else None,
            )
            print(f"Todo {todo_id} updated successfully.")
        elif choice == "2":
            priority_input = get_input("Enter new priority (H/M/L): ")
            try:
                priority = parse_priority(priority_input)
                store.update_priority(todo_id, priority)
                print(f"Todo {todo_id} priority updated to {priority.value}.")
            except ValueError as e:
                print(f"Error: {e}")
        elif choice == "3":
            print("\nTag operations:")
            print("1. Add tags")
            print("2. Remove tags")
            print("3. Replace all tags")
            tag_choice = get_input("Enter choice: ")
            todo = store.get(todo_id)
            if todo is None:
                print(f"Error: Todo with ID {todo_id} not found.")
                return
            if tag_choice == "1":
                tags_input = get_input("Enter tags to add (comma-separated): ")
                new_tags = parse_tags(tags_input)
                for tag in new_tags:
                    try:
                        todo.add_tag(tag)
                    except ValueError as e:
                        print(f"Warning: {e}")
                print(f"Tags added to todo {todo_id}.")
            elif tag_choice == "2":
                tags_input = get_input("Enter tags to remove (comma-separated): ")
                remove_tags = parse_tags(tags_input)
                for tag in remove_tags:
                    todo.remove_tag(tag)
                print(f"Tags removed from todo {todo_id}.")
            elif tag_choice == "3":
                tags_input = get_input("Enter new tags (comma-separated, replaces all): ")
                new_tags = parse_tags(tags_input)
                try:
                    store.update_tags(todo_id, new_tags)
                    print(f"Tags replaced on todo {todo_id}.")
                except ValueError as e:
                    print(f"Error: {e}")
            else:
                print("Error: Invalid tag operation choice.")
        elif choice == "4":
            print("Update canceled.")
        else:
            print("Error: Invalid choice.")
    except ValueError as e:
        print(f"Error: {e}")


def delete_todo(store: TodoStore) -> None:
    """Handle the Delete Todo menu option."""
    try:
        todo_id = int(get_input("Enter todo ID: "))
    except ValueError:
        print("Error: Please enter a valid number.")
        return
    try:
        store.delete(todo_id)
        print(f"Todo {todo_id} deleted successfully.")
    except ValueError as e:
        print(f"Error: {e}")


def toggle_todo(store: TodoStore) -> None:
    """Handle the Toggle Complete/Incomplete menu option."""
    try:
        todo_id = int(get_input("Enter todo ID: "))
    except ValueError:
        print("Error: Please enter a valid number.")
        return
    print("Choose status:")
    print("1. Mark complete")
    print("2. Mark incomplete")
    choice = get_input("Enter choice: ")
    try:
        todo = store.get(todo_id)
        if todo is None:
            print(f"Error: Todo with ID {todo_id} not found.")
            return
        if choice == "1":
            todo.completed = True
            print(f"Todo {todo_id} marked as complete.")
        elif choice == "2":
            todo.completed = False
            print(f"Todo {todo_id} marked as incomplete.")
        else:
            print("Error: Invalid choice.")
    except ValueError as e:
        print(f"Error: {e}")


def search_todos(store: TodoStore) -> None:
    """Handle the Search Todos menu option."""
    keyword = get_input("Enter search keyword (searches title and description): ")
    if not keyword:
        print("Showing all todos (empty keyword)...")
        keyword = ""
    results = store.search(keyword)
    if not results:
        print("No results found.")
        return
    print(f"\n=== Search Results ({len(results)} found) ===")
    print(f"{'ID':<3} | {'Status':<12} | {'Priority':<8} | {'Title'}")
    print("-" * 60)
    for todo in results:
        status = "[x] Complete" if todo.completed else "[ ] Incomplete"
        priority_str = priority_indicator(todo.priority)
        print(f"{todo.id:<3} | {status:<12} | {priority_str:<8} | {todo.title}")
        if todo.description:
            print(f"     Description: {todo.description}")
        if todo.tags:
            tags_str = ", ".join(todo.tags)
            print(f"     Tags: {tags_str}")


def filter_todos(store: TodoStore) -> None:
    """Handle the Filter Todos menu option."""
    print("\nFilter by:")
    print("1. Status (complete/incomplete)")
    print("2. Priority")
    print("3. Tag")
    print("4. Cancel")
    choice = get_input("Enter choice: ")
    results = []
    if choice == "1":
        status_choice = get_input("Filter by: (1) Complete or (2) Incomplete: ")
        if status_choice == "1":
            results = store.filter_by_status(completed=True)
        elif status_choice == "2":
            results = store.filter_by_status(completed=False)
        else:
            print("Error: Invalid choice.")
            return
    elif choice == "2":
        priority_input = get_input("Filter by priority (H/M/L): ")
        try:
            priority = parse_priority(priority_input)
            results = store.filter_by_priority(priority)
        except ValueError as e:
            print(f"Error: {e}")
            return
    elif choice == "3":
        tag = get_input("Enter tag to filter by (case-insensitive): ")
        if not tag:
            print("Error: Tag cannot be empty.")
            return
        results = store.filter_by_tag(tag)
    elif choice == "4":
        print("Filter canceled.")
        return
    else:
        print("Error: Invalid choice.")
        return
    if not results:
        print("No matches found.")
        return
    print(f"\n=== Filter Results ({len(results)} found) ===")
    print(f"{'ID':<3} | {'Status':<12} | {'Priority':<8} | {'Title'}")
    print("-" * 60)
    for todo in results:
        status = "[x] Complete" if todo.completed else "[ ] Incomplete"
        priority_str = priority_indicator(todo.priority)
        print(f"{todo.id:<3} | {status:<12} | {priority_str:<8} | {todo.title}")
        if todo.description:
            print(f"     Description: {todo.description}")
        if todo.tags:
            tags_str = ", ".join(todo.tags)
            print(f"     Tags: {tags_str}")


def sort_todos(store: TodoStore) -> None:
    """Handle the Sort Todos menu option."""
    print("\nSort by:")
    print("1. Title (A-Z)")
    print("2. Priority (High to Low)")
    print("3. Cancel")
    choice = get_input("Enter choice: ")
    results = []
    sort_header = ""
    if choice == "1":
        results = store.sort_by_title()
        sort_header = "Sorted by Title (A-Z)"
    elif choice == "2":
        results = store.sort_by_priority()
        sort_header = "Sorted by Priority (High to Low)"
    elif choice == "3":
        print("Sort canceled.")
        return
    else:
        print("Error: Invalid choice.")
        return
    if not results:
        print("No todos to sort.")
        return
    print(f"\n=== {sort_header} ({len(results)} todos) ===")
    print(f"{'ID':<3} | {'Status':<12} | {'Priority':<8} | {'Title'}")
    print("-" * 60)
    for todo in results:
        status = "[x] Complete" if todo.completed else "[ ] Incomplete"
        priority_str = priority_indicator(todo.priority)
        print(f"{todo.id:<3} | {status:<12} | {priority_str:<8} | {todo.title}")
        if todo.description:
            print(f"     Description: {todo.description}")
        if todo.tags:
            tags_str = ", ".join(todo.tags)
            print(f"     Tags: {tags_str}")


def display_menu() -> None:
    """Display the main menu."""
    print("\n=== Todo Manager ===")
    print("1. Add todo")
    print("2. View todos")
    print("3. Update todo")
    print("4. Delete todo")
    print("5. Mark complete/incomplete")
    print("6. Search todos")
    print("7. Filter todos")
    print("8. Sort todos")
    print("9. Exit")


def main() -> None:
    """Main entry point for the CLI application."""
    store = TodoStore()
    while True:
        display_menu()
        choice = get_input("\nEnter your choice: ")
        if choice == "1":
            add_todo(store)
        elif choice == "2":
            view_todos(store)
        elif choice == "3":
            update_todo(store)
        elif choice == "4":
            delete_todo(store)
        elif choice == "5":
            toggle_todo(store)
        elif choice == "6":
            search_todos(store)
        elif choice == "7":
            filter_todos(store)
        elif choice == "8":
            sort_todos(store)
        elif choice == "9":
            print("Goodbye!")
            break
        else:
            print("Error: Invalid choice. Please enter 1-9.")


if __name__ == "__main__":
    main()
