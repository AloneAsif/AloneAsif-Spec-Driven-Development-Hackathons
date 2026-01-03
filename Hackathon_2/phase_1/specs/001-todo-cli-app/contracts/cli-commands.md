# CLI Contracts: Todo Application

**Feature**: 001-todo-cli-app
**Date**: 2025-12-31

## User Interface

The CLI presents a menu-driven interface with numbered options.

### Main Menu

```
=== Todo Manager ===
1. Add todo
2. View todos
3. Update todo
4. Delete todo
5. Mark complete/incomplete
6. Exit

Enter your choice:
```

## Command Contracts

### 1. Add Todo

**Input**: User provides title and optional description

```
1. Add todo
Enter title: [user input - required]
Enter description (optional): [user input - can be empty]
```

**Output**:
- Success: "Todo added successfully with ID {id}"
- Error: "Error: Title cannot be empty"

### 2. View Todos

**Input**: User selects option (no additional input required)

**Output**:
- If todos exist:
  ```
  === Your Todos ===
  ID | Status      | Title
  ---+-------------+----------------
  1  | [ ] Incomplete | Buy groceries
  2  | [x] Complete   | Call mom

  2 todos displayed.
  ```
- If empty:
  ```
  No todos yet. Add your first todo!
  ```

### 3. Update Todo

**Input**: User provides ID and new values

```
3. Update todo
Enter todo ID: [user input - numeric]
Enter new title (leave empty to keep current): [user input]
Enter new description (leave empty to keep current): [user input]
```

**Output**:
- Success: "Todo {id} updated successfully."
- Error: "Error: Todo with ID {id} not found."

### 4. Delete Todo

**Input**: User provides ID

```
4. Delete todo
Enter todo ID: [user input - numeric]
```

**Output**:
- Success: "Todo {id} deleted successfully."
- Error: "Error: Todo with ID {id} not found."
- Confirmation prompt (optional): "Delete todo {id}? (y/n):"

### 5. Mark Complete/Incomplete

**Input**: User provides ID and desired status

```
5. Mark complete/incomplete
Enter todo ID: [user input - numeric]
Choose status:
1. Mark complete
2. Mark incomplete
Enter choice: [user input - 1 or 2]
```

**Output**:
- Success: "Todo {id} marked as complete."
- Success: "Todo {id} marked as incomplete."
- Error: "Error: Todo with ID {id} not found."

## Error Taxonomy

| Error Type | User Message | Recovery |
|------------|--------------|----------|
| Invalid ID | "Error: Todo with ID {id} not found." | User can try different ID or add new todo |
| Empty title | "Error: Title cannot be empty." | User prompted to re-enter title |
| Non-numeric ID | "Error: Please enter a valid number." | User prompted to re-enter ID |
| Canceled operation | "Operation canceled." | User returns to main menu |

## Input Validation

- Title: 1-200 characters, non-empty
- Description: 0-1000 characters, optional
- ID: Positive integer corresponding to existing todo
