# CLI Command Specifications: Todo CLI Intermediate

**Feature**: 002-todo-cli-intermediate
**Date**: 2025-12-31
**Status**: Design Complete

## Overview

This document specifies the extended CLI interface for Phase I Intermediate features. All Phase I Basic commands are preserved with extensions for priority and tags. New commands added for search, filter, and sort operations.

## Menu Structure

### Main Menu (Extended)

```
=== Todo CLI ===
1. Add Todo
2. View All Todos
3. Update Todo
4. Delete Todo
5. Mark Complete/Incomplete
6. Search Todos
7. Filter Todos
8. Sort and View Todos
9. Exit

Enter choice (1-9):
```

## Command Specifications

### 1. Add Todo (EXTENDED)

**Prompts**:
```
Enter title: [user input]
Enter description (optional, press Enter to skip): [user input]
Enter priority (H/M/L, default Medium): [user input]
Enter tags (comma-separated, optional): [user input]
```

**Input Validation**:
- **Title**: Required, 1-200 characters after stripping
- **Description**: Optional, max 1000 characters
- **Priority**:
  - Accept: `h`, `high`, `H`, `HIGH` → Priority.HIGH
  - Accept: `m`, `medium`, `M`, `MEDIUM` → Priority.MEDIUM
  - Accept: `l`, `low`, `L`, `LOW` → Priority.LOW
  - Empty input → Priority.MEDIUM (default)
  - Invalid → Show error, reprompt
- **Tags**:
  - Parse comma-separated input
  - Strip whitespace from each tag
  - Remove empty tags
  - Allow duplicates to be filtered

**Success Output**:
```
✓ Todo added successfully!
  ID: 3
  Title: Buy groceries
  Description: Milk, eggs, bread
  Priority: High
  Tags: shopping, personal
  Status: Incomplete
```

**Error Output**:
```
✗ Error: Title cannot be empty
✗ Error: Invalid priority. Use H/M/L or press Enter for Medium
✗ Error: Title must be between 1 and 200 characters
```

### 2. View All Todos (EXTENDED)

**Output Format**:
```
=== All Todos (5 items) ===

[1] Buy groceries [HIGH] 🔴
    Description: Milk, eggs, bread
    Tags: shopping, personal
    Status: Incomplete

[2] Call dentist [MEDIUM] 🟡
    Status: Complete ✓

[3] Finish report [HIGH] 🔴
    Description: Q4 analysis due Friday
    Tags: work, urgent
    Status: Incomplete

(No todos message if empty):
No todos found. Use option 1 to add your first todo!
```

**Priority Indicators**:
- HIGH: `[HIGH] 🔴` (red circle)
- MEDIUM: `[MEDIUM] 🟡` (yellow circle)
- LOW: `[LOW] 🟢` (green circle)

**Display Rules**:
- Always show ID, title, priority indicator
- Show description only if non-empty
- Show tags only if non-empty (comma-separated)
- Show status (Complete ✓ / Incomplete)
- Blank line between todos

### 3. Update Todo (EXTENDED)

**Prompts**:
```
Enter todo ID: [user input]

Current details:
  Title: Buy groceries
  Description: Milk, eggs, bread
  Priority: High
  Tags: shopping, personal

What would you like to update?
1. Title
2. Description
3. Priority
4. Tags
5. Cancel

Enter choice (1-5):
```

**Subcommands**:

**3.1 Update Title**:
```
Enter new title (current: Buy groceries): [user input]
✓ Title updated successfully
```

**3.2 Update Description**:
```
Enter new description (current: Milk, eggs, bread): [user input]
✓ Description updated successfully
```

**3.3 Update Priority**:
```
Enter new priority (H/M/L, current: High): [user input]
✓ Priority updated to Medium
```

**3.4 Update Tags**:
```
Current tags: shopping, personal

1. Add tags
2. Remove tags
3. Replace all tags

Enter choice (1-3):

(If 1 - Add):
Enter tags to add (comma-separated): [user input]
✓ Tags added: urgent, important

(If 2 - Remove):
Enter tags to remove (comma-separated): [user input]
✓ Tags removed: personal

(If 3 - Replace):
Enter new tags (comma-separated, empty for none): [user input]
✓ Tags replaced
```

**Error Output**:
```
✗ Error: Todo ID 99 not found
✗ Error: Invalid priority. Use H/M/L
✗ Error: Invalid choice. Enter 1-5
```

### 4. Delete Todo (UNCHANGED)

**Prompts**:
```
Enter todo ID to delete: [user input]

Are you sure you want to delete this todo?
  [5] Buy groceries
Confirm (y/n):
```

**Success Output**:
```
✓ Todo deleted successfully
```

**Error Output**:
```
✗ Error: Todo ID 99 not found
✗ Deletion cancelled
```

### 5. Mark Complete/Incomplete (UNCHANGED)

**Prompts**:
```
Enter todo ID: [user input]
✓ Todo marked as complete
(or)
✓ Todo marked as incomplete
```

**Error Output**:
```
✗ Error: Todo ID 99 not found
```

### 6. Search Todos (NEW)

**Prompts**:
```
Enter search keyword: [user input]
```

**Success Output**:
```
=== Search Results (3 matches for "report") ===

[3] Finish report [HIGH] 🔴
    Description: Q4 analysis due Friday
    Tags: work, urgent
    Status: Incomplete

[7] Review reports [MEDIUM] 🟡
    Tags: work
    Status: Complete ✓

[12] Report bug [LOW] 🟢
    Description: Login form validation error
    Tags: development
    Status: Incomplete
```

**No Results Output**:
```
No todos found matching "xyz"
```

**Empty Search**:
```
(Shows all todos - same as View All Todos)
```

**Search Behavior**:
- Case-insensitive keyword matching
- Matches title OR description
- Returns results sorted by ID
- Displays in same format as View All Todos

### 7. Filter Todos (NEW)

**Prompts**:
```
=== Filter Todos ===
1. By Status (Complete/Incomplete)
2. By Priority (High/Medium/Low)
3. By Tag
4. Cancel

Enter choice (1-4):
```

**7.1 Filter by Status**:
```
Filter by:
1. Complete
2. Incomplete

Enter choice (1-2): [user input]

=== Filtered: Complete (2 items) ===

[2] Call dentist [MEDIUM] 🟡
    Status: Complete ✓

[7] Review reports [MEDIUM] 🟡
    Tags: work
    Status: Complete ✓
```

**7.2 Filter by Priority**:
```
Filter by priority:
1. High
2. Medium
3. Low

Enter choice (1-3): [user input]

=== Filtered: High Priority (3 items) ===

[1] Buy groceries [HIGH] 🔴
    Description: Milk, eggs, bread
    Tags: shopping, personal
    Status: Incomplete

[3] Finish report [HIGH] 🔴
    Description: Q4 analysis due Friday
    Tags: work, urgent
    Status: Incomplete
```

**7.3 Filter by Tag**:
```
Enter tag name: [user input]

=== Filtered: Tag "work" (4 items) ===

[3] Finish report [HIGH] 🔴
    Description: Q4 analysis due Friday
    Tags: work, urgent
    Status: Incomplete

[7] Review reports [MEDIUM] 🟡
    Tags: work
    Status: Complete ✓
```

**Error/Empty Output**:
```
✗ Invalid choice. Enter 1-4
No todos match the selected filter
```

**Filter Behavior**:
- Single filter at a time (no composite filters)
- Tag matching is case-insensitive
- Results displayed in same format as View All Todos
- Results sorted by ID

### 8. Sort and View Todos (NEW)

**Prompts**:
```
=== Sort Todos ===
1. By Title (A-Z)
2. By Priority (High to Low)
3. Cancel

Enter choice (1-3):
```

**8.1 Sort by Title**:
```
=== Sorted by Title (A-Z) ===

[2] Call dentist [MEDIUM] 🟡
    Status: Complete ✓

[1] Buy groceries [HIGH] 🔴
    Description: Milk, eggs, bread
    Tags: shopping, personal
    Status: Incomplete

[3] Finish report [HIGH] 🔴
    Description: Q4 analysis due Friday
    Tags: work, urgent
    Status: Incomplete
```

**8.2 Sort by Priority**:
```
=== Sorted by Priority (High to Low) ===

[1] Buy groceries [HIGH] 🔴
    Description: Milk, eggs, bread
    Tags: shopping, personal
    Status: Incomplete

[3] Finish report [HIGH] 🔴
    Description: Q4 analysis due Friday
    Tags: work, urgent
    Status: Incomplete

[2] Call dentist [MEDIUM] 🟡
    Status: Complete ✓

[12] Report bug [LOW] 🟢
    Description: Login form validation error
    Tags: development
    Status: Incomplete
```

**Sort Behavior**:
- Title sort: Case-insensitive alphabetical (A-Z)
- Priority sort: High → Medium → Low (secondary sort by ID for same priority)
- Does NOT mutate storage order
- Results displayed in same format as View All Todos

### 9. Exit (UNCHANGED)

**Output**:
```
Goodbye! Your todos will not be saved (in-memory only).
```

## Input Parsing Rules

### Priority Parsing
```python
def parse_priority(input_str: str) -> Priority:
    """Parse user input to Priority enum.

    Accepts:
    - h, high, H, HIGH → Priority.HIGH
    - m, medium, M, MEDIUM → Priority.MEDIUM
    - l, low, L, LOW → Priority.LOW
    - Empty string → Priority.MEDIUM (default)

    Raises:
        ValueError: If input is invalid
    """
```

### Tag Parsing
```python
def parse_tags(input_str: str) -> list[str]:
    """Parse comma-separated tags.

    - Split on commas
    - Strip whitespace from each
    - Remove empty strings
    - Remove duplicates while preserving order

    Returns:
        List of cleaned tag strings
    """
```

## Error Handling

### Common Error Messages

| Error Condition | Message |
|----------------|---------|
| Invalid menu choice | `✗ Invalid choice. Please enter a number between 1 and 9.` |
| Empty title | `✗ Error: Title cannot be empty` |
| Title too long | `✗ Error: Title must be between 1 and 200 characters` |
| Description too long | `✗ Error: Description must not exceed 1000 characters` |
| Invalid priority | `✗ Error: Invalid priority. Use H/M/L or press Enter for Medium` |
| Todo not found | `✗ Error: Todo ID {id} not found` |
| Invalid ID format | `✗ Error: Please enter a valid number` |
| Empty tag | `✗ Error: Tags cannot be empty after stripping whitespace` |

### Error Recovery

- **Invalid Input**: Display error, reprompt (don't return to main menu)
- **Not Found Errors**: Display error, return to main menu
- **Validation Errors**: Display error, reprompt with current value shown

## Display Conventions

### Symbols
- ✓ Success checkmark
- ✗ Error cross
- 🔴 High priority (red circle)
- 🟡 Medium priority (yellow circle)
- 🟢 Low priority (green circle)

### Formatting
- **Bold** for headers (use ANSI codes or plain text with `===`)
- IDs in square brackets: `[3]`
- Priority in square brackets with indicator: `[HIGH] 🔴`
- Tags as comma-separated list: `Tags: work, urgent, personal`
- Status as: `Status: Complete ✓` or `Status: Incomplete`

### Empty States
- No todos: `No todos found. Use option 1 to add your first todo!`
- No search results: `No todos found matching "{keyword}"`
- No filter results: `No todos match the selected filter`

## Accessibility Considerations

- Provide text alternatives for emoji indicators (already included: `[HIGH]`, `[MEDIUM]`, `[LOW]`)
- Use clear, descriptive labels
- Confirm destructive actions (delete)
- Provide cancel options in submenus

## Testing Contracts

### Manual Test Scripts

**Test 1: Add Todo with Priority and Tags**
```
1. Select "1. Add Todo"
2. Enter title: "Test Task"
3. Enter description: "Test description"
4. Enter priority: "H"
5. Enter tags: "test, sample"
6. Verify: Todo created with High priority and both tags
```

**Test 2: Search Case-Insensitive**
```
1. Add todo with title "TODO Item"
2. Add todo with title "task to do"
3. Select "6. Search Todos"
4. Enter keyword: "todo"
5. Verify: Both todos returned
```

**Test 3: Filter by Tag Case-Insensitive**
```
1. Add todo with tag "Work"
2. Add todo with tag "WORK"
3. Select "7. Filter Todos" → "3. By Tag"
4. Enter tag: "work"
5. Verify: Both todos returned
```

**Test 4: Sort Does Not Mutate**
```
1. Add todos in order: "Zebra", "Alpha", "Beta"
2. Select "8. Sort and View Todos" → "1. By Title"
3. Verify: Displayed as "Alpha", "Beta", "Zebra"
4. Select "2. View All Todos"
5. Verify: Original order "Zebra", "Alpha", "Beta" maintained
```

## Backward Compatibility Tests

**Test 5: Phase I Basic Operations Still Work**
```
1. Add todo (without priority/tags prompts - use defaults)
2. View all todos
3. Update todo (title and description only)
4. Delete todo
5. Mark todo complete/incomplete
6. Verify: All operations function as in Phase I Basic
```
