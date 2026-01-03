# Quickstart Guide: Todo CLI Intermediate

**Feature**: 002-todo-cli-intermediate
**Date**: 2025-12-31

## Overview

This guide covers the extended Phase I Intermediate Todo CLI with priority, tags, search, filter, and sort capabilities. All Phase I Basic features are preserved and enhanced.

## Prerequisites

- Python 3.13 or higher installed
- UV package manager installed
- Existing Phase I Basic todo-cli implementation (001-todo-cli-app)

## Installation & Setup

### 1. Switch to Feature Branch

```bash
git checkout 002-todo-cli-intermediate
```

### 2. Navigate to Project Directory

```bash
cd todo-cli
```

### 3. Install Dependencies (if needed)

```bash
uv sync
```

*Note: No new external dependencies required - Python stdlib only*

### 4. Run the Application

```bash
# Using UV
uv run python -m src.cli.menu

# Or activate venv and run directly
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
python -m src.cli.menu
```

## Quick Feature Tour

### Priority Management

**Set priority when creating a todo:**
```
=== Todo CLI ===
1. Add Todo

Enter title: Finish quarterly report
Enter description: Q4 analysis and recommendations
Enter priority (H/M/L, default Medium): H
Enter tags (comma-separated, optional): work, urgent

✓ Todo added successfully!
  ID: 1
  Priority: High
  Tags: work, urgent
```

**Update priority on existing todo:**
```
3. Update Todo

Enter todo ID: 1

What would you like to update?
3. Priority

Enter new priority (H/M/L, current: High): M
✓ Priority updated to Medium
```

### Tag Management

**Add tags when creating:**
```
Enter tags (comma-separated, optional): work, important, q4

✓ Tags: work, important, q4
```

**Manage tags on existing todo:**
```
3. Update Todo → 4. Tags

Current tags: work, important

1. Add tags
2. Remove tags
3. Replace all tags

Enter choice: 1
Enter tags to add: urgent, deadline
✓ Tags added: urgent, deadline

Result: work, important, urgent, deadline
```

### Search Functionality

**Search by keyword (case-insensitive):**
```
6. Search Todos

Enter search keyword: report

=== Search Results (2 matches for "report") ===

[1] Finish quarterly report [HIGH] 🔴
    Description: Q4 analysis and recommendations
    Tags: work, urgent
    Status: Incomplete

[5] Review expense reports [MEDIUM] 🟡
    Tags: finance, review
    Status: Complete ✓
```

**Search matches title OR description:**
- Keyword "report" matches title "Finish quarterly report"
- Keyword "Q4" matches description "Q4 analysis and recommendations"

### Filter Operations

**Filter by status:**
```
7. Filter Todos → 1. By Status → 2. Incomplete

=== Filtered: Incomplete (3 items) ===

[1] Finish quarterly report [HIGH] 🔴
[2] Buy groceries [HIGH] 🔴
[4] Call dentist [LOW] 🟢
```

**Filter by priority:**
```
7. Filter Todos → 2. By Priority → 1. High

=== Filtered: High Priority (2 items) ===

[1] Finish quarterly report [HIGH] 🔴
[2] Buy groceries [HIGH] 🔴
```

**Filter by tag (case-insensitive):**
```
7. Filter Todos → 3. By Tag

Enter tag name: work

=== Filtered: Tag "work" (4 items) ===

[1] Finish quarterly report [HIGH] 🔴
[3] Email team updates [MEDIUM] 🟡
[6] Schedule meetings [LOW] 🟢
```

### Sort Operations

**Sort by title (alphabetical):**
```
8. Sort and View Todos → 1. By Title (A-Z)

=== Sorted by Title (A-Z) ===

[2] Buy groceries [HIGH] 🔴
[4] Call dentist [LOW] 🟢
[3] Email team updates [MEDIUM] 🟡
[1] Finish quarterly report [HIGH] 🔴
```

**Sort by priority (High → Low):**
```
8. Sort and View Todos → 2. By Priority (High to Low)

=== Sorted by Priority (High to Low) ===

[1] Finish quarterly report [HIGH] 🔴
[2] Buy groceries [HIGH] 🔴
[3] Email team updates [MEDIUM] 🟡
[4] Call dentist [LOW] 🟢
```

*Note: Sorting only affects display order - original storage order is preserved*

## Common Workflows

### Workflow 1: Organize Work Tasks by Priority

```
1. Add multiple work tasks with priorities:
   - "Finish report" → Priority: High, Tags: work, urgent
   - "Email updates" → Priority: Medium, Tags: work
   - "Schedule meeting" → Priority: Low, Tags: work

2. Filter by tag "work"
3. Sort by priority to see High-priority items first
```

### Workflow 2: Track Shopping by Tags

```
1. Add tasks with tags:
   - "Buy groceries" → Tags: shopping, personal
   - "Pick up prescription" → Tags: health, personal
   - "Return package" → Tags: errands, personal

2. Filter by tag "personal" to see all personal tasks
3. Mark complete as you finish each item
4. Filter by status "Complete" to see what's done
```

### Workflow 3: Search for Related Tasks

```
1. Need to find all tasks related to "report":
   - Use "6. Search Todos"
   - Enter: "report"
   - See all tasks with "report" in title or description

2. Review results and update priorities as needed
```

## Tips & Best Practices

### Priority Usage
- **High**: Urgent tasks with deadlines, critical importance
- **Medium**: Regular tasks, normal priority (default)
- **Low**: Nice-to-have tasks, can be deferred

### Tag Conventions
- Use consistent tag names: "work" not "Work" or "WORK" (though filtering is case-insensitive)
- Keep tags short and descriptive: "urgent", "work", "personal", "health"
- Common tag patterns:
  - Context: work, home, errands
  - Projects: q4-report, vacation-planning
  - Time: urgent, deadline, someday
  - Energy: quick-win, deep-work

### Search Strategy
- Use specific keywords for targeted results
- Remember: empty search shows all todos
- Search is case-insensitive and matches title OR description

### Filter vs Search
- **Use Filter** when you want to see todos by specific criteria (status, priority, tag)
- **Use Search** when you want to find todos containing specific text
- **Combine** by first filtering, then doing another operation on those results

### Sort Usage
- Sort by title for alphabetical browsing
- Sort by priority to focus on high-priority items first
- Remember: sorting doesn't change storage order, only display

## Keyboard Shortcuts & Quick Actions

### Priority Input Shortcuts
- `h` or `H` → High
- `m` or `M` → Medium (or just press Enter)
- `l` or `L` → Low
- Case-insensitive: "high", "HIGH", "High" all work

### Tag Input Format
- Comma-separated: `work, urgent, q4`
- Spaces are trimmed: `work , urgent , q4` → `work, urgent, q4`
- Empty tags ignored: `work, , urgent` → `work, urgent`

## Troubleshooting

### Priority Not Showing
**Issue**: Added todo but don't see priority indicator
**Solution**: Priority is always shown in View/Search/Filter/Sort outputs with format `[HIGH] 🔴`

### Tags Not Filtering
**Issue**: Filter by tag returns no results
**Solution**: Check tag spelling (filtering is case-insensitive but must match exactly)

### Sort Order Reverts
**Issue**: Sorted by title, but next view shows original order
**Solution**: This is expected - sorting is for display only, doesn't change storage order

### Search Too Broad
**Issue**: Search returns too many results
**Solution**: Use more specific keywords or filter first, then search within filtered results by running search on a new session

### Can't Update Multiple Fields at Once
**Issue**: Want to update title AND priority together
**Solution**: Use "3. Update Todo" repeatedly - update title first, then go back and update priority

## Data Persistence Note

**Important**: This is Phase I (Intermediate) - data is still **in-memory only**.

- All todos are lost when you exit the application
- No files, databases, or persistence mechanisms
- Each run starts with an empty todo list

*Phase II will add persistent storage (JSON file)*

## Next Steps

### Learn More
- Read [data-model.md](data-model.md) for entity specifications
- Read [cli-commands.md](contracts/cli-commands.md) for detailed command contracts
- Read [plan.md](plan.md) for architecture decisions

### Run Tests
```bash
# Run all tests
pytest

# Run specific test file
pytest tests/unit/test_todo_model.py
pytest tests/unit/test_todo_service.py

# Run with coverage
pytest --cov=src --cov-report=html
```

### Extend the Application
- Review `/sp.tasks` output for implementation checklist
- Follow TDD: write tests first, then implement
- Maintain backward compatibility with Phase I Basic

## Support & Feedback

For issues or questions:
1. Check the [spec.md](spec.md) for requirements clarity
2. Review [research.md](research.md) for technical decisions
3. Consult [CLAUDE.md](../../CLAUDE.md) for development workflow
4. Create a PHR (Prompt History Record) for significant changes

## Version Information

- **Feature**: 002-todo-cli-intermediate
- **Branch**: 002-todo-cli-intermediate
- **Python**: 3.13+
- **Dependencies**: Python standard library only
- **Status**: Design Complete, Ready for Implementation
