# Phase 0 Research: Todo CLI Intermediate Features

**Feature**: 002-todo-cli-intermediate
**Date**: 2025-12-31
**Status**: Complete

## Overview

This document captures technical decisions and research for extending the Phase I Basic Todo CLI with organization features (priority, tags, search, filter, sort). Since this is an extension of an existing working implementation, research focuses on maintaining backward compatibility and architectural consistency.

## Research Areas

### 1. Priority Implementation

**Decision**: Use Python Enum for priority levels

**Rationale**:
- Type-safe representation of High, Medium, Low values
- Native Python stdlib (no external dependencies)
- Integrates cleanly with existing dataclass model
- Provides iteration and comparison capabilities

**Alternatives Considered**:
- String literals: Rejected due to lack of type safety and validation
- Integer constants: Rejected due to poor readability and semantic meaning
- Custom class: Rejected as over-engineering for simple three-value enum

**Implementation Pattern**:
```python
from enum import Enum

class Priority(Enum):
    HIGH = "High"
    MEDIUM = "Medium"
    LOW = "Low"
```

### 2. Tags Storage

**Decision**: Use `list[str]` for tags with validation in model

**Rationale**:
- Simple, Pythonic representation
- Maintains order (though order may not be semantically significant)
- Easy to iterate, add, remove tags
- JSON-serializable for future persistence needs
- No need for complex tag management system at this stage

**Alternatives Considered**:
- `set[str]`: Rejected to maintain deterministic order for display
- Separate Tag class: Rejected as over-engineering for simple string labels
- Dictionary with metadata: Rejected as tags don't need additional properties

**Validation Rules**:
- Tags must be non-empty strings
- Tags are case-sensitive for storage but case-insensitive for filtering
- No maximum number of tags enforced (reasonable usage assumed)

### 3. Case-Insensitive Search

**Decision**: Use Python `str.lower()` for simple substring matching

**Rationale**:
- Efficient for small datasets (<1000 todos as per scope)
- Python stdlib only (no regex engine needed for simple keyword search)
- Deterministic and predictable behavior
- Matches OR logic: title OR description contains keyword

**Alternatives Considered**:
- Regex matching: Rejected as over-engineering for keyword search
- Full-text search library: Rejected due to external dependency constraint
- Fuzzy matching: Rejected as not in requirements

**Performance**:
- O(n*m) where n = number of todos, m = average text length
- Acceptable for target scale (<1000 todos, <1200 chars per todo)

### 4. Filter Implementation

**Decision**: Separate filter methods per filter type in TodoStore

**Rationale**:
- Single responsibility per filter method
- Clear, testable interfaces
- Supports requirement for single filter at a time
- Easy to extend with additional filter types later

**Filter Methods**:
- `filter_by_status(completed: bool) -> list[Todo]`
- `filter_by_priority(priority: Priority) -> list[Todo]`
- `filter_by_tag(tag: str) -> list[Todo]` (case-insensitive tag matching)

**Alternatives Considered**:
- Single filter method with union type: Rejected for complexity
- Composite filters: Explicitly out of scope per requirements
- Filter builder pattern: Rejected as over-engineering for simple use case

### 5. Sort Implementation

**Decision**: Return sorted copy, never mutate storage order

**Rationale**:
- Requirement: "Sorting does not mutate stored data"
- Use Python `sorted()` with custom key functions
- Preserves insertion order in underlying storage
- Deterministic sort order for title (case-insensitive A-Z)
- Priority sort order: High > Medium > Low

**Sort Methods**:
- `sort_by_title() -> list[Todo]` (case-insensitive alphabetical)
- `sort_by_priority() -> list[Todo]` (High first, then Medium, then Low)

**Key Functions**:
- Title: `key=lambda t: t.title.lower()`
- Priority: `key=lambda t: priority_order[t.priority]` (custom ordering dict)

### 6. Backward Compatibility Strategy

**Decision**: Additive changes only; no breaking modifications

**Approach**:
- Extend Todo model with new fields (priority with default, tags as empty list)
- Extend TodoStore with new methods (keep existing methods unchanged)
- Extend CLI menu with new options (keep existing options unchanged)
- Default values ensure existing functionality works without modification

**Testing Strategy**:
- Run all Phase I Basic tests to verify no regressions
- Add new tests for intermediate features
- Integration tests verify new menu options don't break existing flows

### 7. CLI Menu Design

**Decision**: Add new submenu or numbered options for intermediate features

**Menu Structure** (to be detailed in contracts/):
```
Main Menu:
1. Add Todo (extended: prompt for priority and tags)
2. View All Todos (extended: display priority and tags)
3. Update Todo (extended: allow priority and tag updates)
4. Delete Todo (unchanged)
5. Mark Complete/Incomplete (unchanged)
6. Search Todos (NEW)
7. Filter Todos (NEW)
8. Sort Todos (NEW)
9. Exit
```

**Input Validation**:
- Priority: Accept variations (h/high/HIGH → High, case-insensitive)
- Tags: Comma-separated input, strip whitespace
- Search: Accept empty string as "show all"
- Filter: Validate filter type and value before applying

## Technical Debt & Future Considerations

### Known Limitations
- Search is simple substring match (no fuzzy matching)
- Single filter at a time (no compound filters like "High priority AND work tag")
- Tags have no hierarchy or relationships
- No tag autocomplete or suggestion

### Future Extensions (Out of Scope)
- Persistent storage (Phase II)
- Multi-user support
- Tag management (rename, merge tags globally)
- Advanced search (regex, multiple keywords, date ranges)
- Composite filters
- Custom sort orders (by creation date, by update date)

## Dependencies

**Existing**: All Phase I Basic features implemented and tested in `todo-cli/`

**New Python stdlib imports**:
- `enum.Enum` for Priority
- No additional external dependencies

## Conclusion

All technical decisions align with project constitution requirements:
- Python 3.13+ stdlib only
- In-memory storage maintained
- Backward compatible with Phase I Basic
- Clean architectural extension of existing models/services/cli
- Deterministic behavior preserved

Ready to proceed to Phase 1 (Design).
