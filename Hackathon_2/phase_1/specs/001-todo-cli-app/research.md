# Research: Todo CLI Application

**Feature**: 001-todo-cli-app
**Date**: 2025-12-31

## Research Summary

No technical unknowns required research. All decisions derived from project constitution and feature specification.

## Decisions

| Decision | Rationale | Alternatives Considered |
|----------|-----------|------------------------|
| Python standard library only | Project constraint (Python standard library sufficient for CRUD operations); no external dependencies needed | typer/click forjected - CLI (re stdlib sufficient) |
| In-memory dict storage | Meets in-memory requirement; O(1) operations; simple implementation | list with linear search (rejected - O(n) lookup) |
| pytest for testing | Industry standard for Python; simple fixture-based tests | unittest (rejected - less ergonomic) |
| Menu-driven CLI | Best UX for interactive terminal use; simple input() loop | argparse (rejected - not interactive enough) |

## Technical Choices

### CLI Interface Pattern

Decision: Menu-driven interaction using `input()` loop

Rationale:
- Simpler for users than remembering command syntax
- Matches specification requirement for "menu-driven or command-based"
- No external dependencies required
- Consistent with console application best practices

### Storage Pattern

Decision: Dictionary with auto-incrementing integer keys

Rationale:
- O(1) lookup by ID for all operations
- Simple ID management (counter variable)
- Meets in-memory requirement
- Natural fit for Python data structures

### Testing Approach

Decision: pytest with fixtures for model and service isolation

Rationale:
- pytest is standard Python testing tool
- Fixtures enable clean test isolation
- pytest-xdist optional for parallel execution if needed
- Standard library unittest alternative rejected for verbosity

## References

- Python Standard Library: https://docs.python.org/3/library/
- pytest Documentation: https://docs.pytest.org/
