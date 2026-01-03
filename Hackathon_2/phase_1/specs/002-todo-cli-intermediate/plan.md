# Implementation Plan: Phase I Intermediate - Todo CLI with Organization Features

**Branch**: `002-todo-cli-intermediate` | **Date**: 2025-12-31 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-todo-cli-intermediate/spec.md`

**Note**: This plan extends the existing Phase I Basic Todo CLI (001-todo-cli-app) with priority, tags, search, filter, and sort capabilities.

## Summary

Extend the existing in-memory Todo CLI application with organization and usability features: priority levels (High/Medium/Low), tags, case-insensitive search, filtering (by status/priority/tag), and sorting (by title/priority). All new features must be backward compatible with Phase I Basic features (add, view, update, delete, mark complete) and maintain the existing architecture: Python 3.13 standard library only, in-memory storage, and clean separation between models/services/cli layers.

## Technical Context

**Language/Version**: Python 3.13+ (required by project constitution)
**Primary Dependencies**: Python standard library only (enum, dataclasses for model extensions)
**Storage**: In-memory dictionary-based storage (extends existing TodoStore from 001-todo-cli-app)
**Testing**: pytest for unit and integration tests
**Target Platform**: Cross-platform CLI (Windows WSL 2, Linux, macOS)
**Project Type**: Single project (CLI application) - extending existing todo-cli structure
**Performance Goals**: Sub-second response time for all operations; case-insensitive search efficient for <1000 todos
**Constraints**:
- Must maintain backward compatibility with Phase I Basic (FR-1 through FR-5)
- In-memory only (no persistence between sessions)
- Deterministic behavior for same input sequence
- Sorting must not mutate stored data order
- Single filter at a time (no composite filters)

**Scale/Scope**: Single user, single session, <1000 todos expected per session

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. AI-Driven Development | PASS | All code extensions via Claude Code, no manual coding |
| II. Spec-Driven Workflow | PASS | Following spec → plan → tasks → implement flow |
| III. Clean Code Practices | PASS | Extending existing models/services/cli separation cleanly |
| IV. Five-Feature Completeness | PASS | Extends existing features; maintains all basic operations |
| V. Iterative Refinement | PASS | Following prompt/spec/plan cycle with PHR tracking |
| Development Standards | PASS | Using UV, Python 3.13+, Python stdlib only |
| Repository Structure | PASS | Following constitution-defined structure in todo-cli/ |

**Result**: All gates pass. No complexity tracking required. Extension maintains architectural integrity of Phase I Basic.

## Project Structure

### Documentation (this feature)

```text
specs/002-todo-cli-intermediate/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (minimal - extending existing patterns)
├── data-model.md        # Phase 1 output (Todo model extensions)
├── quickstart.md        # Phase 1 output (updated usage guide)
├── contracts/           # Phase 1 output
│   └── cli-commands.md  # Extended CLI command specifications
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
todo-cli/
├── src/
│   ├── models/
│   │   ├── todo.py          # EXTEND: Add priority enum and tags list
│   │   └── __init__.py
│   ├── services/
│   │   ├── todo_service.py  # EXTEND: Add search, filter, sort methods
│   │   └── __init__.py
│   ├── cli/
│   │   ├── menu.py          # EXTEND: Add menu options for new features
│   │   └── __init__.py
│   └── __init__.py
│
└── tests/
    ├── unit/
    │   ├── test_todo_model.py      # EXTEND: Test priority and tags
    │   └── test_todo_service.py    # EXTEND: Test search/filter/sort
    └── integration/
        └── test_cli_flow.py         # EXTEND: Test new menu options
```

**Structure Decision**: Single project extending existing todo-cli structure. All changes are additive to maintain backward compatibility. Priority enum added to models/, search/filter/sort logic added to services/, and new menu options added to cli/. No new modules required - clean extension of existing architecture.

## Complexity Tracking

> No violations identified. All constitution principles pass.

## Phase 0: Research

See [research.md](research.md) for detailed technical decisions.

**Key Decisions**:
- Priority: Python Enum (High/Medium/Low)
- Tags: list[str] with validation
- Search: Case-insensitive substring matching via str.lower()
- Filter: Separate methods per filter type (status, priority, tag)
- Sort: Return sorted copy, never mutate storage

**Result**: All unknowns resolved. No external dependencies required. Ready for Phase 1.

## Phase 1: Design

### Data Model

See [data-model.md](data-model.md) for complete entity specifications.

**Summary**:
- Todo model extended with `priority: Priority` (default MEDIUM) and `tags: list[str]` (default empty)
- TodoStore extended with search, filter, sort methods
- All changes backward compatible with Phase I Basic

### API Contracts

See [contracts/cli-commands.md](contracts/cli-commands.md) for CLI interface specifications.

**Summary**:
- Extended menu options 1-3 for priority and tags
- New menu options 6-8 for search, filter, sort
- Maintained menu options 4-5 unchanged
- Priority input: H/M/L (case-insensitive)
- Tags input: Comma-separated strings

### Quickstart Guide

See [quickstart.md](quickstart.md) for user-facing usage documentation.

**Summary**:
- Installation: No new dependencies
- Usage: Extended CLI with new menu options
- Examples: Common workflows and tips

## Constitution Check (Post-Design)

*Re-evaluation after Phase 1 design completion.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. AI-Driven Development | PASS | Design artifacts generated via Claude Code |
| II. Spec-Driven Workflow | PASS | spec → plan → research → data-model → contracts complete |
| III. Clean Code Practices | PASS | Design maintains modular structure, single responsibility |
| IV. Five-Feature Completeness | PASS | All basic features maintained, extended appropriately |
| V. Iterative Refinement | PASS | PHR tracked, design artifacts complete |
| Development Standards | PASS | Python 3.13+, stdlib only, no new external deps |
| Repository Structure | PASS | All artifacts in proper locations per constitution |

**Result**: All gates pass. Design complete and constitutionally compliant. Ready for `/sp.tasks`.

## Next Steps

1. **Generate Implementation Tasks**: Run `/sp.tasks` to break down implementation
2. **Verify Tests**: Ensure all Phase I Basic tests still pass
3. **Implement Extensions**: Follow TDD - tests first, then implementation
4. **Integration Testing**: Verify backward compatibility throughout

## Summary

This plan extends Phase I Basic Todo CLI with organization features while maintaining architectural integrity:

- **Priority Management**: Enum-based (High/Medium/Low) with default Medium
- **Tag System**: List-based with validation and case-insensitive filtering
- **Search**: Case-insensitive keyword matching (title OR description)
- **Filter**: By status, priority, or tag (single filter at a time)
- **Sort**: By title (A-Z) or priority (High→Low), display-only

All changes are additive and backward compatible. No breaking changes to Phase I Basic functionality.
