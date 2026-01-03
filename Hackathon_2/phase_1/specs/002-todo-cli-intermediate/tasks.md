# Tasks: Phase I Intermediate - Todo CLI with Organization Features

**Input**: Design documents from `/specs/002-todo-cli-intermediate/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/cli-commands.md

**Tests**: Not explicitly requested in spec - implementation tasks only (tests can be added later if desired)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

All paths relative to `todo-cli/` directory:
- Models: `todo-cli/src/models/`
- Services: `todo-cli/src/services/`
- CLI: `todo-cli/src/cli/`
- Tests: `todo-cli/tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify existing Phase I Basic implementation is ready for extension

**Checkpoint**: Ensure Phase I Basic is functional before adding intermediate features

- [x] T001 Verify Phase I Basic tests pass (run pytest in todo-cli/)
- [x] T002 Create feature branch 002-todo-cli-intermediate from 001-todo-cli-app
- [x] T003 Document baseline state (capture current test results for regression checking)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core model and service extensions that ALL user stories depend on

**⚠️ CRITICAL**: These must be complete before ANY user story implementation begins

**Checkpoint**: Priority enum and tags field added to Todo model

- [x] T004 [P] Add Priority enum to todo-cli/src/models/todo.py (HIGH, MEDIUM, LOW values)
- [x] T005 Extend Todo dataclass with priority field (default=Priority.MEDIUM) in todo-cli/src/models/todo.py
- [x] T006 Extend Todo dataclass with tags field (default_factory=list) in todo-cli/src/models/todo.py
- [x] T007 Add update_priority() method to Todo class in todo-cli/src/models/todo.py
- [x] T008 [P] Add tag management methods (add_tag, remove_tag, set_tags, has_tag) to Todo class in todo-cli/src/models/todo.py
- [x] T009 Update Todo.to_dict() method to include priority and tags in todo-cli/src/models/todo.py
- [x] T010 Run Phase I Basic tests to verify backward compatibility (no breaking changes)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Set Task Priority (Priority: P1) 🎯 MVP

**Goal**: Users can assign and update priority levels (High/Medium/Low) on todos

**Independent Test**: Add todo with priority, update priority, view list with priorities displayed

**Story Benefits**:
- ✅ Can be implemented completely using foundational model changes (T004-T009)
- ✅ Independent of tags, search, filter, sort
- ✅ Delivers immediate value for task prioritization

### Implementation for User Story 1

- [x] T011 [US1] Extend TodoStore.add() to accept optional priority parameter in todo-cli/src/services/todo_service.py
- [x] T012 [US1] Add TodoStore.update_priority(todo_id, priority) method in todo-cli/src/services/todo_service.py
- [x] T013 [US1] Create parse_priority(input_str) helper function in todo-cli/src/cli/menu.py
- [x] T014 [US1] Extend "Add Todo" CLI flow to prompt for priority (with H/M/L input) in todo-cli/src/cli/menu.py
- [x] T015 [US1] Extend "View All Todos" CLI to display priority with indicators (🔴🟡🟢) in todo-cli/src/cli/menu.py
- [x] T016 [US1] Extend "Update Todo" CLI to add priority update submenu option in todo-cli/src/cli/menu.py
- [x] T017 [US1] Add priority input validation and error handling in todo-cli/src/cli/menu.py
- [x] T018 [US1] Manual test: Create todo with High priority and verify display
- [x] T019 [US1] Manual test: Create todo without priority and verify Medium default
- [x] T020 [US1] Manual test: Update todo priority from Medium to High
- [x] T021 [US1] Run all Phase I Basic tests to verify no regressions

**Checkpoint**: Priority feature complete - todos can be created/updated/viewed with priority levels

---

## Phase 4: User Story 2 - Add Tags to Tasks (Priority: P1)

**Goal**: Users can categorize todos with zero or more string tags

**Independent Test**: Add todos with tags, view tags in listings, update tags on existing todos

**Story Benefits**:
- ✅ Independent of priority, search, filter, sort
- ✅ Uses foundational tag methods from T008
- ✅ Enables organization by categories/projects

### Implementation for User Story 2

- [x] T022 [US2] Extend TodoStore.add() to accept optional tags parameter in todo-cli/src/services/todo_service.py
- [x] T023 [US2] Add TodoStore.update_tags(todo_id, tags) method in todo-cli/src/services/todo_service.py
- [x] T024 [US2] Create parse_tags(input_str) helper function in todo-cli/src/cli/menu.py
- [x] T025 [US2] Extend "Add Todo" CLI flow to prompt for comma-separated tags in todo-cli/src/cli/menu.py
- [x] T026 [US2] Extend "View All Todos" CLI to display tags (comma-separated) in todo-cli/src/cli/menu.py
- [x] T027 [US2] Extend "Update Todo" CLI to add tags submenu (add/remove/replace options) in todo-cli/src/cli/menu.py
- [x] T028 [US2] Implement add tags submenu option in todo-cli/src/cli/menu.py
- [x] T029 [US2] Implement remove tags submenu option in todo-cli/src/cli/menu.py
- [x] T030 [US2] Implement replace all tags submenu option in todo-cli/src/cli/menu.py
- [x] T031 [US2] Add tag validation (non-empty after strip, duplicate removal) in todo-cli/src/cli/menu.py
- [x] T032 [US2] Manual test: Create todo with multiple tags and verify display
- [x] T033 [US2] Manual test: Create todo without tags and verify empty list
- [x] T034 [US2] Manual test: Add, remove, and replace tags on existing todo
- [x] T035 [US2] Run all Phase I Basic tests to verify no regressions

**Checkpoint**: Tags feature complete - todos can have tags added/updated/displayed

---

## Phase 5: User Story 3 - Search Tasks (Priority: P2)

**Goal**: Users can search todos by keyword (case-insensitive, matches title OR description)

**Independent Test**: Create diverse todos, search by keyword, verify case-insensitive matching

**Story Benefits**:
- ✅ Independent of filter and sort
- ✅ Works with or without priority/tags
- ✅ Delivers quick task location for growing lists

### Implementation for User Story 3

- [x] T036 [US3] Implement TodoStore.search(keyword) method (case-insensitive, title OR description) in todo-cli/src/services/todo_service.py
- [x] T037 [US3] Add "Search Todos" menu option (option 6) to main menu in todo-cli/src/cli/menu.py
- [x] T038 [US3] Implement search CLI flow (prompt for keyword) in todo-cli/src/cli/menu.py
- [x] T039 [US3] Display search results using same format as "View All Todos" in todo-cli/src/cli/menu.py
- [x] T040 [US3] Add "No results found" message for empty search results in todo-cli/src/cli/menu.py
- [x] T041 [US3] Handle empty keyword search (show all todos) in todo-cli/src/cli/menu.py
- [x] T042 [US3] Manual test: Search by title keyword and verify matches
- [x] T043 [US3] Manual test: Search by description keyword and verify matches
- [x] T044 [US3] Manual test: Search with mixed case (e.g., "TODO") and verify case-insensitive
- [x] T045 [US3] Manual test: Search with no matches and verify clear message
- [x] T046 [US3] Run all Phase I Basic tests to verify no regressions

**Checkpoint**: Search feature complete - users can find todos by keyword

---

## Phase 6: User Story 4 - Filter Tasks (Priority: P2)

**Goal**: Users can filter todos by status, priority, or tag (single filter at a time)

**Independent Test**: Create todos with various statuses/priorities/tags, apply filters, verify results

**Story Benefits**:
- ✅ Independent of search and sort
- ✅ Requires priority/tags from US1/US2 to be fully useful
- ✅ Enables focused views of task subsets

### Implementation for User Story 4

- [x] T047 [US4] Implement TodoStore.filter_by_status(completed) method in todo-cli/src/services/todo_service.py
- [x] T048 [P] [US4] Implement TodoStore.filter_by_priority(priority) method in todo-cli/src/services/todo_service.py
- [x] T049 [P] [US4] Implement TodoStore.filter_by_tag(tag) method (case-insensitive) in todo-cli/src/services/todo_service.py
- [x] T050 [US4] Add "Filter Todos" menu option (option 7) to main menu in todo-cli/src/cli/menu.py
- [x] T051 [US4] Implement filter submenu (status/priority/tag/cancel options) in todo-cli/src/cli/menu.py
- [x] T052 [US4] Implement filter by status CLI flow (complete/incomplete choice) in todo-cli/src/cli/menu.py
- [x] T053 [US4] Implement filter by priority CLI flow (high/medium/low choice) in todo-cli/src/cli/menu.py
- [x] T054 [US4] Implement filter by tag CLI flow (prompt for tag name) in todo-cli/src/cli/menu.py
- [x] T055 [US4] Display filtered results using same format as "View All Todos" in todo-cli/src/cli/menu.py
- [x] T056 [US4] Add "No matches" message for empty filter results in todo-cli/src/cli/menu.py
- [x] T057 [US4] Add invalid filter choice error handling in todo-cli/src/cli/menu.py
- [x] T058 [US4] Manual test: Filter by complete status and verify only complete todos shown
- [x] T059 [US4] Manual test: Filter by High priority and verify only high-priority todos shown
- [x] T060 [US4] Manual test: Filter by tag (case-insensitive) and verify matching todos shown
- [x] T061 [US4] Manual test: Filter with no matches and verify clear message
- [x] T062 [US4] Run all Phase I Basic tests to verify no regressions

**Checkpoint**: Filter feature complete - users can view filtered task subsets

---

## Phase 7: User Story 5 - Sort Tasks (Priority: P3)

**Goal**: Users can sort todos by title (A-Z) or priority (High→Low) for display only

**Independent Test**: Create todos, apply different sorts, verify order; confirm storage order unchanged

**Story Benefits**:
- ✅ Independent of search and filter
- ✅ Non-mutating (display-only) ensures data integrity
- ✅ Delivers flexible viewing options

### Implementation for User Story 5

- [x] T063 [P] [US5] Implement TodoStore.sort_by_title() method (case-insensitive A-Z, returns copy) in todo-cli/src/services/todo_service.py
- [x] T064 [P] [US5] Implement TodoStore.sort_by_priority() method (High→Medium→Low, returns copy) in todo-cli/src/services/todo_service.py
- [x] T065 [US5] Add "Sort and View Todos" menu option (option 8) to main menu in todo-cli/src/cli/menu.py
- [x] T066 [US5] Implement sort submenu (title/priority/cancel options) in todo-cli/src/cli/menu.py
- [x] T067 [US5] Implement sort by title CLI flow in todo-cli/src/cli/menu.py
- [x] T068 [US5] Implement sort by priority CLI flow in todo-cli/src/cli/menu.py
- [x] T069 [US5] Display sorted results using same format as "View All Todos" in todo-cli/src/cli/menu.py
- [x] T070 [US5] Add header indicating sort order (e.g., "Sorted by Title (A-Z)") in todo-cli/src/cli/menu.py
- [x] T071 [US5] Manual test: Sort by title and verify alphabetical order (case-insensitive)
- [x] T072 [US5] Manual test: Sort by priority and verify High→Medium→Low order
- [x] T073 [US5] Manual test: After sorting, view all todos again and verify original order preserved
- [x] T074 [US5] Run all Phase I Basic tests to verify no regressions

**Checkpoint**: Sort feature complete - users can view todos in different sorted orders

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, documentation, and validation

- [x] T075 Update main menu to show 9 options (1-9) with new commands in todo-cli/src/cli/menu.py
- [x] T076 Verify exit option (9) works correctly with goodbye message in todo-cli/src/cli/menu.py
- [x] T077 Add consistent error messages and user-friendly prompts across all new features in todo-cli/src/cli/menu.py
- [x] T078 Run comprehensive regression test suite (all Phase I Basic tests)
- [x] T079 Manual end-to-end test: Complete user workflow (add with priority/tags, search, filter, sort)
- [x] T080 Update quickstart.md examples with actual CLI output (if not already accurate)
- [x] T081 Create release notes documenting all new features (priority, tags, search, filter, sort)

---

## Dependencies & Execution Strategy

### User Story Dependencies

```
Phase 1 (Setup) → Phase 2 (Foundational) → All User Stories can run in parallel

User Story 1 (Priority): No dependencies on other stories
User Story 2 (Tags): No dependencies on other stories
User Story 3 (Search): No dependencies on other stories (works with or without US1/US2)
User Story 4 (Filter): Enhanced by US1/US2 but can work independently
User Story 5 (Sort): Enhanced by US1 but can work independently

Phase 8 (Polish): Depends on all user stories complete
```

### Parallel Execution Opportunities

**After Phase 2 Complete**:
- User Story 1 (Priority) team can work independently
- User Story 2 (Tags) team can work independently
- User Story 3 (Search) team can work independently
- User Story 4 (Filter) team can work independently
- User Story 5 (Sort) team can work independently

**Within Each User Story**:
- Multiple [P] tasks within a story can run in parallel (different files)
- Example US4: T047, T048, T049 can all be implemented simultaneously

### MVP Delivery Strategy

**MVP = User Story 1 + User Story 2** (Both P1 priorities)
- Delivers core organization features (priority + tags)
- Provides immediate value for task management
- Can be released independently before search/filter/sort

**Incremental Releases**:
1. **Release 1**: Setup + Foundational + US1 (Priority only)
2. **Release 2**: Add US2 (Tags)
3. **Release 3**: Add US3 (Search)
4. **Release 4**: Add US4 (Filter)
5. **Release 5**: Add US5 (Sort) + Polish

Each release is fully functional and independently testable.

---

## Implementation Checklist by File

### `todo-cli/src/models/todo.py` (Extended)
- [x] Priority enum (T004)
- [x] priority field (T005)
- [x] tags field (T006)
- [x] update_priority() (T007)
- [x] Tag methods: add_tag, remove_tag, set_tags, has_tag (T008)
- [x] Extended to_dict() (T009)

### `todo-cli/src/services/todo_service.py` (Extended)
- [x] Extend add() for priority/tags (T011, T022)
- [x] update_priority() (T012)
- [x] update_tags() (T023)
- [x] search() (T036)
- [x] filter_by_status() (T047)
- [x] filter_by_priority() (T048)
- [x] filter_by_tag() (T049)
- [x] sort_by_title() (T063)
- [x] sort_by_priority() (T064)

### `todo-cli/src/cli/menu.py` (Extended)
- [x] parse_priority() helper (T013)
- [x] parse_tags() helper (T024)
- [x] Extend add todo flow (T014, T025)
- [x] Extend view todos display (T015, T026)
- [x] Extend update todo menu (T016, T027-T030)
- [x] Add search menu option (T037-T041)
- [x] Add filter menu option (T050-T057)
- [x] Add sort menu option (T065-T070)
- [x] Update main menu to 9 options (T075)
- [x] Polish error handling (T077)

### Testing Strategy
- Regression: T001, T010, T021, T035, T046, T062, T074, T078 (verify no breaks)
- Manual: T018-T020, T032-T034, T042-T045, T058-T061, T071-T073, T079 (verify features work)

---

## Task Summary

**Total Tasks**: 81
- Phase 1 (Setup): 3 tasks
- Phase 2 (Foundational): 7 tasks (BLOCKING)
- Phase 3 (US1 - Priority): 11 tasks
- Phase 4 (US2 - Tags): 14 tasks
- Phase 5 (US3 - Search): 11 tasks
- Phase 6 (US4 - Filter): 16 tasks
- Phase 7 (US5 - Sort): 12 tasks
- Phase 8 (Polish): 7 tasks

**Parallelizable Tasks**: 8 tasks marked [P]
- T003, T004, T008, T048, T049, T063, T064

**MVP Scope**: Phases 1-4 (Setup + Foundational + US1 + US2) = 35 tasks
**Full Feature**: All 81 tasks

**Estimated Effort**:
- MVP: 35 tasks (Priority + Tags features)
- Full: 81 tasks (All features including Search, Filter, Sort)

---

## Validation Checklist

Format compliance:
- ✅ All tasks have checkboxes `- [x]`
- ✅ All tasks have sequential IDs (T001-T081)
- ✅ All tasks marked [P] are truly parallelizable
- ✅ All user story tasks have [US#] labels
- ✅ All tasks include file paths
- ✅ Tasks organized by user story
- ✅ Independent test criteria for each story
- ✅ Dependency graph provided
- ✅ MVP scope identified

This tasks.md is ready for immediate execution.
