---

description: "Task list template for feature implementation"
---

# Tasks: Todo CLI Application

**Input**: Design documents from `/specs/001-todo-cli-app/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), data-model.md, contracts/cli-commands.md, quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Single project: `src/`, `tests/` at repository root
- Paths shown below follow the plan.md structure

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Project initialization and basic structure

- [X] T001 Initialize Python project with UV at repository root
- [X] T002 [P] Create pyproject.toml with Python 3.13+ and pytest dependency
- [X] T003 [P] Create src/__init__.py
- [X] T004 [P] Create src/models/__init__.py
- [X] T005 [P] Create src/services/__init__.py
- [X] T006 [P] Create src/cli/__init__.py
- [X] T007 [P] Create tests/__init__.py
- [X] T008 [P] Create tests/unit/__init__.py
- [X] T009 [P] Create tests/integration/__init__.py
- [X] T010 [P] Create pytest.ini or configure pytest in pyproject.toml

---

## Phase 2: Foundational (Core Model & Storage)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T011 [P] Create Todo dataclass in src/models/todo.py with id, title, description, completed fields
- [X] T012 [P] Add validation methods for title (1-200 chars) and description (0-1000 chars) in src/models/todo.py
- [X] T013 Create TodoStore class in src/services/todo_service.py with in-memory dict storage
- [X] T014 [P] Implement TodoStore.add() with auto-increment ID in src/services/todo_service.py
- [X] T015 [P] Implement TodoStore.get() for O(1) lookup in src/services/todo_service.py
- [X] T016 [P] Implement TodoStore.update() in src/services/todo_service.py
- [X] T017 [P] Implement TodoStore.delete() in src/services/todo_service.py
- [X] T018 [P] Implement TodoStore.list_all() in src/services/todo_service.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Add New Todo (Priority: P1)

**Goal**: Users can add tasks with title and optional description

**Independent Test**: Run CLI, select "Add Todo," enter title, verify task appears with ID

### Unit Tests for User Story 1

- [X] T019 [P] [US1] Add unit test for Todo dataclass validation in tests/unit/test_todo_model.py
- [X] T020 [P] [US1] Add unit test for TodoStore.add() in tests/unit/test_todo_service.py

### Implementation for User Story 1

- [X] T021 [US1] Implement CLI add_todo() handler in src/cli/menu.py (accepts title, description)
- [X] T022 [US1] Integrate add_todo() with TodoService.add() in src/cli/menu.py
- [X] T023 [US1] Add error handling for empty title in src/cli/menu.py

**Checkpoint**: Add Todo feature fully functional and testable independently

---

## Phase 4: User Story 2 - View All Todos (Priority: P1)

**Goal**: Users can see all tasks with ID, title, description, and completion status

**Independent Test**: Add multiple todos, select "View Todos," verify all display correctly

### Unit Tests for User Story 2

- [X] T024 [P] [US2] Add unit test for TodoStore.list_all() in tests/unit/test_todo_service.py

### Implementation for User Story 2

- [X] T025 [US2] Implement CLI view_todos() handler in src/cli/menu.py
- [X] T026 [US2] Implement empty state display ("No todos yet") in src/cli/menu.py
- [X] T027 [US2] Implement formatted list display with ID, status, title, description in src/cli/menu.py
- [X] T028 [US2] Integrate view_todos() with TodoService.list_all() in src/cli/menu.py

**Checkpoint**: View Todos feature fully functional - User Stories 1 AND 2 work independently

---

## Phase 5: User Story 3 - Update Todo (Priority: P2)

**Goal**: Users can modify existing task title and description by ID

**Independent Test**: Add todo, update it, verify changes reflected

### Unit Tests for User Story 3

- [X] T029 [P] [US3] Add unit test for TodoStore.update() in tests/unit/test_todo_service.py

### Implementation for User Story 3

- [X] T030 [US3] Implement CLI update_todo() handler in src/cli/menu.py
- [X] T031 [US3] Implement ID validation (error on invalid ID) in src/cli/menu.py
- [X] T032 [US3] Implement partial update (keep current if empty) in src/cli/menu.py
- [X] T033 [US3] Add success confirmation message in src/cli/menu.py
- [X] T034 [US3] Integrate update_todo() with TodoService.update() in src/cli/menu.py

**Checkpoint**: Update Todo feature fully functional

---

## Phase 6: User Story 4 - Delete Todo (Priority: P2)

**Goal**: Users can remove tasks by ID

**Independent Test**: Add multiple todos, delete one, verify removal from list

### Unit Tests for User Story 4

- [X] T035 [P] [US4] Add unit test for TodoStore.delete() in tests/unit/test_todo_service.py

### Implementation for User Story 4

- [X] T036 [US4] Implement CLI delete_todo() handler in src/cli/menu.py
- [X] T037 [US4] Implement ID validation (error on invalid ID) in src/cli/menu.py
- [X] T038 [US4] Add success confirmation message in src/cli/menu.py
- [X] T039 [US4] Integrate delete_todo() with TodoService.delete() in src/cli/menu.py

**Checkpoint**: Delete Todo feature fully functional

---

## Phase 7: User Story 5 - Toggle Completion Status (Priority: P2)

**Goal**: Users can mark tasks complete or incomplete

**Independent Test**: Mark todo complete, verify status change, mark incomplete, verify change

### Unit Tests for User Story 5

- [X] T040 [P] [US5] Add unit test for completion toggle in TodoStore in tests/unit/test_todo_service.py

### Implementation for User Story 5

- [X] T041 [US5] Implement TodoStore.toggle_complete() method in src/services/todo_service.py
- [X] T042 [US5] Implement CLI toggle_todo() handler in src/cli/menu.py
- [X] T043 [US5] Implement sub-menu for complete/incomplete choice in src/cli/menu.py
- [X] T044 [US5] Implement ID validation (error on invalid ID) in src/cli/menu.py
- [X] T045 [US5] Add success confirmation message in src/cli/menu.py
- [X] T046 [US5] Integrate toggle_todo() with TodoStore.toggle_complete() in src/cli/menu.py

**Checkpoint**: All user stories now independently functional

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T047 [P] Create main() function with menu loop in src/cli/menu.py
- [X] T048 [P] Implement exit option (6) in main menu in src/cli/menu.py
- [X] T049 [P] Add integration test for full CLI workflow in tests/integration/test_cli_flow.py
- [X] T050 [P] Verify all error messages are human-readable and consistent
- [X] T051 [P] Ensure application continues after recoverable errors

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-7)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed) since they share the same service layer
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 5 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Unit tests should be written before implementation (optional - TDD style)
- Models before services
- Services before CLI handlers
- Story complete before moving to next priority (if sequential)

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
Task: "Add unit test for Todo dataclass validation in tests/unit/test_todo_model.py"
Task: "Add unit test for TodoStore.add() in tests/unit/test_todo_service.py"
Task: "Implement CLI add_todo() handler in src/cli/menu.py"
Task: "Integrate add_todo() with TodoService.add() in src/cli/menu.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test Add Todo independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add User Story 5 → Test independently → Deploy/Demo
7. Add Phase 8 Polish → Final delivery
8. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
