# Feature Specification: Phase I Intermediate - Todo CLI with Organization Features

**Feature Branch**: `002-todo-cli-intermediate`
**Created**: 2025-12-31
**Status**: Draft
**Input**: User description for Phase I (Intermediate) enhancement of Todo CLI with priority, tags, search, filter, and sort capabilities

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Set Task Priority (Priority: P1)

As a user, I want to assign a priority level (High, Medium, Low) to each task so that I can focus on what's most important.

**Why this priority**: Priority is a core organizational feature that helps users manage workload and focus on critical tasks. Without priority, users cannot differentiate between urgent and routine tasks.

**Independent Test**: Can be tested by adding a todo with priority, updating a todo's priority, and verifying the priority is displayed in task listings.

**Acceptance Scenarios**:

1. **Given** the application is running, **When** I add a todo without specifying priority, **Then** Medium is assigned by default.
2. **Given** the application is running, **When** I add a todo with priority High, **Then** the todo is saved with High priority.
3. **Given** a todo exists with Medium priority, **When** I update it to High, **Then** the priority changes to High.
4. **Given** todos exist with different priorities, **When** I view the list, **Then** each todo shows its priority level.

---

### User Story 2 - Add Tags to Tasks (Priority: P1)

As a user, I want to categorize tasks with tags so that I can organize and group related tasks across different projects or contexts.

**Why this priority**: Tags enable multi-dimensional organization beyond simple lists, allowing users to group tasks by project, context, or any custom category. This is essential for users managing complex task sets.

**Independent Test**: Can be tested by adding todos with tags, viewing the list to confirm tags are displayed, and updating tags on existing todos.

**Acceptance Scenarios**:

1. **Given** the application is running, **When** I add a todo with one or more tags, **Then** the todo is saved with those tags.
2. **Given** the application is running, **When** I add a todo without tags, **Then** an empty tags list is stored.
3. **Given** a todo exists, **When** I update it to add new tags, **Then** the todo now includes all specified tags.
4. **Given** a todo exists with tags, **When** I update it to remove all tags, **Then** the todo has an empty tags list.
5. **Given** todos exist with various tags, **When** I view the list, **Then** each todo shows its associated tags.

---

### User Story 3 - Search Tasks (Priority: P2)

As a user, I want to search for tasks by keyword so that I can quickly find specific tasks without scrolling through the entire list.

**Why this priority**: Search is essential for usability when the task list grows. Users need to quickly locate specific tasks by title or description content.

**Independent Test**: Can be tested by creating tasks with various titles and descriptions, performing keyword searches, and verifying matching tasks are returned.

**Acceptance Scenarios**:

1. **Given** todos exist with matching keywords in title, **When** I search for that keyword, **Then** all matching todos are displayed.
2. **Given** todos exist with matching keywords in description, **When** I search for that keyword, **Then** all matching todos are displayed.
3. **Given** todos exist, **When** I search for a keyword that matches both title and description, **Then** matching todos are displayed.
4. **Given** no todos match the search term, **When** I search, **Then** I receive a clear message indicating no results found.
5. **Given** the search is case-insensitive, **When** I search for "TODO", **Then** matches include "todo", "Todo", "TODO", etc.

---

### User Story 4 - Filter Tasks (Priority: P2)

As a user, I want to filter the task list by status, priority, or tag so that I can focus on specific subsets of tasks.

**Why this priority**: Filtering helps users concentrate on specific task groups, such as all incomplete tasks or all high-priority items, improving task management efficiency.

**Independent Test**: Can be tested by creating tasks with different statuses, priorities, and tags, applying filters, and verifying only matching tasks are displayed.

**Acceptance Scenarios**:

1. **Given** todos exist with various statuses, **When** I filter by status complete, **Then** only complete todos are displayed.
2. **Given** todos exist with various priorities, **When** I filter by priority High, **Then** only high-priority todos are displayed.
3. **Given** todos exist with various tags, **When** I filter by tag "work", **Then** only todos with that tag are displayed.
4. **Given** I provide an invalid filter value, **When** I apply the filter, **Then** I receive a user-friendly error message.
5. **Given** no todos match the filter criteria, **When** I apply the filter, **Then** I see a message indicating no tasks match.

---

### User Story 5 - Sort Tasks (Priority: P3)

As a user, I want to sort the task list by title or priority so that I can view tasks in a meaningful order.

**Why this priority**: Sorting provides organizational flexibility for users who prefer alphabetical or priority-based task views. It enhances usability without changing the underlying data.

**Independent Test**: Can be tested by creating tasks with various titles and priorities, applying different sort orders, and verifying tasks appear in the correct order.

**Acceptance Scenarios**:

1. **Given** todos exist, **When** I sort by title A-Z, **Then** tasks are displayed in alphabetical order.
2. **Given** todos exist, **When** I sort by priority High to Low, **Then** High priority tasks appear first, followed by Medium, then Low.
3. **Given** todos are sorted, **When** I view the list, **Then** the sorted order is displayed but the stored order remains unchanged.

---

### Edge Cases

- What happens when the user provides an invalid priority level (not High, Medium, or Low)?
- How does the system handle duplicate tags on the same task?
- What happens when a tag is removed from all tasks (does the tag still exist conceptually)?
- How does search handle special characters or empty search terms?
- What is the maximum number of tags allowed per task?
- How does the system behave when filtering by a tag that no tasks have?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-006**: System MUST allow users to set task priority as High, Medium (default), or Low when creating a todo.
- **FR-007**: System MUST allow users to update the priority of an existing todo.
- **FR-008**: System MUST display the priority level for each todo in task listings.
- **FR-009**: System MUST allow users to add zero or more string tags to a todo when creating it.
- **FR-010**: System MUST allow users to add, remove, or replace tags on an existing todo.
- **FR-011**: System MUST display all associated tags for each todo in task listings.
- **FR-012**: System MUST support case-insensitive keyword search that matches against todo title and description.
- **FR-013**: System MUST display all todos whose title OR description contains the search keyword.
- **FR-014**: System MUST display a clear message when no todos match the search criteria.
- **FR-015**: System MUST allow users to filter the todo list by status (complete/incomplete).
- **FR-016**: System MUST allow users to filter the todo list by priority (High, Medium, Low).
- **FR-017**: System MUST allow users to filter the todo list by tag name.
- **FR-018**: System MUST apply only one filter at a time.
- **FR-019**: System MUST display a user-friendly error message for invalid filter values.
- **FR-020**: System MUST allow users to sort the todo list by title in ascending alphabetical order (A-Z).
- **FR-021**: System MUST allow users to sort the todo list by priority from High to Low.
- **FR-022**: System MUST NOT mutate the stored task order when sorting; sorting is for display only.

### Non-Functional Requirements

- **NFR-001**: System MUST be backward compatible with all Phase I Basic features (FR-1 through FR-5).
- **NFR-002**: All data MUST remain in-memory only; no persistence between sessions.
- **NFR-003**: System MUST produce deterministic behavior for the same input sequence.
- **NFR-004**: All code MUST use Python standard library only.
- **NFR-005**: All source code MUST be generated via Claude Code with no manual edits.

### Key Entities

- **Todo**: Extends the existing Todo entity with the following additional attributes:
  - `priority`: Enum value (High, Medium, Low) with Medium as default
  - `tags`: List of strings representing categories or labels associated with the task

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can set priority (High, Medium, Low) when creating or updating any todo.
- **SC-002**: Users can add, update, and remove tags on any todo.
- **SC-003**: Users can search todos using case-insensitive keywords that match title or description.
- **SC-004**: Users can filter todos by a single criterion (status, priority, or tag) and receive appropriate results.
- **SC-005**: Users can sort todos by title (A-Z) or priority (High to Low) for display purposes.
- **SC-006**: All existing Phase I Basic operations continue to work without modification.
- **SC-007**: Task listings clearly display priority and all associated tags for each todo.
- **SC-008**: Invalid inputs (invalid priority, invalid filter, empty search) result in clear error messages.

### Assumptions

- Priority levels are limited to exactly three values: High, Medium, Low.
- Tags are simple strings without special formatting requirements.
- Search performs simple substring matching (not full-text search engine).
- Sorting affects only the display order; underlying data storage order remains unchanged.
- Filter operations are independent of sort operations; both can be applied together.
- Users understand basic CLI conventions for providing command arguments.

### Dependencies

- Python 3.13+ runtime environment.
- UV for dependency management (as per project constitution).
- Phase I Basic features (FR-1 through FR-5) are already implemented and functional.
