# Feature Specification: Remove Docusaurus Dummy Content

**Feature Branch**: `001-clean-docusaurus-dummy`
**Created**: 2025-12-12
**Status**: Draft
**Input**: User description: "/sp.specify – Remove Docusaurus Dummy Content

Goal:
Clean the Docusaurus project by removing all default example content, including:

Blog

Example "tutorial" docs

"Extras" sidebar

Demo pages (intro.md, create-a-blog-post.md, etc.)

1. Functional Requirements

FR1: Remove the entire blog/ directory.
FR2: Remove default example docs inside docs/ (Docusaurus tutorial).
FR3: Clean sidebars.js so it includes only your book sections.
FR4: Remove landing-page dummy text and replace with your book intro.

2. Files/Directories to Delete

blog/

docs/tutorial-basics/

docs/tutorial-extras/

Docusaurus sample markdown:

docs/intro.md (replace with your own intro)

Any auto-generated sample .md with placeholders.

3. Required File Updates
A. Clean sidebars.js

Remove all default categories and keep only:

module.exports = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: ['module-1/intro-to-ros2','module-1/ros2-python-agents','module-1/urdf-basics"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Clean Docusaurus Project Structure (Priority: P1)

As a developer working on the Physical AI & Humanoid Robotics book, I want to remove all default Docusaurus example content so that I can have a clean project structure focused on my book content.

**Why this priority**: This is foundational work that needs to be completed before I can properly organize my book content without distraction from example content.

**Independent Test**: The Docusaurus site should only display the book content without any default tutorial pages, blog posts, or example sections.

**Acceptance Scenarios**:

1. **Given** a Docusaurus project with default example content, **When** the cleaning process is completed, **Then** only the book content remains in the docs directory
2. **Given** a Docusaurus project with default example content, **When** the cleaning process is completed, **Then** the sidebar only shows book-related sections

---

### User Story 2 - Remove Default Blog Content (Priority: P1)

As a developer, I want to remove the default blog directory so that the site focuses on book content rather than example blog posts.

**Why this priority**: The blog content is not relevant to the book and creates unnecessary clutter in the project structure.

**Independent Test**: The blog/ directory should no longer exist in the project after the cleaning process.

**Acceptance Scenarios**:

1. **Given** a Docusaurus project with default blog content, **When** the cleaning process is completed, **Then** the blog/ directory is completely removed

---

### User Story 3 - Update Navigation Structure (Priority: P1)

As a reader of the book, I want to see only relevant navigation items so that I can easily find the book content without being distracted by example pages.

**Why this priority**: Proper navigation is essential for user experience and helps readers focus on the intended content.

**Independent Test**: The sidebar navigation should only contain book-related sections and links.

**Acceptance Scenarios**:

1. **Given** a Docusaurus project with default navigation, **When** the cleaning process is completed, **Then** the sidebar.js file only contains references to the book content
2. **Given** a Docusaurus project with default navigation, **When** the cleaning process is completed, **Then** the main intro page is updated with book-specific content

---

### Edge Cases

- What happens if the blog/ directory doesn't exist? The system should continue with other cleanup tasks.
- What if there are custom modifications to the default content? The system should preserve those while removing standard default content.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST remove the entire blog/ directory
- **FR-002**: System MUST remove default example docs inside docs/ (tutorial-basics, tutorial-extras)
- **FR-003**: System MUST clean sidebars.js to include only book sections
- **FR-004**: System MUST update the main intro.md page to reference the book content
- **FR-005**: System MUST remove any auto-generated sample markdown files with placeholders
- **FR-006**: System MUST preserve existing book content in the module-1 directory

### Key Entities *(include if feature involves data)*

- **Docusaurus Project Structure**: The directory and file organization of the Docusaurus documentation site
- **Navigation Sidebar**: The configuration that defines the site's navigation structure
- **Content Pages**: The markdown files containing the book content

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The blog/ directory is completely removed from the project
- **SC-002**: The tutorial-basics/ and tutorial-extras/ directories are completely removed
- **SC-003**: The sidebar.js file only contains references to book content (intro and module-1 sections)
- **SC-004**: The main intro.md page contains book-specific content rather than default Docusaurus content
- **SC-005**: The Docusaurus site builds successfully with only book-related navigation
- **SC-006**: All existing book content in module-1 remains accessible and functional
