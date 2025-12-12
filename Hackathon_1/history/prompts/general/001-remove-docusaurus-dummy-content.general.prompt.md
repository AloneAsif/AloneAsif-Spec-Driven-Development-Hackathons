---
id: "001"
title: "Remove Docusaurus Dummy Content"
stage: "general"
date: "2025-12-12"
model: "sonnet"
feature: "none"
branch: "001-clean-docusaurus-dummy"
user: "user"
command: "/sp.specify – Remove Docusaurus Dummy Content"
labels: ["cleanup", "docusaurus", "documentation"]
links:
  spec: "specs/001-clean-docusaurus-dummy/spec.md"
  ticket: null
  adr: null
  pr: null
files: []
tests: []
---

# Remove Docusaurus Dummy Content

## User Input

```
/sp.specify – Remove Docusaurus Dummy Content

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
      items: ['module-1/intro-to-ros2','module-1/ros2-python-agents','module-1/urdf-basics
```

## Response Summary

Successfully implemented the feature to clean the Docusaurus project by removing all default example content. The following actions were completed:

1. **Removed directories**:
   - Deleted the entire `blog/` directory
   - Deleted `docs/tutorial-basics/` directory
   - Deleted `docs/tutorial-extras/` directory

2. **Updated configuration files**:
   - Modified `sidebars.js` to only include the intro and module-1 sections as specified
   - Updated `docusaurus.config.js` to disable the blog plugin and remove blog references from navbar and footer

3. **Preserved existing content**:
   - All module-1 book content was preserved
   - The intro.md file was already correctly updated with book-specific content

4. **Created specification**:
   - Generated a complete feature specification in `specs/001-clean-docusaurus-dummy/spec.md`
   - Created a quality checklist at `specs/001-clean-docusaurus-dummy/checklists/requirements.md`

The Docusaurus project is now clean with only the Physical AI & Humanoid Robotics book content remaining, and the navigation structure properly reflects the book modules without any default Docusaurus example content.