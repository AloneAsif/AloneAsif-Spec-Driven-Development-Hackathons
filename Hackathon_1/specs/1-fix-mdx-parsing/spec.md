# Feature Specification: Fix Docusaurus MDX Parsing Error

**Feature**: 1-fix-mdx-parsing
**Version**: 1.0.0
**Created**: 2025-12-13
**Status**: Draft
**Author**: Claude

## Overview

Fix Docusaurus build failures caused by MDX parsing errors in the Physical AI & Humanoid Robotics book. The error occurs when invalid JSX syntax appears inside .md files, specifically in docs/module-3/humanoid-model-loading.md at line ~302, column ~41, with the error "Unexpected character `1` (U+0031) before name, expected a character that can start a name".

## Problem Statement

Docusaurus build fails with MDX parsing errors due to invalid JSX syntax in Markdown files. The specific error occurs when MDX encounters content that looks like JSX but doesn't follow proper JSX syntax rules, particularly when headings or tags start with numbers or contain unescaped characters.

## User Scenarios & Testing

### Primary User Scenario
- As a content author working on the Physical AI & Humanoid Robotics book
- I want Docusaurus to build successfully without MDX parsing errors
- So that I can publish and view the documentation properly

### Testing Scenarios
1. **Build Validation**: Running `npm run build` in the docusaurus-book directory completes without MDX errors
2. **Content Rendering**: All documentation pages render correctly in the browser
3. **Syntax Validation**: All Markdown files pass MDX parsing validation
4. **Navigation**: All links and cross-references work correctly after syntax fixes

## Functional Requirements

### FR1: Fix Numeric Headings
**Requirement**: All headings that start with numbers must be restructured to start with letters
- **Acceptance Criteria**:
  - Headings like `## 1. Load Humanoid Model` are changed to `## Load Humanoid Model` or `## Step 1: Load Humanoid Model`
  - All affected files in docs/module-3/ are updated
  - No MDX parsing errors occur for numeric headings

### FR2: Escape Angle Brackets
**Requirement**: All unescaped angle brackets that look like JSX/HTML tags must be properly escaped or formatted
- **Acceptance Criteria**:
  - Text like `<cmd_vel>` is changed to `` `<cmd_vel>` `` or `&lt;cmd_vel&gt;`
  - Angle brackets used in documentation context are properly escaped
  - No MDX parsing errors occur for angle bracket usage

### FR3: Fix XML/URDF Examples
**Requirement**: All XML/URDF content must be properly enclosed in code blocks
- **Acceptance Criteria**:
  - XML/URDF content like `<link name="1_torso"/>` is enclosed in fenced code blocks
  - All URDF examples are properly formatted as code blocks
  - No MDX parsing errors occur for XML/URDF content

### FR4: Remove Raw JSX
**Requirement**: All raw JSX-like syntax must be converted to plain text or proper Markdown
- **Acceptance Criteria**:
  - Tags like `<IsaacSim>` are changed to `Isaac Sim` or `Isaac Sim`
  - All invalid JSX syntax is converted to valid Markdown
  - No MDX parsing errors occur for raw JSX content

## Non-Functional Requirements

### NFR1: MDX Safety
- All .md files must be MDX-safe and parse without errors
- No raw JSX unless intentionally used for valid Docusaurus components

### NFR2: Content Preservation
- All documentation content and meaning must be preserved during the fix
- Technical accuracy of examples and instructions must remain intact

### NFR3: Code Block Integrity
- All code examples must remain properly formatted in fenced code blocks
- Syntax highlighting and language detection must continue to work

## Success Criteria

### Quantitative Measures
- Docusaurus build completes successfully with 0 MDX parsing errors
- All 100+ pages of the Physical AI & Humanoid Robotics book render correctly
- Build time remains under 2 minutes for the entire documentation site

### Qualitative Measures
- All documentation content remains technically accurate and readable
- No loss of information or functionality during the syntax fixes
- Improved documentation quality with proper MDX-compliant syntax

### Validation Criteria
- [ ] `npm run build` in docusaurus-book directory completes without errors
- [ ] All affected files in docs/module-3/ pass MDX validation
- [ ] Content renders correctly in the browser
- [ ] All navigation and cross-references work properly

## Key Entities

### Affected Files
- `docs/module-3/humanoid-model-loading.md` (primary file with error at line ~302)
- Potentially other files in `docs/module-3/` with similar syntax issues

### Error Location
- File: `docs/module-3/humanoid-model-loading.md`
- Line: ~302
- Column: ~41
- Error: "Unexpected character `1` (U+0031) before name, expected a character that can start a name"

## Assumptions

- The error is caused by headings starting with numbers (e.g., `## 1. Isaac Sim Setup`)
- Other similar syntax patterns exist throughout the module-3 documentation
- Converting numeric headings to text-based headings preserves the instructional meaning
- Code examples in XML/URDF format can be properly enclosed in code blocks
- The Docusaurus build process will validate the fixes properly after changes

## Constraints

- All fixes must maintain the educational content and technical accuracy
- Changes should not alter the logical flow or sequence of instructions
- Code examples must remain functional and properly formatted
- Cross-references and internal links must continue to work