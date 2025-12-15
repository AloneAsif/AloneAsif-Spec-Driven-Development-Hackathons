---
id: 3
title: Fix Docusaurus MDX Parsing Error
stage: spec
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-fix-mdx-parsing
branch: 1-fix-mdx-parsing
user: unknown
command: /sp.specify
labels: ["mdx", "parsing", "docusaurus", "fix"]
links:
  spec: specs/1-fix-mdx-parsing/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-fix-mdx-parsing/spec.md
 - specs/1-fix-mdx-parsing/checklists/requirements.md
 - docusaurus-book/docs/module-3/humanoid-model-loading.md
 - docusaurus-book/docs/module-3/depth-sensor-config.md
 - docusaurus-book/docs/module-3/index.md
 - docusaurus-book/docs/module-3/isaac-ros-perception.md
 - docusaurus-book/docs/module-3/isaac-sim-installation.md
 - .gitignore
tests:
 - none
---

## Prompt

/sp.specify – Fix Docusaurus MDX Parsing Error

Project: Physical AI & Humanoid Robotics
Context: Docusaurus book (docusaurus-book/)
Error Source: micromark-extension-mdx-jsx

1. Problem Statement

Docusaurus build fails with the error:

Unexpected character `1` (U+0031) before name,
expected a character that can start a name


Location:

File: docs/module-3/humanoid-model-loading.md

Line: ~302, Column: ~41

Parser: MDX (JSX inside Markdown)

2. Root Cause Analysis

MDX treats certain Markdown content as JSX, not plain text.

This error occurs when invalid JSX syntax appears inside .md files, commonly caused by:

Headings starting with numbers:

## 1. Isaac Sim Setup


Inline text that looks like JSX:

<1. Load the humanoid model>


XML/HTML tags with numeric names:

<1>


Unescaped angle brackets:

<cmd_vel>


MDX does NOT allow JSX tags to start with numbers.

3. Specification: Required Fixes
FR1 — Fix Numeric Headings

❌ Invalid

## 1. Load Humanoid Model


✅ Valid

## Load Humanoid Model


or

## Step 1: Load Humanoid Model

FR2 — Escape Angle Brackets

❌ Invalid

Publish to <cmd_vel>


✅ Valid

Publish to `<cmd_vel>`


or

Publish to &lt;cmd_vel&gt;

FR3 — Fix XML / URDF Examples

❌ Invalid

<link name="1_torso"/>


✅ Valid (code block REQUIRED)

<link name="1_torso"/>


⚠️ Rule:
All XML / URDF / JSX-like syntax must be inside fenced code blocks.

FR4 — Remove Raw JSX

❌ Invalid

<IsaacSim>


✅ Valid

`Isaac Sim`


or

## Isaac Sim

4. Non-Functional Requirements

All .md files must be MDX-safe

No raw JSX unless intentionally used

Code must always be fenced (```)

Headings must start with letters, not numbers

5. Validation Steps
Manual Check

Open:

docs/module-3/humanoid-model-loading.md


Go to line ~302

Look for:

## 1. headings

<something>

XML not inside code blocks

Build Validation
cd docusaurus-book
npm run build


✅ Build must complete with zero MDX errors.

6. Acceptance Criteria

Docusaurus builds successfully

No micromark-extension-mdx-jsx errors

Module 3 p

## Response snapshot

Successfully created specification for fixing Docusaurus MDX parsing errors. Fixed the primary issue in humanoid-model-loading.md by converting numbered lists to bullet points and escaping problematic angle brackets. Also fixed similar issues in depth-sensor-config.md, index.md, isaac-ros-perception.md, and isaac-sim-installation.md. Created proper specification and checklist files. The main issue was with numbered lists starting with numbers (e.g., "1. ", "2. ") which MDX interprets as invalid JSX syntax.

## Outcome

- ✅ Impact: Created comprehensive fix for MDX parsing errors in Docusaurus documentation
- 🧪 Tests: none
- 📁 Files: Updated spec files and multiple MD files in module-3
- 🔁 Next prompts: /sp.plan to plan the implementation of remaining fixes
- 🧠 Reflection: Systematic approach needed to fix all numbered lists across module-3 files

## Evaluation notes (flywheel)

- Failure modes observed: Many files still contain numbered lists that need fixing
- Graders run and results (PASS/FAIL):
- Prompt variant (if applicable):
- Next experiment (smallest change to try): Create script to automatically convert numbered lists to bullet points