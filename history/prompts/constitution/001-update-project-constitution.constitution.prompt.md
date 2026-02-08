---
id: 001
title: Update project constitution
stage: constitution
date: 2026-02-04
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: Asif
command: sp.constitution
labels: [constitution, governance, principles]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:
  - None
---

## Prompt

Project: Phase II: Todo Full-Stack Web Application
Core principles:

Spec-driven development: All features and implementations must originate from and adhere to structured specifications in the /specs directory
Automated implementation: No manual coding; use Claude Code exclusively for generating and editing code based on specs
Monorepo integration: Maintain a single repository for frontend, backend, and specs to enable seamless cross-cutting changes and context
User security and isolation: Enforce JWT-based authentication to ensure users only access their own data
Technology adherence: Strictly use the defined stack for frontend (Next.js 16+ App Router), backend (Python FastAPI with SQLModel), database (Neon Serverless PostgreSQL), and authentication (Better Auth with JWT)
Key standards:
Spec organization: Follow Spec-Kit conventions with specs in /specs/overview.md, /specs/features/, /specs/api/, /specs/database/, and /specs/ui/
Referencing: Always reference specs using @specs/[path]/[file].md in Claude Code prompts and implementations
API design: Implement RESTful endpoints exactly as defined (e.g., GET /api/{user_id}/tasks, POST /api/{user_id}/tasks) with JWT verification in headers
Database integrity: Use SQLModel for ORM, with schema including users (managed by Better Auth) and tasks table with user_id foreign key, indexes for user_id and completed
Frontend patterns: Use server components by default, client components only for interactivity, Tailwind CSS for styling, and /lib/api.ts for API calls
Backend conventions: Routes under /api/, Pydantic models for requests/responses, HTTPException for errors, DATABASE_URL from environment
Authentication flow: Enable JWT plugin in Better Auth, attach tokens to API headers on frontend, verify on backend with shared BETTER_AUTH_SECRET
Constraints:
No additional technologies: Do not introduce libraries or tools outside the specified stack (e.g., no alternative ORMs, auth libraries, or frameworks)
Monorepo structure: Adhere to hackathon-todo/ with .spec-kit/config.yaml, specs/, CLAUDE.md, frontend/, backend/, docker-compose.yml
Feature scope: Implement only the 5 basic level features (task CRUD: create, read, update, delete, toggle complete) as a multi-user web app
Environment variables: Use BETTER_AUTH_SECRET for JWT, DATABASE_URL for Neon PostgreSQL connection
Testing: Ensure all operations enforce user ownership; no shared data access across users
Success criteria:
All API endpoints functional and secured: Return 401 for unauthorized requests, filter data by authenticated user_id
Responsive frontend: Built with Next.js, displaying tasks with CRUD operations, filtering/sorting, and authentication UI
Data persistence: Tasks stored and retrieved correctly from Neon PostgreSQL, with automatic timestamps and indexes
Full authentication: User signup/signin working, JWT issuance and verification across frontend/backend
Spec compliance: All implemented code traceable to specs; no deviations without spec updates
Operational: Runs via docker-compose up, with separate dev commands

## Response snapshot

I have successfully updated the project constitution with the provided principles and requirements. Here's what was accomplished:

1. Updated the constitution file at `.specify/memory/constitution.md` with the complete set of core principles, key standards, constraints, and success criteria.

2. Updated the plan template at `.specify/templates/plan-template.md` to include constitution compliance checks.

3. Updated the spec template at `.specify/templates/spec-template.md` to include constitution compliance requirements.

4. Updated the tasks template at `.specify/templates/tasks-template.md` to include constitution compliance requirements.

The new constitution document includes:
- Five core principles covering spec-driven development, automated implementation, monorepo integration, user security, and technology adherence
- Key standards for spec organization, referencing, API design, database integrity, frontend patterns, backend conventions, and authentication flow
- Constraints on technology usage, monorepo structure, feature scope, environment variables, and testing
- Success criteria for API endpoints, frontend, data persistence, authentication, spec compliance, and operational aspects

## Outcome

- ✅ Impact: Major update to project governance with comprehensive constitution and template alignments
- 🧪 Tests: None required for this configuration update
- 📁 Files: 4 files updated to maintain consistency with new constitution
- 🔁 Next prompts: Ready to proceed with spec-driven development using the new constitution
- 🧠 Reflection: The constitution now provides clear governance framework for the todo full-stack web application project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All templates updated consistently
- Prompt variant (if applicable): Standard constitution update
- Next experiment (smallest change to try): Begin feature specification based on new constitution