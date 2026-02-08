<!--
Sync Impact Report:
Version change: N/A → 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections as this is initial constitution
Removed sections: N/A
Templates requiring updates:
- ✅ .specify/templates/plan-template.md (updated)
- ✅ .specify/templates/spec-template.md (updated)
- ✅ .specify/templates/tasks-template.md (updated)
- ✅ .specify/templates/commands/sp.constitution.md (updated)
Templates updated to reflect new principles and constraints
-->

# Phase II: Todo Full-Stack Web Application Constitution

## Core Principles

### Spec-driven development
All features and implementations must originate from and adhere to structured specifications in the /specs directory. Code generation and modifications must be based on formal specs maintained in the /specs directory, ensuring traceability between requirements and implementation.

### Automated implementation
No manual coding; use Claude Code exclusively for generating and editing code based on specs. All code creation, modification, and refactoring must be performed through Claude Code using formal specifications as input, eliminating ad-hoc manual changes.

### Monorepo integration
Maintain a single repository for frontend, backend, and specs to enable seamless cross-cutting changes and context. All project components must coexist in a unified repository structure to facilitate coordinated development and holistic project visibility.

### User security and isolation
Enforce JWT-based authentication to ensure users only access their own data. Implement strict data access controls that prevent users from viewing or modifying other users' data, with all operations requiring proper authentication and authorization.

### Technology adherence
Strictly use the defined stack for frontend (Next.js 16+ App Router), backend (Python FastAPI with SQLModel), database (Neon Serverless PostgreSQL), and authentication (Better Auth with JWT). No additional technologies may be introduced without explicit approval and specification updates.

### API design compliance
Implement RESTful endpoints exactly as defined with JWT verification in headers. All API endpoints must follow established patterns with proper authentication middleware and consistent request/response structures.

## Key Standards

### Spec organization
Follow Spec-Kit conventions with specs in /specs/overview.md, /specs/features/, /specs/api/, /specs/database/, and /specs/ui/. Specifications must be organized hierarchically with clear relationships between different aspects of the application.

### Referencing
Always reference specs using @specs/[path]/[file].md in Claude Code prompts and implementations. All code implementations must include clear traceability to corresponding specification files to maintain consistency.

### Database integrity
Use SQLModel for ORM, with schema including users (managed by Better Auth) and tasks table with user_id foreign key, indexes for user_id and completed. Ensure proper data modeling with appropriate constraints and indexing for performance.

### Frontend patterns
Use server components by default, client components only for interactivity, Tailwind CSS for styling, and /lib/api.ts for API calls. Maintain clear separation between presentation and business logic with consistent styling patterns.

### Backend conventions
Routes under /api/, Pydantic models for requests/responses, HTTPException for errors, DATABASE_URL from environment. Implement consistent error handling and response patterns across all API endpoints.

### Authentication flow
Enable JWT plugin in Better Auth, attach tokens to API headers on frontend, verify on backend with shared BETTER_AUTH_SECRET. Ensure secure token handling and validation across frontend and backend components.

## Constraints

### No additional technologies
Do not introduce libraries or tools outside the specified stack (e.g., no alternative ORMs, auth libraries, or frameworks). Technology choices are strictly limited to the predefined stack to maintain consistency and reduce complexity.

### Monorepo structure
Adhere to hackathon-todo/ with .spec-kit/config.yaml, specs/, CLAUDE.md, frontend/, backend/, docker-compose.yml. Maintain the established directory structure and configuration files to ensure consistent project layout.

### Feature scope
Implement only the 5 basic level features (task CRUD: create, read, update, delete, toggle complete) as a multi-user web app. No additional features may be implemented without explicit specification updates.

### Environment variables
Use BETTER_AUTH_SECRET for JWT, DATABASE_URL for Neon PostgreSQL connection. All configuration must be managed through environment variables with proper secrets handling.

### Testing
Ensure all operations enforce user ownership; no shared data access across users. Implement proper testing strategies that verify data isolation and authentication flows.

## Success Criteria

### All API endpoints functional and secured
Return 401 for unauthorized requests, filter data by authenticated user_id. All API endpoints must properly authenticate users and enforce data access controls.

### Responsive frontend
Built with Next.js, displaying tasks with CRUD operations, filtering/sorting, and authentication UI. The user interface must provide intuitive access to all required functionality with responsive design.

### Data persistence
Tasks stored and retrieved correctly from Neon PostgreSQL, with automatic timestamps and indexes. Database operations must be reliable with proper data integrity and performance characteristics.

### Full authentication
User signup/signin working, JWT issuance and verification across frontend/backend. Authentication system must function seamlessly across all application components.

### Spec compliance
All implemented code traceable to specs; no deviations without spec updates. Every code change must correspond to an approved specification modification.

### Operational
Runs via docker-compose up, with separate dev commands. The application must be deployable and operable using the defined container orchestration setup.

## Governance

All development activities must comply with these constitutional principles. Any proposed changes to these principles require formal amendment procedures with stakeholder approval. Code reviews must verify compliance with all constitutional requirements before merging. Violations of constitutional principles must be addressed before feature completion.

**Version**: 1.0.0 | **Ratified**: 2026-02-04 | **Last Amended**: 2026-02-04
