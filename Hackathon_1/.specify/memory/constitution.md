<!--
Sync Impact Report:
- Version change: N/A → 1.0.0
- Added principles: Spec-Driven Content Generation, Technical Accuracy, Engineering-Focused Writing, Content Traceability, Docusaurus Book Structure, RAG Chatbot Integration
- Added sections: Additional Constraints, Development Workflow
- Templates requiring updates: ✅ updated (.specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md)
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Spec-Driven Content Generation
All content MUST be generated from Spec-Kit Plus specs with no freehand writing. This ensures consistency, traceability, and maintainability of the book content. Content creation follows the spec-driven development approach where specifications precede implementation.

### II. Technical Accuracy
All technical content MUST maintain high accuracy, particularly regarding ROS 2, Gazebo, Isaac, VLA, and related robotics technologies. Technical descriptions must be precise, up-to-date, and reflect current best practices in the field.

### III. Engineering-Focused Writing
All content MUST follow clear, engineering-focused writing principles. Documentation should be concise, technically precise, and focused on practical implementation rather than theoretical concepts alone.

### IV. Content Traceability
Content MUST be traceable to module specifications. Each chapter maps to specific module specs ensuring clear connections between learning objectives and delivered content. This enables systematic updates and quality assurance.

### V. Docusaurus Book Structure
The book MUST follow a Docusaurus-based structure with 4 modules, each containing 2–3 chapters. This ensures consistent organization, navigation, and user experience across the entire book.

### VI. RAG Chatbot Integration
The RAG chatbot MUST be integrated into the Docusaurus site using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, and Qdrant. The chatbot MUST NOT hallucinate and MUST respond with "Not found in book" when information is not available in the source materials.

## Additional Constraints

Technology Stack Requirements:
- Frontend: Docusaurus for book presentation
- Backend: FastAPI for API services
- Database: Neon Postgres for data storage
- Vector Database: Qdrant for RAG functionality
- AI Integration: OpenAI Agents/ChatKit for chatbot functionality

Module Themes (must appear in book):
- ROS 2 — nodes, topics, services, URDF
- Simulation — Gazebo physics, Unity visuals, LiDAR
- AI for Robotics — perception, planning, control
- Humanoid Systems — locomotion, manipulation, control

## Development Workflow

- Spec-Driven Development: All features and content must be specified before implementation
- Module-based Structure: Development organized around 4 core modules
- Quality Assurance: All content must pass technical review and accuracy verification
- Integration Testing: RAG functionality must be thoroughly tested with book content
- Version Control: All changes tracked with clear commit messages following conventional format

## Governance

This constitution supersedes all other development practices for this project. All development activities must comply with these principles. Amendments to this constitution require explicit documentation, team approval, and migration planning. All pull requests and code reviews must verify compliance with these principles. All contributors must acknowledge and adhere to this constitution when participating in the project.

**Version**: 1.0.0 | **Ratified**: 2025-12-11 | **Last Amended**: 2025-12-11
