<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: Updated from Physical AI & Humanoid Robotics to Integrated RAG Chatbot for SpecifyPlus Book
- Added principles: Accuracy, Relevance, Reliability, Reproducibility
- Added sections: Key Standards, Constraints
- Templates requiring updates: ✅ updated (.specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md)
- Follow-up TODOs: None
-->

# Integrated RAG Chatbot for SpecifyPlus Book Constitution

## Core Principles

### I. Accuracy
Responses generated MUST be factually accurate based on the book's content and selected text. This ensures that users receive reliable information that directly corresponds to the source material without misrepresentation or fabrication.

### II. Relevance
Answers MUST directly address user questions using only verified book material. The system must prioritize delivering responses that are directly connected to the user's inquiry and grounded in the provided book content rather than offering tangential information.

### III. Reliability
System MUST be stable, fast, and handle multiple queries concurrently. The infrastructure must maintain consistent performance under varying loads and provide dependable service availability for users seeking information from the book content.

### IV. Reproducibility
Retrieval and generation processes MUST be traceable and consistent. Users and developers must be able to understand how answers were derived and reproduce the same results under identical conditions for debugging and validation purposes.

## Key Standards

Technology Stack Requirements:
- Text Generation: OpenAI Agents/ChatKit SDKs for generating responses
- Vector Storage: Qdrant Cloud Free Tier for vector-based storage of book content
- Database: Neon Serverless Postgres to store user queries, session history, and metadata
- Backend Framework: FastAPI to provide clean, RESTful endpoints for frontend integration
- Embeddings and Retrieval: Prioritize precision and context relevance in all retrieval operations
- Security: User queries and stored data must be handled securely with proper encryption

## Constraints

- The RAG chatbot MUST only answer questions based on the book's content and selected text
- The system MUST NOT hallucinate or fabricate information not present in the source material
- All responses MUST be grounded in verified book content with appropriate citations where possible
- The chatbot MUST respond with "Not found in book" when information is not available in the source materials
- All data handling MUST comply with security and privacy requirements specified in Key Standards

## Development Workflow

- Spec-Driven Development: All features and content must be specified before implementation
- Quality Assurance: All content must pass technical review and accuracy verification
- Integration Testing: RAG functionality must be thoroughly tested with book content
- Version Control: All changes tracked with clear commit messages following conventional format
- Compliance Verification: All implementations must be validated against constitutional principles

## Governance

This constitution supersedes all other development practices for this project. All development activities must comply with these principles. Amendments to this constitution require explicit documentation, team approval, and migration planning. All pull requests and code reviews must verify compliance with these principles. All contributors must acknowledge and adhere to this constitution when participating in the project.

**Version**: 1.1.0 | **Ratified**: 2025-12-11 | **Last Amended**: 2025-12-18
