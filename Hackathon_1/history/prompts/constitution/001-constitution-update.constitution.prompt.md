---
id: 1
title: Constitution Update
stage: constitution
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: backend
user: claude
command: /sp.constitution
labels: [constitution,rag-chatbot,specifyplus]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

Project: Integrated RAG Chatbot for SpecifyPlus Book

Core principles:
- Accuracy: Responses generated must be factually accurate based on the book's content and selected text.
- Relevance: Answers must directly address user questions using only verified book material.
- Reliability: System must be stable, fast, and handle multiple queries concurrently.
- Reproducibility: Retrieval and generation processes must be traceable and consistent.

Key standards:
- Use OpenAI Agents/ChatKit SDKs for text generation.
- Utilize Qdrant Cloud Free Tier for vector-based storage of book content.
- Neon Serverless Postgres database to store user queries, session history, and metadata.
- FastAPI backend must provide clean, RESTful endpoints for the frontend integration.
- All embeddings and retrievals must prioritize precision and context relevance.
- Security: User queries and stored data must be handled securely with proper encryption.

Constraints:
- The RAG chatbot should only answer questions based on

## Response snapshot

Updated the project constitution to reflect the new "Integrated RAG Chatbot for SpecifyPlus Book" project requirements. The constitution now includes the core principles of Accuracy, Relevance, Reliability, and Reproducibility as specified by the user. Key standards and constraints sections were added to document the technology stack requirements and project constraints. The version was updated from 1.0.0 to 1.1.0 to reflect these changes.

## Outcome

- ✅ Impact: Updated project constitution to align with new RAG chatbot project
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: Continue with project planning and implementation
- 🧠 Reflection: Constitution successfully updated to reflect new project direction

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A