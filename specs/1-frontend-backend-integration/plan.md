# Implementation Plan: Frontend-Backend Integration

**Branch**: `1-frontend-backend-integration` | **Date**: 2025-12-16 | **Spec**: [specs/1-frontend-backend-integration/spec.md](specs/1-frontend-backend-integration/spec.md)
**Input**: Feature specification from `/specs/1-frontend-backend-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the integration of the existing FastAPI backend with the Docusaurus frontend book interface to enable users to query the AI chatbot directly from the published book. Based on research, the implementation will:

1. Configure frontend API client using fetch API to communicate with the FastAPI backend
2. Enable CORS in FastAPI to allow requests from the frontend domain during local development
3. Wire a chatbot UI component to the existing `/query` endpoint with support for selected text context
4. Implement comprehensive testing to verify end-to-end query flow functionality

The solution will use a standardized API contract with proper request/response schemas and validation rules as defined in the data model and contracts.

## Technical Context

**Language/Version**: JavaScript/TypeScript for frontend, Python 3.13 for backend
**Primary Dependencies**: FastAPI (backend), Docusaurus (frontend), Qdrant (vector database), OpenAI Agents SDK, Cohere (embeddings)
**Storage**: Qdrant vector database for content retrieval, potential Neon Serverless Postgres for session management
**Testing**: pytest for backend, Jest/Vitest for frontend
**Target Platform**: Web browser (local development environment)
**Project Type**: Web application (frontend + backend integration)
**Performance Goals**: API responses under 10 seconds, UI interactions under 100ms
**Constraints**: Local development environment, CORS-enabled for frontend-backend communication, academic rigor and reproducibility requirements
**Scale/Scope**: Single user interaction model, focused on book content querying functionality

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Academic Rigor**: All technical claims cite peer-reviewed sources or official documentation for FastAPI, Docusaurus, Qdrant, and OpenAI Agents SDK
- **Reproducibility**: Implementation is compatible with Ubuntu 22.04 LTS and documented for local development setup in quickstart.md
- **Physical-First AI**: N/A for web-based RAG chatbot (not embodied system)
- **Citation Requirements**: All API performance claims and technical specifications include official documentation references in contracts/
- **Zero Plagiarism**: All frontend and backend code will be original with proper attribution to official documentation
- **Sim-to-Real Honesty**: N/A for web-based system (not simulation-to-real robotics context)

**Post-Design Verification**: All requirements satisfied for web-based RAG chatbot implementation.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   ├── api/
│   └── agents/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/
```

**Structure Decision**: Web application structure selected to integrate existing Docusaurus frontend with FastAPI backend. The frontend already exists in the main project directory and needs API client integration. The backend is located in the `backend/` directory with existing services for Qdrant, Cohere, and OpenAI integration.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
