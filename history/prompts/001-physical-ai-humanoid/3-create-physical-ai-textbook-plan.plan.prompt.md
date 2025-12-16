---
name: Create Physical AI Textbook Plan
model: claude-sonnet-4-5-20250929
description: Create comprehensive implementation plan for Physical AI & Humanoid Robotics textbook with research, data model, and API contracts.
---

## Prompt

Project

Title: Physical AI & Humanoid Robotics: Embodied Intelligence from Simulation to Reality

Output Type: AI/Spec-driven academic textbook
Delivery Platform: Docusaurus → GitHub Pages
Scope: Physical AI, Humanoid Robotics, Vision–Language–Action systems

Create
1. Architecture Sketch

Produce a clear, system-level architecture describing:

Content Architecture

Docusaurus site structure

Chapter-to-sidebar mapping

Version-controlled Markdown sources

Knowledge Architecture

Concept flow: Physical World → Middleware → Simulation → Perception → Action → Cognition

Dependency graph between chapters (prerequisites)

Tooling Architecture

Claude Code for spec-driven writing

Spec-Kit Plus for structure enforcement

GitHub Pages for deployment

(Forward-compatible) RAG chatbot integration points (content chunking, metadata boundaries)

2. Section & Chapter Structure

Define a phase-aligned chapter plan mapped to learning progression:

Phase 1 – Research

Physical AI & embodied intelligence foundations

Humanoid robotics literature

ROS 2, Gazebo, NVIDIA Isaac primary sources

Phase 2 – Foundation

Sensors, perception, ROS 2 middleware

Robot description formats (URDF/SDF)

Physics simulation fundamentals

Phase 3 – Analysis

VSLAM, navigation, reinforcement learning

Sim-to-real transfer challenges

Hardware and infrastructure tradeoffs

Phase 4 – Synthesis

Vision–Language–Action pipelines

Conversational robotics

End-to-end autonomous humanoid capstone

Future outlook

Each chapter must specify:

Learning objectives

Key claims requiring citations

Diagrams or architecture figures

Reproducibility notes

3. Research Approach

Follow a research-concurrent writing model:

Research and writing happen in parallel

Sources are added incrementally per chapter

Claims are written only after at least one supporting source is identified

Prioritize:

Peer-reviewed robotics and AI journals

Official ROS, NVIDIA Isaac, OpenAI documentation

Maintain a living references file (APA format)

4. Quality Validation Plan

Define validation gates at multiple levels:

Content Validation

All factual claims cited

≥50% peer-reviewed sources

Minimum 15 total sources

Clarity Validation

Flesch-Kincaid grade 10–12

Terminology consistency

Reproducibility Validation

Explicit versions (ROS 2, Ubuntu, Isaac)

Hardware assumptions clearly stated

Ethical & Safety Validation

Clear distinction between simulation and real-world deployment

No unsafe real-world instructions without disclaimers

Decisions Needing Documentation

Document key architectural and pedagogical decisions, including:

Simulation-first vs hardware-first learning

Options: Full simulation | Early hardware exposure

Tradeoff: Accessibility vs realism

Humanoid vs proxy robots

Options: Biped humanoid | Quadruped/arm proxy

Tradeoff: Cost vs fidelity

On-prem GPU lab vs cloud infrastructure

Options: RTX workstations | Cloud GPU instances

Tradeoff: CapEx vs OpEx, latency

ROS 2 Python vs C++ focus

Options: Python-first | Mixed-language

Tradeoff: Accessibility vs performance

Depth of mathematical treatment

Options: Conceptual | Equation-heavy

Tradeoff: CS accessibility vs robotics rigor

Testing Strategy

Define tests aligned with acceptance criteria:

Structural Tests

All chapters present and linked in Docusaurus sidebar

Navigation builds successfully on GitHub Pages

Academic Tests

Citation completeness check

APA formatting validation

Plagiarism scan (0% tolerance)

Technical Consistency Tests

ROS 2 terminology accuracy

Simulation stack coherence (Gazebo ↔ Isaac ↔ Unity)

Hardware requirements internally consistent

Acceptance Criteria Validation

Reader can explain:

Physical AI vs digital AI

ROS 2-based robot control

Sim-to-real transfer

Vision–Language–Action systems

Capstone described end-to-end with no conceptual gaps

Technical Details

Authoring: Spec-driven writing using Claude Code

Documentation: Docusaurus Markdown

Deployment: GitHub Pages

Citation Style: APA (as defined in Constitution)

Workflow:

## Response snapshot

Completed comprehensive planning for the Physical AI & Humanoid Robotics textbook project. Created:

1. Implementation plan (plan.md) with technical context, constitution checks, and project structure
2. Research document (research.md) with technology decisions, architectural choices, and validation approaches
3. Data model (data-model.md) defining content entities, academic entities, and platform entities
4. Quickstart guide (quickstart.md) with setup instructions and development workflow
5. API contract for RAG chatbot (contracts/rag-chatbot-api.yaml) with proper OpenAPI specification

All artifacts align with the constitution requirements for academic rigor, reproducibility, and ethical standards. The plan addresses the 4-phase approach (Research → Foundation → Analysis → Synthesis) and includes all required quality validation gates.

## Outcome

- ✅ Impact: Comprehensive planning completed for Physical AI textbook project
- 🧪 Tests: none
- 📁 Files: specs/001-physical-ai-humanoid/plan.md, research.md, data-model.md, quickstart.md, contracts/rag-chatbot-api.yaml
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Successfully created complete planning artifacts with proper academic rigor and technical architecture

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A