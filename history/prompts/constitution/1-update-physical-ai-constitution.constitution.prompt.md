---
name: Update Physical AI Constitution
model: claude-sonnet-4-5-20250929
description: Create updated constitution for Physical AI & Humanoid Robotics textbook project with comprehensive principles for accuracy, academic rigor, reproducibility, and RAG chatbot constraints.
---

## Prompt

"""Project

Title: Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World
Type: Unified academic textbook + embedded RAG-based learning assistant
Domain: Physical AI, Robotics, Embodied Intelligence, AI-Native Systems
Audience: Upper-level undergraduate & graduate students (Computer Science / Robotics background)

I. Core Constitutional Principles
1. Accuracy & Verifiability (Non-Negotiable)

Every factual claim must be traceable to a primary or authoritative secondary source

Prefer peer-reviewed research papers, official documentation (ROS 2, NVIDIA Isaac, OpenAI), and standards bodies

No speculative or unverified claims presented as fact

All numerical benchmarks, hardware requirements, and performance claims must cite sources

2. Academic Rigor

Minimum 50% peer-reviewed sources

Citation style: APA (7th edition)

Each chapter must include:

Inline citations

A references section

Claims about learning outcomes, system performance, or architectural superiority require citations

3. Reproducibility & Engineering Fidelity

All technical workflows must be:

Reproducible on Ubuntu 22.04 LTS

Compatible with ROS 2 (Humble or Iron)

Executable on documented hardware (RTX workstations or cloud equivalents)

Code examples must be:

Deterministic where possible

Explicit about versions, dependencies, and configuration

Simulation-to-Real gaps must be explicitly discussed and mitigated

4. Physical-First AI Perspective

Treat AI as an embodied system, not a purely digital artifact

Emphasize:

Physics constraints

Sensor noise

Latency

Actuation limits

Avoid abstract AI explanations detached from real-world robotic constraints

5. Clarity for a Computer Science Audience

Assume strong programming knowledge, but no prior robotics experience

Introduce robotics concepts progressively:

Middleware → Simulation → Perception → Action → Cognition

Maintain Flesch-Kincaid grade level 10–12

Use diagrams, system architecture visuals, and flowcharts wherever possible

II. Book Architecture Standards
6. Modular, Spec-Driven Structure

Each chapter must include:

Learning objectives

Conceptual explanation

System architecture

Practical implementation

Failure modes & limitations

Align chapters with:

ROS 2

Gazebo / Unity

NVIDIA Isaac

Vision-Language-Action pipelines

7. Toolchain Consistency

Documentation platform: Docusaurus

Source control & deployment: GitHub Pages

Specification framework: Spec-Kit Plus

Authoring workflow: Claude Code

All diagrams and code must live inside the repository

8. Zero Plagiarism Policy

0% plagiarism tolerance

No verbatim copying without quotation and citation

Paraphrasing must still include attribution

All AI-generated content must be original and synthesis-based

III. RAG Chatbot Constitution
9. Grounded Responses Only

The embedded chatbot must:

Answer only from the book's indexed content

Explicitly say "Not found in selected text" when applicable

No hallucinations

No external knowledge unless explicitly enabled and cited

10. Selected-Text Constrained Reasoning

When users select text:

Responses must be strictly constrained to that selection

No inference beyond provided context

Clearly differentiate:

Book-wide answers

Selection-only answers

11. Transparent System Design

RAG architecture must be documented:

Embedding model

Vector directory (Qdrant)

Metadata schema

Retrieval strategy

APIs built with FastAPI

Storage via Neon Serverless Postgres

IV. Ethical & Educational Standards
12. Responsible Robotics

Explicit discussion of:

Safety

Human-robot interaction risks

Deployment limitations

Avoid anthropomorphism beyond technical necessity

13. Sim-to-Real Honesty

Clearly state when results are:

Simulated

Synthetic

Real-world validated

Discuss latency, sensor drift, and hardware failures as first-class topics

V. Output & Submission Constraints
14. Formal Requirements

Word count: 5,000–7,000 words

Minimum sources: 15

Final format: PDF

Embedded citations and references

Must pass:

Fact-checking review

Plagiarism detection

 """

## Response snapshot

Created updated constitution file with all specified principles and standards for the Physical AI & Humanoid Robotics textbook project. The new constitution includes sections on accuracy and verifiability, academic rigor, reproducibility, physical-first AI perspective, clarity for CS audience, modular structure, toolchain consistency, zero plagiarism policy, RAG chatbot constraints, ethical standards, and governance procedures.

## Outcome

- ✅ Impact: Updated project constitution with comprehensive principles for Physical AI textbook
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: None needed
- 🧠 Reflection: Successfully transformed template constitution with specific requirements for academic robotics textbook

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A