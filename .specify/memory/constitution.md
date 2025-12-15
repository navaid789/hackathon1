<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: All principles completely redefined based on user requirements
- Added sections: Academic rigor, Reproducibility, Physical-first AI perspective, RAG chatbot constraints, Ethical standards
- Removed sections: Original template placeholders
- Templates requiring updates: ✅ Updated / ⚠ Pending
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World Constitution

## Core Principles

### I. Accuracy & Verifiability (Non-Negotiable)
Every factual claim must be traceable to a primary or authoritative secondary source. Prefer peer-reviewed research papers, official documentation (ROS 2, NVIDIA Isaac, OpenAI), and standards bodies. No speculative or unverified claims presented as fact. All numerical benchmarks, hardware requirements, and performance claims must cite sources.

### II. Academic Rigor
Maintain minimum 50% peer-reviewed sources with APA (7th edition) citation style. Each chapter must include inline citations and a references section. Claims about learning outcomes, system performance, or architectural superiority require citations.

### III. Reproducibility & Engineering Fidelity
All technical workflows must be reproducible on Ubuntu 22.04 LTS and compatible with ROS 2 (Humble or Iron). Executable on documented hardware (RTX workstations or cloud equivalents). Code examples must be deterministic where possible, explicit about versions, dependencies, and configuration. Simulation-to-Real gaps must be explicitly discussed and mitigated.

### IV. Physical-First AI Perspective
Treat AI as an embodied system, not a purely digital artifact. Emphasize physics constraints, sensor noise, latency, and actuation limits. Avoid abstract AI explanations detached from real-world robotic constraints.

### V. Clarity for a Computer Science Audience
Assume strong programming knowledge, but no prior robotics experience. Introduce robotics concepts progressively: Middleware → Simulation → Perception → Action → Cognition. Maintain Flesch-Kincaid grade level 10–12. Use diagrams, system architecture visuals, and flowcharts wherever possible.

### VI. Modular, Spec-Driven Structure
Each chapter must include learning objectives, conceptual explanation, system architecture, practical implementation, and failure modes & limitations. Align chapters with ROS 2, Gazebo / Unity, NVIDIA Isaac, and Vision-Language-Action pipelines.

## Additional Standards

### Toolchain Consistency
Documentation platform: Docusaurus. Source control & deployment: GitHub Pages. Specification framework: Spec-Kit Plus. Authoring workflow: Claude Code. All diagrams and code must live inside the repository.

### Zero Plagiarism Policy
0% plagiarism tolerance. No verbatim copying without quotation and citation. Paraphrasing must still include attribution. All AI-generated content must be original and synthesis-based.

## RAG Chatbot Constitution

### Grounded Responses Only
The embedded chatbot must answer only from the book's indexed content. Explicitly say "Not found in selected text" when applicable. No hallucinations. No external knowledge unless explicitly enabled and cited.

### Selected-Text Constrained Reasoning
When users select text, responses must be strictly constrained to that selection. No inference beyond provided context. Clearly differentiate book-wide answers and selection-only answers.

### Transparent System Design
RAG architecture must be documented including embedding model, vector database (Qdrant), metadata schema, retrieval strategy. APIs built with FastAPI. Storage via Neon Serverless Postgres.

## Ethical & Educational Standards

### Responsible Robotics
Explicit discussion of safety, human-robot interaction risks, and deployment limitations. Avoid anthropomorphism beyond technical necessity.

### Sim-to-Real Honesty
Clearly state when results are simulated, synthetic, or real-world validated. Discuss latency, sensor drift, and hardware failures as first-class topics.

## Output & Submission Constraints

### Formal Requirements
Word count: 5,000–7,000 words. Minimum sources: 15. Final format: PDF. Embedded citations and references. Must pass fact-checking review and plagiarism detection.

## Governance

All contributions must comply with these constitutional principles. Amendments require documentation of rationale and approval by project maintainers. All PRs and reviews must verify compliance with accuracy, academic rigor, and reproducibility standards. This constitution supersedes all other practices and serves as the ultimate authority for project decisions.

**Version**: 1.1.0 | **Ratified**: 2025-12-13 | **Last Amended**: 2025-12-13