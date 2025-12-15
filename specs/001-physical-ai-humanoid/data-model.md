# Data Model: Physical AI & Humanoid Robotics Textbook

## Content Entities

### Chapter
- **Fields**: id, title, number, learning_objectives, key_claims, diagrams, reproducibility_notes, prerequisites, citations_count, peer_reviewed_sources_count
- **Relationships**: belongs_to Part, has_many Sections, has_many Figures, has_many Citations
- **Validation**: Must have 1+ learning objectives, 1+ key claims requiring citations, 1+ diagrams or architecture figures
- **State transitions**: Draft → Review → Approved → Published

### Section
- **Fields**: id, title, content, chapter_id, section_number, learning_outcomes
- **Relationships**: belongs_to Chapter, has_many Subsections
- **Validation**: Must be part of a valid chapter, section_number must follow hierarchical structure

### Figure
- **Fields**: id, title, description, file_path, type (diagram/architecture/equation), chapter_id
- **Relationships**: belongs_to Chapter
- **Validation**: Must have alt text for accessibility, must be referenced in content

### Citation
- **Fields**: id, reference, type (peer_reviewed/official_documentation/other), url, chapter_id, is_verified
- **Relationships**: belongs_to Chapter
- **Validation**: Must be of peer_reviewed or official_documentation type to meet academic rigor requirement

## Academic Entities

### LearningObjective
- **Fields**: id, description, chapter_id, measurable_outcome
- **Relationships**: belongs_to Chapter
- **Validation**: Must be measurable and achievable

### TechnicalConcept
- **Fields**: id, name, definition, chapter_id, prerequisites, applications
- **Relationships**: belongs_to Chapter, has_many RelatedConcepts
- **Validation**: Must have clear definition and real-world application

## Quality Assurance Entities

### QualityGate
- **Fields**: id, type (content/clarity/reproducibility/ethical), status, chapter_id, notes
- **Validation**: Must pass all gates before publication

### ValidationRule
- **Fields**: id, description, requirement_type, threshold, test_method
- **Examples**:
  - Citation completeness: ≥50% peer-reviewed sources
  - Readability: Flesch-Kincaid grade 10-12
  - Plagiarism: 0% tolerance

## Platform Entities

### DocusaurusPage
- **Fields**: id, slug, title, markdown_content, sidebar_position, previous_id, next_id
- **Relationships**: belongs_to Chapter
- **Validation**: Must have valid slug, proper navigation links

### NavigationNode
- **Fields**: id, title, type (doc/category), doc_id, children, sidebar_position
- **Validation**: Must form valid hierarchical structure without cycles

## RAG Integration Entities

### ContentChunk
- **Fields**: id, content, chapter_id, section_id, metadata, embedding_ready
- **Relationships**: belongs_to Chapter
- **Validation**: Must preserve context and meaning when chunked

### MetadataBoundary
- **Fields**: id, type, start_position, end_position, content_chunk_id
- **Validation**: Must preserve citation and attribution information

## Hardware/Software Requirements

### SystemRequirement
- **Fields**: id, component_type, minimum_spec, recommended_spec, chapter_id, justification
- **Examples**:
  - GPU: RTX 2080 minimum, RTX 3080 recommended for Isaac Sim
  - OS: Ubuntu 22.04 LTS required
  - RAM: 32GB minimum for simulation workloads

### SoftwareDependency
- **Fields**: id, name, version, installation_method, chapter_id, compatibility_notes
- **Examples**:
  - ROS 2: Humble Hawksbill (LTS)
  - Isaac Sim: 2023.1+
  - Gazebo: Garden