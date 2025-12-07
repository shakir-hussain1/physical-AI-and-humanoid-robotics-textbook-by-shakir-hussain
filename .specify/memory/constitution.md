<!--
=============================================================================
SYNC IMPACT REPORT
=============================================================================
Version Change: 0.0.0 → 1.0.0 (initial constitution)
Bump Rationale: MAJOR - First ratification of project governance

Modified Principles: N/A (new)

Added Sections:
- Core Principles (6 principles)
- Documentation & Content Standards
- Development & Deployment Workflow
- Governance

Removed Sections: None (initial constitution)

Templates Requiring Updates:
- .specify/templates/plan-template.md ✅ No changes needed - Constitution Check
  section references this file generically
- .specify/templates/spec-template.md ✅ No changes needed - template is
  technology-agnostic
- .specify/templates/tasks-template.md ✅ No changes needed - task phases align
  with constitution principles
- .specify/templates/phr-template.prompt.md ✅ No changes needed

Follow-up TODOs: None
=============================================================================
-->

# Physical AI and Humanoid Robotics Constitution

## Core Principles

### I. Technical Accuracy and Correctness

All content MUST be technically accurate, especially regarding robotics frameworks
and platforms:

- **ROS 2**: Nodes, topics, services, actions, and rclpy pipelines MUST follow
  official ROS 2 Jazzy/Humble documentation and best practices
- **Simulation**: Gazebo, Unity, and NVIDIA Isaac Sim configurations MUST be
  runnable and produce consistent, reproducible results
- **Standards**: URDF, SDF, and sensor models MUST conform to official schemas
- **Navigation**: Nav2 path planning and VSLAM implementations MUST use
  documented APIs and parameters

**Rationale**: Robotics software involves safety-critical systems. Inaccurate
instructions can cause hardware damage, unsafe robot behavior, or student
frustration from non-working examples.

### II. Pedagogical Clarity

Content MUST be written for university-level learners in AI, robotics, and
physical computing:

- **Reading level**: Clear for technical learners (grade 10–12 comprehension)
- **Progressive complexity**: Concepts MUST build from fundamentals to advanced
  topics following the 13-week curriculum structure
- **Worked examples**: Each concept MUST include concrete, annotated code
  examples that demonstrate the principle
- **Learning objectives**: Each module and chapter MUST state explicit learning
  outcomes

**Rationale**: A textbook's primary purpose is education. Content that is
technically correct but pedagogically unclear fails its core mission.

### III. Modularity and Structure

Content MUST be cleanly organized into modules, chapters, and weekly learning
paths:

- **Module independence**: Each module (ROS 2, Simulation, Isaac, VLA) MUST be
  self-contained with clear prerequisites stated
- **Chapter structure**: Chapters MUST follow: Introduction → Core Concepts →
  Hands-on Exercises → Summary → Exercises → References
- **Weekly alignment**: All content MUST map to the 13-week academic quarter
  schedule
- **Navigation**: Docusaurus sidebar MUST provide logical progression and
  cross-references

**Rationale**: Modular structure enables instructors to adapt content to their
curricula and allows students to focus on specific topics.

### IV. Reproducibility

All technical steps MUST be executable by students:

- **Environment specifications**: All ROS 2, Gazebo, Isaac Sim, and Unity setups
  MUST include exact version requirements and installation commands
- **Code completeness**: No pseudo-code or partial snippets; all code MUST be
  copy-paste runnable
- **Asset availability**: All URDF models, meshes, and configuration files MUST
  be provided or clearly linked
- **Tested workflows**: Every tutorial MUST be validated on target platforms
  (Ubuntu 22.04, Windows 11 with WSL2 where applicable)

**Rationale**: Students learn robotics by doing. Non-reproducible examples waste
student time and undermine confidence in the material.

### V. Consistency

Definitions, diagrams, terminology, and model names MUST remain uniform:

- **Glossary authority**: All terms MUST be defined in the glossary and used
  consistently across chapters
- **Naming conventions**: Robot models, ROS node names, topic names MUST follow
  a documented naming scheme
- **Diagram style**: All Mermaid/Markdown diagrams MUST use consistent styling
  (colors, shapes, arrows)
- **Version alignment**: Framework versions (ROS 2 Jazzy, Gazebo Harmonic,
  Isaac Sim 4.0+) MUST be consistent throughout

**Rationale**: Inconsistent terminology confuses learners and makes the textbook
appear unprofessional.

### VI. AI-Native Writing Standards

Content produced using Spec-Kit Plus MUST be structured, traceable, and
version-controlled:

- **Feature specs**: Each major content piece MUST have a corresponding spec
  under `specs/<feature>/`
- **Prompt history**: All significant AI-assisted content generation MUST be
  logged via PHR under `history/prompts/`
- **Architectural decisions**: Technology choices (RAG stack, deployment
  platform) MUST be documented via ADRs
- **Original content**: All text MUST be originally generated; no plagiarism

**Rationale**: Traceability enables content auditing, collaborative refinement,
and demonstrates responsible AI-assisted authoring.

## Documentation & Content Standards

### Book Structure Requirements

The textbook MUST include:

- **Preface**: Author introduction, how to use this book, prerequisites
- **Module chapters**: 4 core modules plus capstone per the content outline
- **Weekly curriculum**: 13-week schedule with clear deliverables
- **Assignments**: Graded exercises aligned with learning objectives
- **Capstone walkthrough**: Complete autonomous humanoid robot system
- **Glossary**: Comprehensive terminology reference
- **Appendix**: Installation guides for ROS 2, Gazebo, Isaac Sim, Unity

### Technical Documentation Rules

- **Code blocks**: Use syntax-highlighted fenced code blocks with language tags
- **Diagrams**: Prefer Mermaid for architecture and flow diagrams
- **Commands**: All shell commands MUST specify the working directory and shell
- **File paths**: Use relative paths from project root; indicate OS variations
- **Error handling**: Document common errors and troubleshooting steps

### RAG Chatbot Requirements

The integrated chatbot MUST adhere to:

- **Backend**: FastAPI with async endpoints
- **Database**: Neon Serverless PostgreSQL for user sessions/metadata
- **Vector store**: Qdrant Cloud Free Tier for embeddings
- **Embeddings/LLM**: OpenAI Agents/ChatKit SDKs
- **Grounding**: Answers MUST be sourced strictly from book content
- **Citation**: MUST highlight or retrieve exact text snippets used
- **Modes**: Support both global search and user-selected text querying
- **UI integration**: MUST be embedded within the Docusaurus site

## Development & Deployment Workflow

### Content Development

1. **Spec first**: Create feature spec before writing content
2. **Plan review**: Technical approach reviewed before implementation
3. **Incremental commits**: Commit logical units of work with meaningful messages
4. **Peer review**: Major content changes require review before merge

### Build and Deployment

- **Framework**: Docusaurus static site generator
- **Hosting**: GitHub Pages from `gh-pages` branch
- **CI/CD**: GitHub Actions for build validation and deployment
- **Preview**: Pull request previews for content review

### Quality Gates

- **Markdown linting**: All `.md` files MUST pass linting rules
- **Link validation**: All internal and external links MUST be verified
- **Code testing**: All code examples MUST be syntax-checked
- **Build success**: Docusaurus build MUST complete without errors

## Governance

### Authority

This constitution supersedes all other practices for the Physical AI and
Humanoid Robotics textbook project. Conflicts with external guidelines MUST be
resolved in favor of this constitution.

### Amendment Process

1. Propose amendment via pull request modifying this file
2. Document rationale and impact in PR description
3. Require author (Shakir Hussain) approval for merge
4. Update `LAST_AMENDED_DATE` and increment version per semantic rules

### Version Semantics

- **MAJOR**: Backward-incompatible principle changes or removals
- **MINOR**: New principles, sections, or materially expanded guidance
- **PATCH**: Clarifications, typos, non-semantic refinements

### Compliance

- All PRs MUST verify compliance with these principles
- Content reviewers MUST check Constitution alignment
- Violations MUST be documented and resolved before merge

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
