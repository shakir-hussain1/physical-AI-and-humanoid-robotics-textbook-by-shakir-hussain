---
id: 001
title: Physical AI Textbook Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: Shakir Hussain
command: /sp.constitution
labels: ["constitution", "governance", "physical-ai", "robotics", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

Project: Textbook — "Physical AI and Humanoid Robotics"
Author: Shakir Hussain

Core Objectives:
1. Create a complete, structured textbook titled "Physical AI and Humanoid Robotics".
2. Build the book using Docusaurus and deploy it to GitHub Pages.
3. Use Spec-Kit Plus and Claude Code for book generation, content structuring, and automation.
4. Develop and integrate a RAG (Retrieval-Augmented Generation) chatbot inside the Docusaurus site.
   - Chatbot must answer questions strictly from the book's content.
   - Must support user-selected text querying.
   - Must use OpenAI Agents/ChatKit SDKs, FastAPI backend, Neon Serverless Postgres, and Qdrant Cloud Free Tier.

---

Core Principles:
- **Accuracy and technical correctness**, especially regarding robotics frameworks (ROS 2, Gazebo, Unity, NVIDIA Isaac, Nav2, URDF/SDF).
- **Pedagogical clarity** for university-level learners in AI, robotics, and physical computing.
- **Modularity**: content must be cleanly structured into modules, chapters, and weekly learning paths.
- **Reproducibility**: all technical steps (ROS setups, Gazebo configs, Isaac pipelines) must be runnable by students.
- **Consistency**: definitions, diagrams, terminology, and model names must remain uniform across chapters.
- **AI-native writing standards**: content produced using Spec-Kit Plus must be structured, traceable, and version-controlled.

---

Content Requirements:
The book must cover the following themes and modules in clear, structured textbook format:

1. **Introduction to Physical AI**
   - Embodied intelligence principles
   - Digital brain → physical robot transition
   - Why humanoid form factors matter

2. **Module 1: The Robotic Nervous System (ROS 2)**
   - Nodes, topics, services, actions
   - Python (rclpy) control pipelines
   - URDF for humanoid robots

3. **Module 2: Digital Twin Simulation**
   - Physics simulation using Gazebo
   - Visual simulation using Unity
   - Sensor simulation (LiDAR, depth, IMUs)

4. **Module 3: The AI-Robot Brain (NVIDIA Isaac)**
   - Isaac Sim for photorealistic simulation
   - Isaac ROS for VSLAM, navigation, perception
   - Nav2 path planning for bipedal locomotion

5. **Module 4: Vision–Language–Action (VLA Systems)**
   - Voice-to-Action with Whisper
   - Natural language → ROS 2 task planning using LLMs
   - Conversational robotics

6. **Capstone: Autonomous Humanoid Robot System**
   - Takes a voice command
   - Plans motions
   - Navigates obstacles
   - Performs object identification
   - Executes manipulation

7. **Weekly Breakdown (13-week academic quarter)**
   - Weeks 1–2: Foundations of Physical AI
   - Weeks 3–5: ROS 2 fundamentals
   - Weeks 6–7: Gazebo simulation
   - Weeks 8–10: NVIDIA Isaac platform
   - Weeks 11–12: Humanoid robot mechanics
   - Week 13: Conversational robotics with GPT models

---

Documentation Standards:
- Writing level: Clear for technical learners (grade 10–12 comprehension).
- Diagrams: Use Mermaid/Markdown diagrams where appropriate.
- Code: All code must be runnable, tested, and version-controlled.
- No plagiarism; all text must be originally generated.

---

Book Output Requirements:
- Format: Docusaurus site with sidebar navigation.
- Deployment: GitHub Pages.
- Must include:
  - **Preface**
  - **Module chapters**
  - **Weekly curriculum**
  - **Assignments**
  - **Capstone walkthrough**
  - **Glossary**
  - **Appendix: Installation guides for ROS 2, Gazebo, Isaac Sim, Unity**

---

RAG Chatbot Requirements:
- Backend: FastAPI
- Database: Neon Serverless PostgreSQL
- Vector DB: Qdrant Cloud (Free Tier)
- Embeddings + chat logic: OpenAI Agents / ChatKit SDKs
- Capabilities:
  - Answer questions based solely on book content.
  - Highlight or retrieve the exact text snippets used.
  - Offer both global search and "user-selected text only" mode.
- Must integrate into the Docusaurus UI.

---

Success Criteria:
- The complete textbook is generated, validated, and deployable.
- Fully working Docusaurus site hosted on GitHub Pages.
- RAG chatbot functions reliably with accurate grounded answers.
- All modules and weekly instruction match the Physical AI curriculum.
- Code examples complete and runnable.

## Response snapshot

Created the initial constitution for the "Physical AI and Humanoid Robotics" textbook project at `.specify/memory/constitution.md` (v1.0.0).

**Six Core Principles Established:**
1. **Technical Accuracy and Correctness** - ROS 2, Gazebo, Isaac Sim, URDF/SDF must follow official documentation
2. **Pedagogical Clarity** - Grade 10-12 reading level, progressive complexity, worked examples
3. **Modularity and Structure** - Self-contained modules, consistent chapter structure, 13-week alignment
4. **Reproducibility** - Copy-paste runnable code, version-specified environments, tested workflows
5. **Consistency** - Glossary authority, naming conventions, diagram style guidelines
6. **AI-Native Writing Standards** - Feature specs, PHR logging, ADR documentation

**Additional Sections:**
- Documentation & Content Standards (book structure, RAG chatbot requirements)
- Development & Deployment Workflow (Docusaurus, GitHub Pages, quality gates)
- Governance (amendment process, version semantics, compliance)

All dependent templates reviewed - no updates required.

## Outcome

- Impact: Established project governance foundation for the Physical AI textbook
- Tests: N/A (governance document)
- Files: `.specify/memory/constitution.md` created
- Next prompts: Create feature specs for book modules, setup Docusaurus project, design RAG chatbot architecture
- Reflection: Constitution comprehensively captures the textbook's technical, pedagogical, and tooling requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A - governance document
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate constitution principles during first feature spec creation
