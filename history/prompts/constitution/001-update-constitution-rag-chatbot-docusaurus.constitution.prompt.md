---
id: 001
title: Update Constitution for RAG Chatbot + Docusaurus Book
stage: constitution
date: '2025-12-25'
model: claude-haiku-4-5-20251001
feature: none
branch: 1-rag-chatbot-backend
user: Claude
command: Analyze and update constitution.md according to unified book project requirements
labels:
  - constitution
  - rag-chatbot
  - docusaurus
  - book-deployment
  - hardware-infrastructure
  - course-structure
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - .specify/memory/constitution.md
tests: []
---

# Update Constitution for RAG Chatbot + Docusaurus Book

## Summary
Updated the project constitution (v1.1.0 → v1.2.0) to fully reflect the unified Physical AI & Humanoid Robotics learning platform, including the embedded RAG chatbot, Docusaurus book deployment to GitHub Pages, comprehensive hardware infrastructure guidance (three tiers), and detailed course structure with assessment criteria.

## Changes Made
- **Mission Statement:** Expanded to explicitly include Docusaurus textbook deployment, RAG chatbot with OpenAI Agents/FastAPI/Neon/Qdrant, and hardware guidance for three learning tiers (Budget Proxy, Miniature Humanoid, Premium)

- **Added Four New Principles:**
  - PRINCIPLE_9: RAG Chatbot Integration and Knowledge Accessibility (OpenAI Agents, Qdrant Cloud, Neon Postgres, text selection support)
  - PRINCIPLE_10: Deployment and Platform Accessibility (GitHub Pages, FastAPI serverless, Neon/Qdrant free tiers, browser-based access)
  - PRINCIPLE_11: Hardware and Lab Infrastructure Pragmatism (three hardware tiers, CapEx vs. OpEx models)
  - PRINCIPLE_12: Rigorous Assessment and Capstone Validation (module projects, capstone requirements, evaluation criteria)

- **New Section: Core Technical Stack**
  - Book delivery: Docusaurus 3.x, GitHub Pages, Algolia search
  - RAG backend: OpenAI API, Qdrant Cloud, Neon Postgres, FastAPI, embeddings
  - Simulation: ROS 2, Gazebo, Unity, NVIDIA Isaac, Nav2, Whisper
  - Hardware infrastructure: CapEx (RTX workstations + Jetson kits) and OpEx (AWS cloud instances) options with cost analysis

- **New Section: Course Structure and Assessment**
  - Four core technical modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA) with week allocation
  - Foundational weeks (Physical AI principles) and capstone week
  - Capstone project requirements (voice commands, planning, navigation, object identification, manipulation)
  - Module-level assessments aligned with practical outcomes

- **Enhanced AI-Assisted Development Framework**
  - Added human-in-the-loop for RAG chatbot responses (grounding, no hallucination)
  - Added backend infrastructure validation (FastAPI, Neon, Qdrant)
  - Enhanced QA protocols: simulation testing, chatbot retrieval audits, backend integration testing
  - Infrastructure code standards: production-readiness, secrets management, monitoring, logging

## Outcome
The constitution now provides a complete governance framework for a unified learning platform combining:
1. A Docusaurus textbook with automated GitHub Pages deployment
2. An embedded RAG chatbot grounded in book content
3. Comprehensive hardware and infrastructure options for diverse student contexts
4. Clear course structure aligned with four core technical modules
5. Rigorous assessment criteria and capstone validation requirements

Version updated to 1.2.0 with ratification date of 2025-12-25.

## Evaluation
All major deliverables are now fully specified in the constitution:
- ✅ Docusaurus book architecture defined
- ✅ RAG chatbot components explicitly named (OpenAI Agents, FastAPI, Qdrant, Neon)
- ✅ Three hardware tiers documented with cost analysis
- ✅ CapEx and OpEx lab infrastructure options detailed
- ✅ Course structure (weeks 1-13) with module progression
- ✅ Assessment criteria and capstone requirements defined
- ✅ AI development framework enhanced for chatbot and backend validation

All sections are internally consistent and align with the user's comprehensive requirements document. The constitution is now ready to guide all downstream specification, planning, and implementation phases.

---
PROMPT_TEXT: "Analyze and update constitution.md for the following unified Physical AI & Humanoid Robotics book project: (1) Docusaurus-based textbook deployed to GitHub Pages, (2) Embedded RAG chatbot using OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant Cloud with text selection support, (3) Hardware guidance for three tiers (Budget Proxy, Miniature, Premium) and two lab models (CapEx on-premise, OpEx cloud), (4) Four core modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA) spanning 13 weeks with weekly breakdown, (5) Assessments including module projects and capstone autonomous humanoid robot. Include cost estimates, infrastructure options, course schedule, and capstone requirements."

RESPONSE_TEXT: "Updated constitution from v1.1.0 to v1.2.0 with expanded mission statement, four new principles (RAG Chatbot, Deployment, Hardware Infrastructure, Assessment), new Core Technical Stack section covering Docusaurus/GitHub Pages, RAG backend (OpenAI/Qdrant/Neon/FastAPI), and simulation tools, new Hardware Infrastructure section with CapEx/OpEx options and cost breakdown (~$700 student kit, $180-205/quarter cloud), new Course Structure section with 13-week schedule and four modules, and enhanced AI-Assisted Development Framework. Constitution now governs complete unified learning platform."
