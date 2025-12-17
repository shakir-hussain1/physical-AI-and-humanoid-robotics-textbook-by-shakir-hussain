---
id: 001
title: Establish backend RAG constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: shakir-hussain
command: /sp.constitution
labels: ["rag", "backend", "academic-integrity", "zero-hallucination", "fastapi"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/.specify/memory/constitution.md
tests:
  - none
---

## Prompt

You are a senior AI systems architect, backend engineer, and academic integrity auditor.

Your task is to define the CONSTITUTION for the backend RAG chatbot system of the project:

"Physical AI and Humanoid Robotics – Academic Book Platform"

This constitution governs all future specifications, plans, tasks, and code related to the chatbot backend.

────────────────────────────────
PROJECT PURPOSE
────────────────────────────────
Design and implement an Integrated Retrieval-Augmented Generation (RAG) chatbot backend that is embedded into a Docusaurus-based academic book.

The chatbot must answer user queries strictly based on:
• The book's verified content
• User-selected text passages (context-restricted answers)
• Retrieved academic chunks with citations

The system must never hallucinate or answer outside the verified knowledge base.

────────────────────────────────
CORE DELIVERABLE
────────────────────────────────
A production-ready RAG backend using:

• FastAPI (Python)
• ERIA (vector database)

────────────────────────────────
REQUIREMENTS
────────────────────────────────
• All answers verifiable against stored sources
• Zero hallucinations in evaluation
• Passes academic fact-checking review
• Fully compliant with the book's academic constitution

────────────────────────────────
OVERRIDING AUTHORITY
────────────────────────────────
This constitution overrides:
• Style preferences
• Performance optimizations
• Later conflicting instructions

All future /sp.specify, /sp.plan, and /sp.tasks MUST strictly comply with this constitution.

## Response snapshot

**Constitution Created Successfully (v1.0.0)**

Ratified 6 core principles for the backend RAG chatbot:

1. **Knowledge Source Verification** — Every answer grounded in verified sources (book content, user passages, retrieved chunks with citations)
2. **Zero Hallucination Guarantee** — Retrieval-first, confidence thresholds, citation-required, out-of-scope detection
3. **Academic Integrity & Citation Standards** — Full citations [Source ID / Page / Section], no paraphrasing without source linkage
4. **Test-First Answer Validation** — Mandatory fact-checking before deploy; ≥95% accuracy required; unit, integration, manual review
5. **FastAPI + Vector DB Architecture** — Response schema with sources + confidence, 5-second timeout, structured logging
6. **Context-Restricted Answers** — User passage selection mode bypasses vector search; answers bounded to selected text

**Additional Sections Established:**
- Security & Academic Integrity (data handling, fact-checking review gate)
- Observability & Quality Assurance (logging, metrics, testing gates ≥80% coverage)
- Development Workflow (spec requirements, code review standards, deployment + rollback procedures)
- Governance (amendment process, compliance review, version control)

**Next Steps:** Generate feature specs, implementation plan, and task breakdown aligned to these principles.

## Outcome

- ✅ Impact: Backend constitution established as authoritative governance document; governs all future /sp.specify, /sp.plan, /sp.tasks for RAG chatbot
- 🧪 Tests: None (constitution is governance artifact; grading rules embedded in future test tasks)
- 📁 Files: backend/.specify/memory/constitution.md (v1.0.0)
- 🔁 Next prompts: /sp.specify (RAG chatbot feature spec), /sp.plan (architecture design), /sp.tasks (implementation tasks)
- 🧠 Reflection: Constitution captures non-negotiable academic integrity requirements (zero hallucination, source verification, fact-checking review gate) upfront; this prevents costly pivots during development and ensures RAG backend meets academic standards for textbook embedding

## Evaluation notes (flywheel)

- Failure modes observed: None (constitution creation is deterministic governance task)
- Graders run and results (PASS/FAIL): Not applicable (governance artifact)
- Prompt variant (if applicable): None
- Next experiment: Validate constitution alignment in first feature spec (/sp.specify); confirm all acceptance criteria tie to one of 6 core principles
