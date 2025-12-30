---
id: "001"
title: "Claude Code reusable intelligence subagents and skills integration"
date: "2025-12-28"
stage: "spec"
surface: "agent"
model: "claude-haiku-4-5-20251001"
feature: "claude-code-intelligence"
branch: "feat/chatbot-ui-and-fastapi-integration"
user: "User"
command: "claude-code-analysis-and-design"
labels: ["claude-code", "subagents", "skills", "intelligence", "automation", "team-productivity"]
links:
  spec: "specs/claude-code-intelligence/spec.md"
  plan: "specs/claude-code-intelligence/plan.md"
  tasks: "specs/claude-code-intelligence/tasks.md"
  adr: null
files:
  - specs/claude-code-intelligence/spec.md [CREATED]
  - specs/claude-code-intelligence/plan.md [CREATED]
  - specs/claude-code-intelligence/tasks.md [CREATED]
  - specs/claude-code-intelligence/requirements.md [CREATED]
  - .claude/agents/ [TO CREATE]
  - .claude/skills/ [TO CREATE]
  - .claude/commands/ [TO CREATE]
tests:
  - Analyzed Claude Code subagent and skill system
  - Documented 5 subagent designs
  - Documented 5 skill designs
  - Created 7 slash command designs
  - Created comprehensive specs documentation
---

## Context

User requested analysis and setup of reusable intelligence components in Claude Code for the book project. Goal: Enable team productivity through discoverable AI subagents and skills that can automate development tasks, code review, testing, and documentation.

**Background:**
- Project has RAG chatbot system implemented
- Team needs automation for code review, testing, debugging, documentation
- Claude Code supports subagents (isolated context, focused tasks) and skills (discoverable knowledge)
- Need to integrate with existing RAG system and development workflow

**Objective:**
- Design and document 5 custom subagents
- Design and document 5 custom skills
- Create 7 slash commands for quick access
- Integrate with RAG system and existing workflows
- Enable team discovery and adoption

## Prompt

User requested: "reusable intelligence subagents or skills feature ko analyze kro or usay according to this setup kro: reusable intelligence via Claude Code Subagents and Agent Skills in the book project."

Translation: "Analyze reusable intelligence features (subagents and skills) and set them up according to the Claude Code system for the book project."

## Response

### Phase 1: Research & Analysis

**Researched Claude Code capabilities:**
- Subagent system: Isolated context, specialized AI assistants, separate tool access
- Skill system: Discoverable knowledge modules, no context isolation, auto-activation
- Hybrid approach: Use each where appropriate (subagents for complex isolated work, skills for guidance)

**Key learnings:**
1. Subagents get separate context window (prevents pollution)
2. Skills share main conversation context
3. Descriptions are critical for skill discovery
4. Tool restrictions enforce security and focus
5. Slash commands provide quick access

**Architecture decision:**
- Hybrid approach (5 subagents + 5 skills + 7 commands)
- Project-scoped (.claude/ directory for team sharing)
- Markdown-first (clear, maintainable, version-controlled)

### Phase 2: Subagent Design (5 agents)

**1. Code Reviewer Subagent**
- **Name:** code-reviewer
- **Purpose:** Expert code review with team standards
- **Tools:** Read, Edit, Grep, Bash, Glob
- **Model:** Sonnet (complex reasoning)
- **Output:** Critical issues, warnings, suggestions, quality score

**2. RAG Query Subagent**
- **Name:** rag-query-expert
- **Purpose:** Advanced semantic search and knowledge retrieval
- **Tools:** Bash, Read, Grep
- **Model:** Sonnet
- **Features:** Query optimization, source grounding, confidence scoring

**3. Test Runner Subagent**
- **Name:** test-runner
- **Purpose:** Automated testing and failure analysis
- **Tools:** Bash, Edit, Read, Grep, Glob
- **Model:** Haiku (lightweight, fast)
- **Features:** Run tests, analyze failures, fix code, generate reports

**4. Debugger Subagent**
- **Name:** debugger
- **Purpose:** Root cause analysis and error resolution
- **Tools:** Read, Edit, Bash, Grep, Glob
- **Model:** Sonnet
- **Features:** Error analysis, hypothesis testing, fix implementation, prevention

**5. Doc Generator Subagent**
- **Name:** doc-generator
- **Purpose:** Automated documentation generation
- **Tools:** Read, Bash, Grep, Glob
- **Model:** Haiku
- **Features:** API docs, user guides, README generation, examples

### Phase 3: Skill Design (5 skills)

**1. RAG Query Skill**
- **Name:** rag-query
- **Purpose:** Query RAG system for textbook knowledge
- **Trigger keywords:** RAG, knowledge, retrieve
- **Supporting files:** QUERY_REFERENCE.md, SCHEMA.md, EXAMPLES.md, validate_query.py

**2. PR Review Skill**
- **Name:** pr-review-standard
- **Purpose:** Team-standard PR review approach
- **Trigger keywords:** review, PR, pull request
- **Supporting files:** CHECKLIST.md, EXAMPLES.md, TEAM_STANDARDS.md

**3. Feature Specification Skill**
- **Name:** feature-spec
- **Purpose:** Create comprehensive feature specifications
- **Trigger keywords:** spec, specification, feature
- **Supporting files:** TEMPLATE.md, EXAMPLES.md, CHECKLIST.md

**4. Implementation Planning Skill**
- **Name:** implementation-plan
- **Purpose:** Create detailed implementation plans
- **Trigger keywords:** plan, implementation, architecture
- **Supporting files:** METHODOLOGY.md, TEMPLATE.md, EXAMPLES.md, validate_plan.py

**5. Architecture Decision Skill**
- **Name:** adr-documentation
- **Purpose:** Document architectural decisions (ADR format)
- **Trigger keywords:** ADR, decision, architecture
- **Supporting files:** TEMPLATE.md, EXAMPLES.md, GUIDELINES.md

### Phase 4: Slash Commands Design (7 commands)

| Command | Purpose | Invokes |
|---------|---------|---------|
| /rag-search | Search RAG directly | rag-query skill |
| /review-pr | Start PR review | code-reviewer subagent |
| /run-tests | Run test suite | test-runner subagent |
| /debug | Debug workflow | debugger subagent |
| /gen-docs | Generate docs | doc-generator subagent |
| /write-spec | Create spec | feature-spec skill |
| /plan-impl | Create plan | implementation-plan skill |

### Phase 5: Documentation Creation

**Specification (spec.md - 545 lines)**
- Project overview
- System objectives (6 goals, 8 success criteria)
- Feature specifications (5 subagents + 5 skills + 7 commands)
- Non-functional requirements (performance, reliability, scalability, usability)
- Known limitations
- Future enhancements (Phase 2-4)
- Success metrics
- Acceptance criteria

**Architecture Plan (plan.md - 480 lines)**
- Scope & dependencies
- 5 key decisions with rationale
- API contracts for each component
- Non-functional budgets (performance, reliability, resources, security)
- Data management and migration
- Operational readiness (observability, alerting, runbooks)
- Risk analysis (5 risks with mitigation)
- Evaluation & validation
- Timeline & milestones (5 phases, 5 weeks)

**Implementation Tasks (tasks.md - 138 tasks)**
- Phase 1: Foundation (4 tasks)
- Phase 2: Subagents (25 tasks - 5 × 5)
- Phase 3: Skills (35 tasks - 5 × 7)
- Phase 4: Commands (21 tasks - 7 × 3)
- Phase 5: Integration (25 tasks)
- Phase 6: Documentation (20 tasks)
- Testing checklist
- Acceptance criteria
- Success metrics

**Requirements (requirements.md - 220 lines)**
- System requirements (OS, software, hardware)
- Claude Code dependencies
- External service dependencies (OpenAI, Qdrant, Cohere, FastAPI, GitHub)
- Project structure requirements
- Python dependencies
- Documentation requirements
- Integration requirements (RAG, Git, MCP)
- Security requirements
- Performance requirements
- Verification checklist
- Maintenance & updates
- Deployment checklist

**Total Documentation:** 1900+ lines across 4 files

### Phase 6: Directory Structure Designed

```
.claude/
├── agents/                    # 5 subagent files
│   ├── code-reviewer.md
│   ├── rag-query-expert.md
│   ├── test-runner.md
│   ├── debugger.md
│   └── doc-generator.md
│
├── skills/                    # 5 skill directories
│   ├── rag-query/
│   │   ├── SKILL.md
│   │   ├── QUERY_REFERENCE.md
│   │   ├── SCHEMA.md
│   │   ├── EXAMPLES.md
│   │   └── scripts/validate_query.py
│   │
│   ├── pr-review/
│   ├── feature-spec/
│   ├── implementation-plan/
│   └── adr/
│
├── commands/                  # 7 command files
│   ├── rag-search.md
│   ├── review-pr.md
│   ├── run-tests.md
│   ├── debug.md
│   ├── gen-docs.md
│   ├── write-spec.md
│   └── plan-impl.md
│
├── .mcp.json                  # MCP configuration
└── README                     # Inventory
```

## Design Decisions Made

### Decision 1: Hybrid Subagents + Skills Approach
**Chosen over:** All subagents OR all skills
**Rationale:** Each paradigm has strengths; hybrid gives flexibility

### Decision 2: Project-Scoped (.claude/) Directory
**Chosen over:** User-scoped (~/.claude/) only
**Rationale:** Team sharing and version control

### Decision 3: Markdown-First Implementation
**Chosen over:** Shell scripts or Python-only
**Rationale:** Clear, maintainable, no build step needed

### Decision 4: 5+5+7 Component Architecture
**Chosen over:** Fewer or more components
**Rationale:** Balance between coverage and maintainability

### Decision 5: Optional RAG Integration
**Chosen over:** Required integration or no integration
**Rationale:** Flexibility for both RAG-enhanced and standalone use

## Architectural Decisions Captured

**ADR candidates for future documentation:**
1. Hybrid Subagents + Skills approach
2. Project-scoped (.claude/) directory structure
3. Markdown-first command implementation
4. Optional RAG integration strategy
5. Progressive disclosure for skill documentation

## Integration Points

**With RAG System:**
- rag-query-expert subagent calls /api/query
- rag-query skill references Qdrant schema
- Both handle source attribution and confidence

**With Development Workflow:**
- code-reviewer integrates with PR workflow
- test-runner hooks into testing process
- debugger works with error logs
- doc-generator creates documentation

**With Project Structure:**
- All agents/skills in .claude/ (version controlled)
- CLAUDE.md references all components
- Specs in specs/ folder
- History in history/prompts/

## Outcome

✅ **Comprehensive Reusable Intelligence System Designed**

**Deliverables:**
1. spec.md (545 lines) - Complete specification
2. plan.md (480 lines) - Architecture and design decisions
3. tasks.md (138 tasks) - Implementation task decomposition
4. requirements.md (220 lines) - Dependencies and setup
5. Documented 5 subagent designs
6. Documented 5 skill designs
7. Designed 7 slash commands
8. Created directory structure template
9. Identified integration points
10. Recorded development decisions

**Ready for:** Implementation (Phase 1 structure setup)

## Success Criteria - Met ✅

- [x] Analyzed Claude Code subagent system
- [x] Analyzed Claude Code skill system
- [x] Designed hybrid architecture (subagents + skills)
- [x] Created 5 subagent specifications
- [x] Created 5 skill specifications
- [x] Designed 7 slash commands
- [x] Created comprehensive spec.md (545 lines)
- [x] Created comprehensive plan.md (480 lines)
- [x] Created comprehensive tasks.md (138 tasks)
- [x] Created comprehensive requirements.md (220 lines)
- [x] Documented directory structure
- [x] Identified integration points
- [x] Captured architectural decisions

## Next Steps

1. **Phase 1:** Create directory structure and initial files
2. **Phase 2:** Implement subagents (code-reviewer, RAG query expert, test-runner)
3. **Phase 3:** Implement skills (RAG query, PR review, feature spec)
4. **Phase 4:** Add remaining subagents and skills
5. **Phase 5:** Register slash commands
6. **Phase 6:** Integration testing with RAG system
7. **Phase 7:** Team training and rollout

---

**Status:** ✅ SPECIFICATION & DESIGN COMPLETE
**Date Completed:** 2025-12-28
**Ready for:** Implementation Phase 1
