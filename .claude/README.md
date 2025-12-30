# Claude Code Intelligence System

Complete inventory of reusable intelligence subagents and skills for the Physical AI and Humanoid Robotics project.

---

## Quick Overview

This `.claude/` directory contains:
- **5 Specialized Subagents:** Autonomous AI agents for complex tasks
- **5 Discoverable Skills:** Knowledge modules that auto-activate
- **7 Slash Commands:** Quick-access commands for common workflows

Everything is automatically discovered and available in Claude Code.

---

## Available Subagents

Specialized AI agents with isolated contexts, focused on specific domains.

### 1. Code Reviewer
**File:** `.claude/agents/code-reviewer.md`

Expert code reviewer providing quality, security, and architecture assessment.

- **Triggers:** `/review-pr PR#123` or provide code
- **Dimensions:** Quality (40%), Security (30%), Architecture (20%), Testing (10%)
- **Outputs:** Critical issues, warnings, suggestions with scores
- **Tools:** Read, Edit, Grep, Bash, Glob
- **Model:** Sonnet (complex reasoning)

### 2. RAG Query Expert
**File:** `.claude/agents/rag-query-expert.md`

Advanced semantic search specialist for knowledge base retrieval.

- **Triggers:** `/rag-search [question]`
- **Specialization:** Query optimization, source grounding, confidence scoring
- **Capabilities:** Vector search, LLM generation, result ranking
- **Integration:** Qdrant database, Cohere embeddings
- **Tools:** Bash, Read, Grep
- **Model:** Sonnet

### 3. Test Runner
**File:** `.claude/agents/test-runner.md`

Test automation specialist that runs tests and fixes failures.

- **Triggers:** `/run-tests` or `/run-tests tests/file.py`
- **Capabilities:** Run tests, analyze failures, fix issues, coverage reports
- **Frameworks:** pytest, unittest, Jest, Mocha, RSpec
- **Features:** Auto-fix common patterns, coverage analysis
- **Tools:** Bash, Edit, Read, Grep, Glob
- **Model:** Haiku (lightweight, fast)

### 4. Debugger
**File:** `.claude/agents/debugger.md`

Root cause analysis specialist for error resolution.

- **Triggers:** `/debug [error message or stack trace]`
- **Process:** Error capture → scope narrowing → hypothesis → testing → fix
- **Categories:** Config, API/network, data, logic, async/concurrency
- **Output:** Root cause, fix, prevention recommendations
- **Tools:** Read, Edit, Bash, Grep, Glob
- **Model:** Sonnet

### 5. Documentation Generator
**File:** `.claude/agents/doc-generator.md`

Documentation specialist for code, APIs, and features.

- **Triggers:** `/gen-docs [what to document]`
- **Outputs:** API docs, README, user guides, code examples, architecture diagrams
- **Standards:** Writing style, code examples, organization
- **Formats:** Markdown, HTML, PDF, Confluence
- **Tools:** Read, Bash, Grep, Glob
- **Model:** Haiku

---

## Available Skills

Discoverable knowledge modules that auto-activate based on keywords.

### 1. RAG Query Skill
**Location:** `.claude/skills/rag-query/`

Semantic search and knowledge retrieval.

**Files:**
- `SKILL.md` - Overview and quick start (180 lines)
- `SCHEMA.md` - Database info, payload structure, rate limits (150 lines)
- `EXAMPLES.md` - 6 detailed examples with different query types (320 lines)

**Activation Keywords:** "rag", "semantic search", "knowledge base", "query"

**Capabilities:**
- Query formulation and optimization
- Result interpretation and ranking
- Source attribution with confidence
- Performance optimization tips

### 2. PR Review Skill
**Location:** `.claude/skills/pr-review/`

Code review standards and practices.

**Files:**
- `SKILL.md` - Review categories, checklist (120 lines)
- `CHECKLIST.md` - Detailed verification items (420 lines)
- `EXAMPLES.md` - Good/bad review examples (380 lines)
- `TEAM_STANDARDS.md` - Code standards, patterns, security, testing (550 lines)

**Activation Keywords:** "code review", "pull request", "PR", "review"

**Content Sections:**
- Code quality standards (Python, TypeScript/JavaScript)
- Naming conventions and style
- Architecture patterns and DRY principle
- Testing standards (80%+ coverage)
- Security guidelines (OWASP top 10)
- API design conventions
- Git and versioning standards

### 3. Feature Specification Skill
**Location:** `.claude/skills/feature-spec/`

Creating comprehensive feature specifications.

**Files:**
- `SKILL.md` - Structure and quality checklist (100 lines)
- `TEMPLATE.md` - Full specification template (380 lines)
- `EXAMPLES.md` - 2 detailed example specs (580 lines)
- `CHECKLIST.md` - Verification checklist (420 lines)

**Activation Keywords:** "specification", "spec", "feature", "requirements"

**Specification Structure:**
1. Overview (description, value, success definition)
2. Objectives (goals, success criteria, out of scope)
3. Feature Details (scenarios, behavior, edge cases)
4. Requirements (functional, non-functional)
5. Acceptance Criteria (testable conditions)
6. Data Models (request/response examples)
7. API/Interface Design (endpoints, validation)
8. Testing Strategy (unit, integration, edge cases)
9. Known Limitations (constraints, deferred work)

### 4. Implementation Plan Skill
**Location:** `.claude/skills/implementation-plan/`

Detailed project planning and execution.

**Files:**
- `SKILL.md` - Structure and principles (110 lines)
- `METHODOLOGY.md` - 8-step planning methodology (480 lines)
- `TEMPLATE.md` - Comprehensive plan template (400 lines)
- `EXAMPLES.md` - 2 detailed example plans (650 lines)

**Activation Keywords:** "plan", "implementation", "planning", "design"

**Planning Methodology:**
1. Scope & dependencies analysis
2. Architectural decisions
3. Task decomposition (2-16 hour tasks)
4. Sequencing & dependencies
5. Timeline & milestones
6. Risk analysis (probability, impact, mitigation)
7. Testing strategy
8. Success metrics & validation

**Key Principles:**
- Realism over optimism
- Dependencies first
- Risk-aware approach
- Regular validation
- 15-20% buffer for unknowns

### 5. ADR Skill
**Location:** `.claude/skills/adr/`

Architecture Decision Records for significant decisions.

**Files:**
- `SKILL.md` - ADR structure and quality checklist (115 lines)
- `TEMPLATE.md` - Complete ADR template (400 lines)
- `EXAMPLES.md` - Real ADR examples (520 lines)
- `GUIDELINES.md` - Writing guidelines and process (580 lines)

**Activation Keywords:** "ADR", "architecture decision", "decision record"

**ADR Structure:**
1. Title (clear, descriptive noun phrase)
2. Status (Proposed | Accepted | Deprecated | Superseded)
3. Context (problem, why it matters, constraints)
4. Decision (what's being decided)
5. Rationale (why chosen, problems solved)
6. Consequences (positive & negative)
7. Alternatives Considered (with pros/cons)
8. Related Decisions (linked ADRs)

**When to Create ADRs:**
- Technology/framework selection
- Architectural patterns
- Data model decisions
- API design choices
- Deployment strategy
- Major refactoring

---

## Available Commands

Quick-access slash commands for common workflows.

### Command Reference

| Command | Purpose | Agent/Skill | Invokes | Response Time |
|---------|---------|-------------|---------|---|
| `/rag-search` | Knowledge retrieval | RAG Query | rag-query-expert | <5s |
| `/review-pr` | Code review | PR Review | code-reviewer | 30-60s |
| `/run-tests` | Test automation | Test Runner | test-runner | 60-120s |
| `/debug` | Error debugging | Debugger | debugger | 30-60s |
| `/gen-docs` | Documentation | Documentation | doc-generator | 30-60s |
| `/write-spec` | Feature spec | Feature Spec Skill | feature-spec | <5s |
| `/plan-impl` | Implementation plan | Implementation Plan Skill | implementation-plan | <5s |

### Command Details

#### 1. `/rag-search`
Search the RAG knowledge base for Physical AI and robotics information.

```
/rag-search What is ROS2?
/rag-search How do humanoid robots maintain balance?
/rag-search How do I implement inverse kinematics in Python?
```

**Returns:** Grounded answers with sources and confidence scores

**See:** `.claude/commands/rag-search.md`

#### 2. `/review-pr`
Conduct professional code reviews using team standards.

```
/review-pr PR#42
/review-pr https://github.com/owner/repo/pull/123
/review-pr [paste code]
```

**Returns:** Quality/security/architecture assessment with severity levels

**See:** `.claude/commands/review-pr.md`

#### 3. `/run-tests`
Run test suite and automatically fix failures.

```
/run-tests
/run-tests tests/test_auth.py
/run-tests tests/test_auth.py::test_login
```

**Returns:** Test results, coverage report, fixed tests

**See:** `.claude/commands/run-tests.md`

#### 4. `/debug`
Debug errors with systematic root cause analysis.

```
/debug TypeError: cannot read property 'name' of undefined
/debug [paste stack trace]
/debug Database connection randomly drops after 10 minutes
```

**Returns:** Root cause analysis, fix, prevention recommendations

**See:** `.claude/commands/debug.md`

#### 5. `/gen-docs`
Generate professional documentation automatically.

```
/gen-docs API documentation for /api/users endpoint
/gen-docs README for authentication module
/gen-docs User guide for feature X
```

**Returns:** Formatted documentation with examples

**See:** `.claude/commands/gen-docs.md`

#### 6. `/write-spec`
Create comprehensive feature specifications.

```
/write-spec Add favorite button to course cards
/write-spec Implement RAG-powered semantic search
/write-spec Create user profile API endpoints
```

**Returns:** Specification with objectives, requirements, acceptance criteria

**See:** `.claude/commands/write-spec.md`

#### 7. `/plan-impl`
Create detailed implementation plans.

```
/plan-impl User authentication system from the specification
/plan-impl Dark mode support across entire UI
/plan-impl Integrate Qdrant vector database for RAG search
```

**Returns:** Task decomposition, timeline, risk analysis

**See:** `.claude/commands/plan-impl.md`

---

## Directory Structure

```
.claude/
├── README.md              ← You are here
├── agents/
│   ├── code-reviewer.md           (277 lines)
│   ├── rag-query-expert.md        (280 lines)
│   ├── test-runner.md             (285 lines)
│   ├── debugger.md                (380 lines)
│   └── doc-generator.md           (315 lines)
│
├── skills/
│   ├── rag-query/
│   │   ├── SKILL.md               (180 lines)
│   │   ├── SCHEMA.md              (150 lines)
│   │   └── EXAMPLES.md            (320 lines)
│   │
│   ├── pr-review/
│   │   ├── SKILL.md               (120 lines)
│   │   ├── CHECKLIST.md           (420 lines)
│   │   ├── EXAMPLES.md            (380 lines)
│   │   └── TEAM_STANDARDS.md      (550 lines)
│   │
│   ├── feature-spec/
│   │   ├── SKILL.md               (100 lines)
│   │   ├── TEMPLATE.md            (380 lines)
│   │   ├── EXAMPLES.md            (580 lines)
│   │   └── CHECKLIST.md           (420 lines)
│   │
│   ├── implementation-plan/
│   │   ├── SKILL.md               (110 lines)
│   │   ├── METHODOLOGY.md         (480 lines)
│   │   ├── TEMPLATE.md            (400 lines)
│   │   └── EXAMPLES.md            (650 lines)
│   │
│   └── adr/
│       ├── SKILL.md               (115 lines)
│       ├── TEMPLATE.md            (400 lines)
│       ├── EXAMPLES.md            (520 lines)
│       └── GUIDELINES.md          (580 lines)
│
└── commands/
    ├── COMMANDS.md                (91 lines) - Reference guide
    ├── rag-search.md
    ├── review-pr.md
    ├── run-tests.md
    ├── debug.md
    ├── gen-docs.md
    ├── write-spec.md
    └── plan-impl.md
```

**Total Code:** ~8,300 lines across 34 files

---

## How Claude Code Discovers These

### Subagent Discovery
Claude Code automatically discovers agents in `.claude/agents/` directory.

Each agent file must have:
- YAML frontmatter with metadata
- Name and description
- Model specification (haiku, sonnet, opus)
- Allowed tools list
- Detailed instructions

### Skill Discovery
Claude Code automatically discovers skills in `.claude/skills/*/` directories.

Each skill must have:
- `SKILL.md` file with YAML frontmatter
- Name and description keywords
- How it's used
- Supporting documentation files

### Command Discovery
Claude Code automatically discovers slash commands in `.claude/commands/` directory.

Each command file:
- YAML metadata (name, invokes, description)
- Usage examples
- Detailed documentation

---

## Integration Points

### With RAG System
- `/rag-search` command queries Qdrant vector database
- Returns semantically relevant results with sources
- Integrates with Cohere embeddings (embed-english-v3.0)

### With FastAPI Backend
- `/review-pr`, `/run-tests`, `/debug` analyze FastAPI code
- `/gen-docs` generates API documentation
- Commands work with project's Python/FastAPI patterns

### With Development Workflow
- `/write-spec` creates specs for new features
- `/plan-impl` creates implementation plans
- `/review-pr` ensures code quality in PRs

---

## Quick Start

### Using a Command
```
/rag-search What is inverse kinematics?
```

Claude Code finds and invokes the rag-query-expert agent automatically.

### Invoking a Skill
The skill auto-activates when mentioned:
```
"Let me write a feature spec for..."
```

Feature-spec skill auto-activates because of keywords.

### Getting Help
Each agent, skill, and command has documentation:
- `.claude/agents/[agent].md` - Full agent documentation
- `.claude/skills/[skill]/SKILL.md` - Skill overview
- `.claude/commands/[command].md` - Command details

---

## Development Notes

### Adding New Agents
1. Create `.claude/agents/[name].md`
2. Include YAML frontmatter with metadata
3. Include detailed instructions
4. Specify model and allowed tools
5. Document in this README

### Adding New Skills
1. Create `.claude/skills/[name]/` directory
2. Create `SKILL.md` with overview
3. Create supporting files (TEMPLATE.md, EXAMPLES.md, etc.)
4. Update this README

### Adding New Commands
1. Create `.claude/commands/[name].md`
2. Include YAML metadata
3. Document usage and examples
4. Reference in COMMANDS.md

---

## Quality Standards

All agents, skills, and commands follow:

- **Clear Documentation:** Easy to understand and use
- **Concrete Examples:** Real working examples
- **Complete Coverage:** All major features documented
- **Cross-References:** Links between related items
- **Progressive Disclosure:** Summary → Details → Advanced

---

## Related Documentation

- **Specification Details:** See `specs/claude-code-intelligence/`
- **Implementation Plan:** See `specs/claude-code-intelligence/plan.md`
- **Tasks Completed:** See `specs/claude-code-intelligence/tasks.md`
- **PHR Records:** See `history/prompts/claude-code-intelligence/`

---

## Support

For questions about:

- **Subagents:** See individual agent files in `.claude/agents/`
- **Skills:** See `SKILL.md` in each skill directory
- **Commands:** See command files in `.claude/commands/`
- **General:** See `COMMANDS.md` for quick reference

---

## Version

**System Version:** 1.0
**Last Updated:** 2025-12-28
**Components:** 5 agents + 5 skills + 7 commands
**Total Lines:** ~8,300 across 34 files

**Status:** Ready for team use ✓
