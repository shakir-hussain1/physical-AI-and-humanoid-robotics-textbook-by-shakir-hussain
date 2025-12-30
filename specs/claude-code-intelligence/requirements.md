# Claude Code Reusable Intelligence - Requirements

## 1. System Requirements

### Development Environment

**Operating System:**
- Windows 10+ (or WSL 2)
- macOS 11+
- Linux (Ubuntu 20.04+)

**Software Requirements:**
- Claude Code CLI v1.0+ (with Agent SDK support)
- Python 3.9+
- Git 2.30+
- Node.js 14+ (for frontend integration, optional)
- Bash/Shell (for command execution)

**Recommended Hardware:**
- RAM: 8GB minimum, 16GB+ recommended
- Disk: 2GB free for .claude/ directory and caches
- Network: Stable internet (for API calls to external services)

---

## 2. Claude Code & Dependencies

### Claude Code Version
- **Minimum:** v1.0.0 (with Agent SDK)
- **Recommended:** Latest stable version
- **Installation:** `curl ... | bash`

### Claude Code Features Required
- Subagent system (for isolated contexts)
- Skill system (for discoverable knowledge)
- Tool access (Read, Edit, Bash, Grep, Glob)
- Slash command registration
- MCP server integration

### Tool Requirements

| Tool | Required | Used By | Purpose |
|------|----------|---------|---------|
| **Read** | Yes | All agents/skills | Read file contents |
| **Edit** | Recommended | code-reviewer, test-runner, debugger | Modify files |
| **Bash** | Yes | rag-query, test-runner, debugger, doc-generator | Execute commands |
| **Grep** | Yes | code-reviewer, debugger, doc-generator | Search code |
| **Glob** | Yes | code-reviewer, doc-generator | Find files |

### Access Levels Required

```yaml
Subagents:
  code-reviewer:
    - Read (files for review)
    - Edit (suggest fixes)
    - Grep (search patterns)
    - Bash (run linters)
    - Glob (find related files)

  rag-query-expert:
    - Bash (call API)
    - Read (reference docs)
    - Grep (search schema)

  test-runner:
    - Bash (run tests, pip)
    - Edit (fix code)
    - Read (analyze code)
    - Grep (search patterns)
    - Glob (find test files)

  debugger:
    - Read (analyze code)
    - Edit (apply fixes)
    - Bash (run debug commands)
    - Grep (search for issues)
    - Glob (find related files)

  doc-generator:
    - Read (analyze code)
    - Bash (run doc tools)
    - Grep (extract info)
    - Glob (find modules)
```

---

## 3. External Service Dependencies

### APIs Required

**OpenAI API (Existing)**
- Status: Already configured
- Used by: RAG Query subagent (indirectly via /api/query)
- Configuration: OPENAI_API_KEY in .env

**Qdrant Vector Database (Existing)**
- Status: Already configured
- Used by: RAG Query subagent/skill
- Configuration: QDRANT_URL, QDRANT_API_KEY in .env

**Cohere Embedding Service (Existing)**
- Status: Already configured
- Used by: RAG Query subagent/skill
- Configuration: COHERE_API_KEY in .env

**FastAPI Backend (Existing)**
- Status: Implemented
- Used by: RAG Query subagent/skill
- Endpoints:
  - POST /api/query - Main RAG endpoint
  - POST /api/retrieve - Vector search
  - GET /api/health - Health check
- Requirements: Must be running on port 8000

**GitHub API (Optional)**
- Status: Already configured via MCP
- Used by: Code reviewer, PR review skill
- Configuration: Git CLI with auth

---

## 4. Project Structure Requirements

### Directory Layout

```
E:\Physical-AI-and-Humanoid-Robotics\
├── .claude/                              # REQUIRED
│   ├── agents/                           # 5 subagent files
│   ├── skills/                           # 5 skill directories
│   ├── commands/                         # 7 command files
│   ├── .mcp.json                         # MCP configuration
│   └── README                            # Inventory
│
├── backend/                              # REQUIRED (for RAG API)
│   ├── src/
│   │   ├── api/
│   │   │   ├── endpoints/
│   │   │   │   ├── query.py              # Must exist and work
│   │   │   │   ├── retrieve.py
│   │   │   │   └── health.py
│   │   │   └── main.py
│   │   ├── agent/
│   │   └── retrieval/
│   └── requirements.txt
│
├── specs/                                # REQUIRED (for documentation)
│   └── claude-code-intelligence/
│       ├── spec.md
│       ├── plan.md
│       ├── tasks.md
│       └── requirements.md
│
├── history/                              # REQUIRED (for PHRs)
│   └── prompts/
│       └── claude-code-intelligence/
│           └── PHR files
│
├── CLAUDE.md                             # REQUIRED (inventory)
├── .env                                  # REQUIRED (secrets)
├── .gitignore                            # REQUIRED (version control)
└── START_BACKEND.bat                     # REQUIRED (backend startup)
```

### Configuration Files

**CLAUDE.md - Project Memory**
```yaml
Required sections:
  - Available Subagents (with descriptions)
  - Available Skills (with descriptions)
  - Available Commands (with documentation)
  - Discovery instructions
  - Invocation examples
```

**.env - Environment Variables**
```bash
# Existing (must keep)
OPENAI_API_KEY=sk-proj-...
OPENAI_MODEL=gpt-4o-mini
QDRANT_URL=https://...
QDRANT_API_KEY=...
QDRANT_COLLECTION=textbook_embeddings
COHERE_API_KEY=...

# New (optional additions)
CLAUDE_CODE_SUBAGENT_MODEL=sonnet
CLAUDE_CODE_LOG_LEVEL=INFO
```

**.mcp.json - MCP Configuration**
```json
{
  "mcpServers": {
    "github": {
      "enabled": true,
      "tools": ["all"]
    },
    "ide": {
      "enabled": true,
      "tools": ["all"]
    }
  }
}
```

---

## 5. Python Dependencies

### For Subagents/Skills Support Scripts

```
# scripts/validate_query.py
requests>=2.28.0
pydantic>=2.0.0

# scripts/validate_plan.py
PyYAML>=6.0
jsonschema>=4.0

# Backend Integration
fastapi>=0.104.0  (already in backend/requirements.txt)
pydantic>=2.0.0   (already in backend/requirements.txt)
openai>=1.3.0     (already in backend/requirements.txt)
qdrant-client>=2.7.0  (already in backend/requirements.txt)
cohere>=5.0.0     (already in backend/requirements.txt)
```

### Installation

```bash
# No separate installation needed!
# All Python deps already in backend/requirements.txt

# Verify installation:
pip list | grep -E "fastapi|pydantic|openai|qdrant|cohere"
```

---

## 6. Documentation Requirements

### Specification Files (Required)

```
specs/claude-code-intelligence/
├── spec.md           # Feature specification (545+ lines)
├── plan.md           # Architecture plan (480+ lines)
├── tasks.md          # Implementation tasks (138 tasks)
├── requirements.md   # This file
└── README.md         # Overview and quick start
```

### Subagent Documentation (Required)

Each subagent file must have:
```markdown
---
name: agent-name
description: Clear description with trigger keywords
tools: Tool1, Tool2, Tool3
model: sonnet|haiku
---

# Purpose
[What it does]

# System Prompt
[Detailed instructions]

# Input/Output
[Contract specification]

# Examples
[Sample usage]
```

### Skill Documentation (Required)

Each skill must have:
```markdown
.claude/skills/skill-name/
├── SKILL.md                    # Overview (max 500 lines)
├── [REFERENCE.md]              # Optional: Detailed reference
├── [EXAMPLES.md]               # Optional: Usage examples
├── [TEMPLATES.md]              # Optional: Templates
└── scripts/                    # Optional: Validation/helper scripts
    └── validate_[name].py
```

### Command Documentation (Required)

Each command must have:
```markdown
---
name: /command-name
description: What it does
trigger: When to use it
invokes: agent-name or skill-name
---

## Parameters
[List of parameters]

## Examples
[Realistic examples]

## Output
[What it returns]
```

---

## 7. Integration Requirements

### With RAG System (Required)

**API Availability:**
- FastAPI backend running on http://localhost:8000
- /api/query endpoint accessible and working
- /api/retrieve endpoint accessible
- /api/health endpoint responding

**Configuration:**
```bash
# Backend must be running
START_BACKEND.bat

# Or manually:
cd backend
python -m uvicorn src.api.main:app --host 0.0.0.0 --port 8000
```

**Integration Points:**
- rag-query-expert subagent calls /api/query
- rag-query skill references /api/query parameters
- Both handle source attribution and confidence scores

### With Version Control (Required)

**Git Configuration:**
```bash
# Must be in git repository
git init

# Must track .claude/ directory
git add .claude/

# Must have proper .gitignore
cat .gitignore | grep -E "\.env|venv|\.pyc"

# All changes must be committed
git commit -m "Add Claude Code intelligence components"

# Must be pushable to remote
git push origin main
```

### With MCP Servers (Optional but Recommended)

**GitHub MCP:**
- Enables PR review workflows
- Allows code-reviewer to access repository
- Configuration: Requires GitHub token

**IDE MCP:**
- Enables IDE integration
- Allows quick file access
- Configuration: Usually automatic

---

## 8. Security Requirements

### Secrets Management

**No Secrets in .claude/:**
- ❌ Don't put API keys in skill/agent files
- ❌ Don't put passwords in command files
- ✅ Use environment variables from .env
- ✅ Reference env vars in code: `os.environ.get('KEY')`

**File Permissions:**
```bash
# .env should NOT be readable by others
chmod 600 .env

# .claude/ can be shared (no secrets)
chmod 755 .claude/
```

**Git Security:**
```bash
# .env must be gitignored
echo ".env" >> .gitignore

# Check for accidental secrets
git log --all -p -- .claude/ | grep -i "key\|secret\|token"
```

### Input Validation

**Subagents/Skills must validate:**
- User-provided file paths (prevent directory traversal)
- Query parameters (prevent injection attacks)
- External API inputs (verify format)
- Command output (sanitize for display)

### Error Handling

**Never expose:**
- Full API keys (mask in logs)
- User credentials
- Internal file paths
- Stack traces to users

---

## 9. Performance Requirements

### Subagent Performance

| Subagent | Startup | Execution | Max Memory |
|----------|---------|-----------|------------|
| code-reviewer | <2s | 10-30s per 500 LOC | 500MB |
| rag-query-expert | <2s | 5-15s per query | 400MB |
| test-runner | <2s | 30-120s | 800MB |
| debugger | <2s | 10-30s | 400MB |
| doc-generator | <2s | 15-60s | 500MB |

### Skill Performance

| Skill | Load Time | Discovery | Max Size |
|-------|-----------|-----------|----------|
| rag-query | <500ms | <1s | 100MB |
| pr-review | <500ms | <1s | 50MB |
| feature-spec | <500ms | <1s | 25MB |
| implementation-plan | <500ms | <1s | 30MB |
| adr | <500ms | <1s | 20MB |

### System Performance

- **Total .claude/ size:** <50MB
- **SKILL.md file:** <500 lines
- **Concurrent subagents:** 3-5 max
- **Command invocation:** <100ms response

---

## 10. Verification Checklist

### Before Deployment

- [ ] Claude Code v1.0+ installed
- [ ] Python 3.9+ available
- [ ] Git configured and repository initialized
- [ ] All required directories created
- [ ] All configuration files present (.env, CLAUDE.md, .mcp.json)
- [ ] FastAPI backend running and accessible
- [ ] All 5 subagents created and tested
- [ ] All 5 skills created and discoverable
- [ ] All 7 commands registered
- [ ] No secrets in .claude/ files
- [ ] No breaking changes to existing workflows
- [ ] Team has reviewed and approved

### Runtime Checks

```bash
# Verify backend running
curl http://localhost:8000/api/health

# Verify skills are discoverable
# (Command: "What skills are available?")

# Verify subagents are available
# (Try: "Use code-reviewer to review this code")

# Verify commands registered
# (Type: /rag-search, /review-pr, etc.)

# Check .claude/ directory structure
ls -la .claude/

# Verify no secrets
grep -r "sk-" .claude/
grep -r "Bearer " .claude/
grep -r "secret" .claude/ | grep -v ".md"
```

---

## 11. Maintenance & Updates

### Regular Tasks

**Weekly:**
- Monitor subagent performance
- Check skill usage metrics
- Review error logs
- Address team feedback

**Monthly:**
- Update CLAUDE.md inventory
- Review and archive old agents/skills
- Audit for security issues
- Plan improvements

**Quarterly:**
- Full system review
- Performance optimization
- Documentation update
- Team training refresh

### Troubleshooting

**Skill not discovered:**
```
Check: SKILL.md frontmatter syntax
Check: Description includes keywords
Fix: Reload Claude Code
```

**Subagent not responding:**
```
Check: Tool access allowed
Check: Model availability (sonnet/haiku)
Check: Tool restrictions honored
Fix: Check error logs, resume agent
```

**Command not working:**
```
Check: Registered in CLAUDE.md
Check: Markdown syntax valid
Check: Tool access allowed
Fix: Verify invocation path
```

---

## 12. Deployment Checklist

- [ ] All files created and tested
- [ ] Security review passed (no secrets exposed)
- [ ] Performance benchmarks met
- [ ] Documentation complete
- [ ] Team training completed
- [ ] Version control ready (git commit)
- [ ] Backup of current state
- [ ] Rollback plan documented
- [ ] Monitoring set up
- [ ] Success metrics defined

---

**Status:** REQUIREMENTS SPECIFICATION COMPLETE
**Ready for:** Implementation and deployment
