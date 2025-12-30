# Claude Code Intelligence Commands

Quick-access commands for common workflows.

## Available Commands

### /rag-search
Search the RAG knowledge base for Physical AI and robotics information.
**Invokes:** rag-query skill
**Usage:** `/rag-search What is ROS2?`
**Returns:** Grounded answers with sources and confidence

### /review-pr
Start professional PR review with team standards.
**Invokes:** code-reviewer subagent
**Usage:** `/review-pr PR#42` or provide code directly
**Returns:** Quality, security, architecture assessment

### /run-tests
Run test suite and fix any failures.
**Invokes:** test-runner subagent
**Usage:** `/run-tests` or `/run-tests tests/test_module.py`
**Returns:** Test results, coverage report, fixed tests

### /debug
Start debugging workflow for errors and issues.
**Invokes:** debugger subagent
**Usage:** `/debug [error message or stack trace]`
**Returns:** Root cause analysis, fix, prevention recommendations

### /gen-docs
Generate documentation for code or features.
**Invokes:** doc-generator subagent
**Usage:** `/gen-docs API documentation for query endpoint`
**Returns:** Formatted documentation, examples, guides

### /write-spec
Create a comprehensive feature specification.
**Invokes:** feature-spec skill
**Usage:** `/write-spec User authentication with JWT tokens`
**Returns:** Formatted specification with acceptance criteria

### /plan-impl
Create detailed implementation plan.
**Invokes:** implementation-plan skill
**Usage:** `/plan-impl Authentication feature from spec`
**Returns:** Task decomposition, timeline, risk analysis

---

## Command Quick Reference

| Command | Purpose | Agent/Skill | Response Time |
|---------|---------|------------|---------------|
| /rag-search | Knowledge retrieval | rag-query | <5s |
| /review-pr | Code review | code-reviewer | 30-60s |
| /run-tests | Test automation | test-runner | 60-120s |
| /debug | Debugging | debugger | 30-60s |
| /gen-docs | Documentation | doc-generator | 30-60s |
| /write-spec | Specification | feature-spec | <5s |
| /plan-impl | Planning | implementation-plan | <5s |

---

**All commands available and ready to use!**
