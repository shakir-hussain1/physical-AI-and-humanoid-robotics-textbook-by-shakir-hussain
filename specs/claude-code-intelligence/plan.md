# Claude Code Reusable Intelligence - Architecture Plan

## 1. Scope & Dependencies

### In Scope:
- 5 custom subagents with isolated contexts
- 5 custom skills with supporting documentation
- 7 slash commands for quick access
- Integration with existing RAG system
- Version control and team sharing
- Comprehensive documentation

### Out of Scope (Phase 2+):
- Web dashboard for skill management
- Skill marketplace or community sharing
- Advanced skill chaining with data flow
- Skill versioning and rollback
- Performance profiling and caching
- Enterprise approval workflows

### External Dependencies:
- Claude Code CLI (v1.0+)
- Agent SDK integration
- Python 3.9+ (for supporting scripts)
- Git for version control
- MCP server configuration
- GitHub API (for PR integration)

---

## 2. Key Decisions & Rationale

### Decision 1: Subagents vs Skills for Different Tasks

**Options Considered:**
1. All as subagents (isolated context, dedicated resources)
2. All as skills (lightweight, knowledge-based)
3. Hybrid approach (use each where appropriate)

**Chosen:** Hybrid Approach

**Rationale:**
- **Subagents for:** Complex, multi-step tasks needing isolation
  - Code reviewer (detailed analysis, separate context)
  - RAG query expert (complex reasoning, multiple searches)
  - Test runner (long-running, failure recovery)
  - Debugger (hypothesis testing, isolation)
  - Doc generator (sustained output, focused work)

- **Skills for:** Knowledge-based guidance within conversation
  - RAG Query Skill (how-to for queries)
  - PR Review Skill (team standards)
  - Feature Spec Skill (template guidance)
  - Implementation Plan Skill (methodology)
  - ADR Skill (decision documentation)

**Trade-off:**
- Subagents use more resources but provide better isolation
- Skills share context but load faster
- Hybrid gives us flexibility for both scenarios

### Decision 2: Directory Structure (.claude/ vs ~/.claude/)

**Options:**
1. Project-only (.claude/) - team sharing only
2. User-only (~/.claude/) - personal, all projects
3. Both (project + user hierarchy)

**Chosen:** Project-first (.claude/) with user fallback

**Rationale:**
- Team can version control and review agents/skills
- Ensures consistency across team
- Personal ~/.claude/ still available for user experiments
- Clear hierarchy: Project overrides User

**Trade-off:**
- Project .claude/ requires git coordination
- But ensures team alignment on standards

### Decision 3: Slash Commands Implementation

**Options:**
1. Pure shell scripts (.sh files)
2. Markdown files with Claude prompts (.md)
3. Python scripts with full logic
4. Hybrid (markdown + scripts)

**Chosen:** Hybrid (Markdown + Supporting Scripts)

**Rationale:**
- Markdown keeps CLI readable and maintainable
- Scripts handle complex logic validation
- Team can edit without Python knowledge
- Aligns with project structure (Markdown-first)

### Decision 4: Skill Organization & File Size

**Options:**
1. Single large SKILL.md (500+ lines)
2. Split across multiple files
3. SKILL.md + optional supporting files

**Chosen:** SKILL.md + Progressive Disclosure

**Rationale:**
- Keep SKILL.md <500 lines for fast loading
- Supporting files for detailed reference
- Progressive disclosure: essentials → details
- Easier for Claude to maintain context

### Decision 5: Integration with RAG System

**Options:**
1. Separate intelligence system (no integration)
2. Full integration (skills use RAG APIs)
3. Optional integration (skills work standalone or with RAG)

**Chosen:** Optional Integration with RAG Enhancement

**Rationale:**
- RAG Query Skill/Subagent enhance RAG capabilities
- Other agents/skills work independently
- Subagents can call RAG APIs when needed
- Flexibility for future features

---

## 3. Interfaces & API Contracts

### 3.1 Subagent Interface

**Invocation:**
```python
# Automatic
User: "Review this code"
→ Claude recognizes task, invokes code-reviewer subagent

# Explicit
User: "Use the code-reviewer subagent to check this"
→ Direct invocation
```

**Input/Output Contract:**

```markdown
Subagent: code-reviewer
Input: Code snippet or file path
Output:
{
  "critical_issues": [
    {
      "file": "path/to/file",
      "line": 42,
      "severity": "critical",
      "issue": "description",
      "fix": "suggested fix",
      "impact": "why this matters"
    }
  ],
  "warnings": [...],
  "suggestions": [...],
  "summary": "overall assessment",
  "quality_score": 7.5/10
}
```

### 3.2 Skill Interface

**Invocation:**
```
User: "Create a feature specification for user authentication"
→ Claude matches to feature-spec skill description
→ Claude asks: "Use feature-spec skill for this?"
→ User: "Yes"
→ Skill provides TEMPLATE.md and guidance
```

**Input/Output Contract:**

```markdown
Skill: feature-spec
Input: Feature description or requirements
Output:
1. Prompt user with TEMPLATE.md
2. Guide through each section:
   - Overview
   - Objectives
   - Features
   - Requirements
   - Acceptance Criteria
3. Return formatted spec.md
```

### 3.3 Slash Command Interface

**Registration (in CLAUDE.md):**
```yaml
commands:
  - name: /rag-search
    description: Search RAG system directly
    invokes: rag-query skill

  - name: /review-pr
    description: Start PR review workflow
    invokes: code-reviewer subagent
```

**Usage:**
```
/rag-search "humanoid robot dynamics"
→ Invokes rag-query skill
→ Returns formatted search results

/review-pr PR#42
→ Invokes code-reviewer subagent
→ Reviews PR changes
→ Returns detailed review
```

### 3.4 RAG Query API Contract

```bash
# RAG Query Subagent
POST /api/query
Input: {
  "query": "string",
  "user_role": "student|teacher|researcher",
  "conversation_history": []
}
Output: {
  "answer": "string",
  "sources": [{...}],
  "confidence": "high|medium|low",
  "metadata": {
    "latency_ms": int,
    "grounding": bool
  }
}
```

---

## 4. Non-Functional Requirements & Budgets

### Performance Budgets

| Component | Target | Measurement |
|-----------|--------|-------------|
| Subagent startup | <2 sec | Time to first response |
| Skill discovery | <1 sec | Time to identify matching skill |
| Command execution | <100ms | CLI response time |
| Code review | <30 sec | Time per 500 LOC |
| Test execution | <60 sec | Time to run full suite |

### Reliability Targets

| Aspect | Target | Monitoring |
|--------|--------|-----------|
| Subagent availability | 100% | Invocation success rate |
| Skill loading | 99.5% | Graceful fallback on error |
| Command registration | 100% | Startup validation |
| Error recovery | 95% | Fallback to manual approach |

### Resource Constraints

| Resource | Budget | Justification |
|----------|--------|-------------|
| Subagent context | 10K tokens | Isolated, focused work |
| Skill size | <500 lines | Fast discovery loading |
| Total .claude/ size | <50MB | Git repo constraint |
| Supporting scripts | <5MB | Quick execution |

### Security & Access

| Aspect | Approach | Details |
|--------|----------|---------|
| Tool restriction | Explicit allowed-tools | Each subagent/skill limited to necessary tools |
| Sensitive data | Environment variables | No secrets in .claude/ files |
| Version control | Git-tracked | All code reviewed before merge |
| Permission model | Team consensus | Changes require PR review |

---

## 5. Data Management

### Source of Truth:
- **.claude/** directory - Team's canonical intelligence system
- **CLAUDE.md** - Central inventory and configuration
- **specs/** - Feature documentation
- **history/prompts/** - Development records

### Schema:

**Subagent schema (.md file):**
```yaml
---
name: agent-name
description: What it does and when to use
tools: Tool1, Tool2, Tool3
model: sonnet|haiku|opus
---

# System Prompt and Instructions
```

**Skill schema (.md file structure):**
```markdown
---
name: skill-name
description: What it does, trigger keywords
allowed-tools: Tool1, Tool2
model: (optional)
---

# Skill Overview
# Quick Start
# Instructions
# Examples
# Advanced Features
```

### Migration & Rollback:

**Adding new subagent/skill:**
1. Create in .claude/
2. Commit to feature branch
3. Test and validate
4. PR review and approval
5. Merge to main

**Removing/deprecating:**
1. Update CLAUDE.md status
2. Add deprecation notice
3. Document migration path
4. Maintain for 2 weeks
5. Archive or remove

---

## 6. Operational Readiness

### Observability

**Logging:**
```yaml
Subagent invocation:
  - timestamp: 2025-12-28T10:30:00Z
  - subagent_name: code-reviewer
  - input_size: 1500 tokens
  - execution_time: 12.5 seconds
  - result_tokens: 2000
  - status: success

Skill usage:
  - timestamp: 2025-12-28T10:35:00Z
  - skill_name: feature-spec
  - match_confidence: 0.95
  - activation: automatic
  - user_action: accepted
```

**Metrics:**
- Subagent invocations per day
- Average execution time per subagent
- Skill usage frequency
- Command popularity
- Error rates

### Alerting:

```yaml
Triggers:
  - Subagent execution time >30 sec → Check for resource issues
  - Skill discovery failure → Validate SKILL.md format
  - Command registration failure → Verify markdown syntax
  - Tool access denied errors → Review allowed-tools configuration
```

### Runbooks:

**Subagent not responding:**
1. Check if tool is available
2. Verify token limits not exceeded
3. Check error logs for API failures
4. Resume subagent or restart

**Skill not discovered:**
1. Verify description matches user query
2. Check SKILL.md syntax (YAML frontmatter)
3. Reload Claude Code
4. Add keywords to description

**Command not working:**
1. Verify command registered in CLAUDE.md
2. Check markdown syntax
3. Verify tool access (allowed-tools)
4. Test in isolation

### Deployment Strategy:

1. **Local development:** Create in .claude/ locally
2. **Team testing:** Push to feature branch
3. **Review:** PR with skill/agent review checklist
4. **Approval:** Team consensus
5. **Release:** Merge to main
6. **Rollout:** Team pulls latest
7. **Monitoring:** Watch for issues

### Rollback:

```bash
# If new skill/agent causes issues
git revert <commit>
git push origin main

# Team pulls latest
git pull origin main

# Affects take effect immediately (no restart needed)
```

---

## 7. Risk Analysis & Mitigation

### Risk 1: Subagent Context Isolation Too Heavy

**Impact:** Performance degradation with many concurrent subagents
**Probability:** Medium
**Blast Radius:** All subagent-based workflows

**Mitigation:**
- Limit concurrent subagents to 3-5
- Use Haiku model for lightweight subagents
- Monitor execution time and token usage
- Queue long-running tasks

### Risk 2: Skill Discovery Doesn't Work as Expected

**Impact:** Users don't find/use valuable skills
**Probability:** Medium
**Blast Radius:** Skill adoption and effectiveness

**Mitigation:**
- Write killer descriptions with keywords
- Test discovery before committing
- Provide "/skill-name" shortcut commands
- Document in README with examples
- Gather user feedback and iterate

### Risk 3: Tool Restrictions Too Limiting

**Impact:** Subagents/skills can't do what's needed
**Probability:** Low
**Blast Radius:** Specific subagent capabilities

**Mitigation:**
- Iterate on allowed-tools based on needs
- Start permissive, restrict if needed
- Document rationale for restrictions
- Ask for permission when needed

### Risk 4: CLAUDE.md Becomes Outdated

**Impact:** Inventory doesn't match actual agents/skills
**Probability:** Medium
**Blast Radius:** Team discoverability

**Mitigation:**
- Add updating CLAUDE.md to PR checklist
- Automate inventory generation if possible
- Regular audit (monthly)
- Link to CLAUDE.md in README

### Risk 5: Large .claude/ Directory

**Impact:** Slow git operations, large clone
**Probability:** Low
**Blast Radius:** Developer experience

**Mitigation:**
- Keep supporting files small
- Use scripts instead of embedding data
- Archive old/unused agents/skills
- Target <50MB total size

---

## 8. Evaluation & Validation

### Definition of Done:

- [ ] All 5 subagents created and tested
- [ ] All 5 skills created and discoverable
- [ ] All 7 commands registered and working
- [ ] Integrated with RAG system
- [ ] CLAUDE.md updated with inventory
- [ ] All files version-controlled
- [ ] Documentation complete (spec, plan, tasks, requirements)
- [ ] Team can discover and use all components
- [ ] No breaking changes to existing workflows
- [ ] Security review passed

### Testing Strategy:

**Subagent Testing:**
```bash
# Test code-reviewer on sample code
/review-pr sample_code.py

# Test rag-query on knowledge questions
/rag-search "humanoid robot dynamics"

# Test test-runner on test suite
/run-tests

# Test debugger on error
/debug error_message.txt

# Test doc-generator on codebase
/gen-docs
```

**Skill Testing:**
```bash
# Verify skill is discovered
> List available skills
→ Should see all 5 skills

# Verify skill activation
> Create a feature specification for [feature]
→ Claude should offer feature-spec skill

# Verify skill output
> Yes, use feature-spec skill
→ Should produce formatted specification
```

**Integration Testing:**
```bash
# RAG Query Skill uses RAG API
> /rag-search "about ROS2"
→ Should return grounded answers from RAG

# Subagents can access needed tools
> /review-pr PR#42
→ Should read files, run linters, etc.

# Commands properly invoke agents/skills
> /write-spec "new authentication"
→ Should invoke feature-spec skill properly
```

### Acceptance Criteria:

**Subagents:** ✅ All working with proper isolation and output
**Skills:** ✅ All discoverable and providing value
**Commands:** ✅ All registered and functional
**Integration:** ✅ Works with RAG and existing systems
**Documentation:** ✅ Complete and current
**Team Adoption:** ✅ >80% team using regularly

---

## 9. Timeline & Milestones

### Milestone 1: Foundation (Week 1)
- [ ] Create directory structure (.claude/)
- [ ] Create spec, plan, tasks documentation
- [ ] Create first subagent (code-reviewer)
- [ ] Create first skill (rag-query)
- [ ] Update CLAUDE.md

### Milestone 2: Core Agents & Skills (Week 2)
- [ ] Create RAG Query subagent
- [ ] Create Test Runner subagent
- [ ] Create PR Review skill
- [ ] Create Feature Spec skill
- [ ] Add to version control

### Milestone 3: Extended Components (Week 3)
- [ ] Create Debugger subagent
- [ ] Create Doc Generator subagent
- [ ] Create Implementation Plan skill
- [ ] Create ADR skill
- [ ] Register all slash commands

### Milestone 4: Integration & Testing (Week 4)
- [ ] Test all subagents
- [ ] Test all skills
- [ ] Verify RAG integration
- [ ] Team testing and feedback
- [ ] Documentation finalization

### Milestone 5: Rollout (Week 5)
- [ ] Team training
- [ ] Deploy to production
- [ ] Monitor usage
- [ ] Gather feedback
- [ ] Plan Phase 2 enhancements

---

## 10. Architecture Decisions Captured

**ADR Candidates:**
1. Hybrid Subagents + Skills approach
2. Project-scoped (.claude/) directory structure
3. Markdown-first slash commands
4. Optional RAG integration strategy
5. Progressive disclosure for skill documentation

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-28 | Initial architecture plan |

---

**Status:** ARCHITECTURE PLAN COMPLETE
**Ready for:** Task decomposition and implementation
