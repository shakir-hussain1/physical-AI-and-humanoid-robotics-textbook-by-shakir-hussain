---
name: implementation-plan
description: Create detailed implementation plans with task decomposition, risk analysis, and timeline. Use when planning implementation, defining architecture, or when the user mentions planning or designing features.
allowed-tools: Read
model: sonnet
---

# Implementation Plan Skill

Guide for creating comprehensive implementation plans.

## Plan Structure

### 1. Scope & Dependencies
- In scope items
- Out of scope items
- External dependencies
- Risk assessment

### 2. Key Decisions
- Design decisions with rationale
- Alternatives considered
- Trade-offs evaluated
- Principles applied

### 3. Architecture Design
- System components
- Interfaces and contracts
- Data flow
- Deployment model

### 4. Task Decomposition
- Break into specific tasks
- Identify dependencies
- Estimate effort
- Assign owners

### 5. Timeline & Milestones
- Phased delivery
- Key checkpoints
- Dependencies between phases
- Realistic scheduling

### 6. Risk Analysis
- Top 3-5 risks
- Impact and probability
- Mitigation strategies
- Contingency plans

### 7. Testing Strategy
- Unit testing approach
- Integration testing
- Performance testing
- Acceptance criteria

### 8. Documentation Plan
- What to document
- When to document
- Documentation owners
- Update cadence

## Planning Principles

✅ **GOOD Plans:**
- Realistic task estimates (2-16 hours per task)
- Clear dependencies
- Defined success metrics
- Risk-aware approach
- Regular milestones

❌ **POOR Plans:**
- Too vague ("implement feature")
- Unrealistic estimates
- Missing dependencies
- No risk analysis
- Unclear success criteria

## Task Quality

**Well-Defined Tasks:**
```
Create authentication endpoint
- Input validation for credentials
- Database query for user
- JWT token generation
- Error handling for invalid users
Estimate: 8 hours
Dependency: Database schema finalized
```

**Poorly-Defined Tasks:**
```
Implement authentication
Estimate: 40 hours (too vague)
```

See [METHODOLOGY.md](METHODOLOGY.md) for planning methodology.
See [TEMPLATE.md](TEMPLATE.md) for plan template.
See [EXAMPLES.md](EXAMPLES.md) for example plans.
