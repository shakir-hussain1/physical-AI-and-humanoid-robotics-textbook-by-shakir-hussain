---
name: plan-impl
description: Create detailed implementation plan
invokes: implementation-plan
usage: "/plan-impl Authentication feature from spec"
returns: Task decomposition, timeline, risk analysis
---

# /plan-impl

Create detailed implementation plans with task decomposition, risk analysis, and timelines.

## Usage

```
/plan-impl [What you want to implement]
```

## Examples

### Implement from Specification
```
/plan-impl User authentication system from the specification
```

### Add New Feature
```
/plan-impl Dark mode support across entire UI
```

### Integration Work
```
/plan-impl Integrate Qdrant vector database for RAG search
```

### Refactoring Project
```
/plan-impl Migrate database from MySQL to PostgreSQL
```

### Performance Improvement
```
/plan-impl Optimize database queries for better performance
```

## What Gets Generated

### Scope & Dependencies
- What's in scope
- What's out of scope
- External dependencies
- Blocking factors

### Key Decisions
- Architecture choices explained
- Alternatives considered
- Trade-offs documented
- Principles applied

### Architecture Design
- System components
- Component interfaces
- Data flow
- Deployment model

### Task Decomposition
- Broken into specific tasks (2-16 hours each)
- Acceptance criteria for each task
- Clear dependencies
- Effort estimates

### Timeline & Milestones
- Phased delivery plan
- Weekly milestones
- Critical path analysis
- Buffer time for unknowns

### Risk Analysis
- Top 3-5 risks identified
- Mitigation strategies
- Contingency plans
- Risk owners

### Testing Strategy
- Unit testing approach
- Integration testing plan
- Performance testing (if needed)
- User acceptance testing

### Documentation Plan
- What to document
- When to document
- Who's responsible

## Output Format

```markdown
# Project Implementation Plan

## Executive Summary
[Project overview with key metrics]

## 1. Scope & Dependencies
[What's in/out of scope, external dependencies]

## 2. Key Decisions & Rationale
[Architecture decisions with trade-offs]

## 3. Architecture Design
[System components and interfaces]

## 4. Task Decomposition

### Phase 1: Foundation
[Tasks 1-5 with effort estimates]

### Phase 2: Implementation
[Tasks 6-15 with effort estimates]

### Phase 3: Testing & Polish
[Tasks 16-20 with effort estimates]

## 5. Timeline & Milestones
[Realistic schedule with weekly checkpoints]

## 6. Risk Analysis
[Top risks with mitigation strategies]

## 7. Testing Strategy
[How to verify implementation]

## 8. Documentation Plan
[What gets documented]

## 9. Success Metrics
[How to know if project succeeded]
```

## Plan Quality

Generated plans include:
- ✅ Realistic effort estimates
- ✅ Clear task dependencies
- ✅ Specific acceptance criteria
- ✅ Risk identification and mitigation
- ✅ Milestone-based timeline
- ✅ Testing strategy
- ✅ Buffer time for unknowns

## Example Plans

### Authentication Implementation
```
/plan-impl User authentication with JWT tokens
```

Returns 2-week plan including:
- Database schema tasks
- API endpoint implementation
- Middleware development
- Testing and documentation
- Weekly milestones
- Risk analysis (JWT refresh complexity, etc.)

### Database Migration
```
/plan-impl Migrate from PostgreSQL to MongoDB
```

Returns 4-week plan including:
- Data model redesign
- Schema migration
- Code refactoring
- Testing strategy
- Rollback plan
- Performance validation

### Feature Implementation
```
/plan-impl RAG-powered search system
```

Returns 3-week plan including:
- Vector database setup
- Embedding pipeline
- Search endpoint implementation
- Integration testing
- Performance optimization
- Deployment checklist

## Using Plans

Generated plans help with:
1. **Estimation:** Realistic project timeline
2. **Execution:** Clear task list to follow
3. **Communication:** Status updates to stakeholders
4. **Risk Management:** Identify and mitigate issues
5. **Tracking:** Progress against milestones
6. **Learning:** Retrospective review after completion

## Workflow

1. **Create plan:** `/plan-impl [what to build]`
2. **Review plan:** Check feasibility and timeline
3. **Adjust:** Customize based on team constraints
4. **Approve:** Get team/stakeholder sign-off
5. **Execute:** Follow task breakdown
6. **Track:** Monitor progress against milestones
7. **Learn:** Review actual vs estimated after completion

## Customization

After generation, customize:
- Adjust team size and capacity
- Change timeline if needed
- Modify risk mitigation strategies
- Add team member assignments
- Include specific acceptance criteria
- Adjust performance targets

## Key Principles

**Realism:** Estimates account for unknowns and buffer
**Dependencies:** Clear blocking relationships
**Risk-Aware:** Top risks identified with mitigations
**Measurable:** Clear success metrics
**Achievable:** Team believes plan is realistic

## See Also

- [Implementation Plan Skill](./claude/skills/implementation-plan/)
- [Planning Methodology](./claude/skills/implementation-plan/METHODOLOGY.md)
- [Plan Template](./claude/skills/implementation-plan/TEMPLATE.md)
- [Plan Examples](./claude/skills/implementation-plan/EXAMPLES.md)
- [Project Planning Guide](./guides/project-planning.md)
