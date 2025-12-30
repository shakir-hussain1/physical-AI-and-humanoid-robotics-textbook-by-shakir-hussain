---
name: adr-documentation
description: Document architectural decisions with context, rationale, and consequences. Use when making significant architecture decisions or when the user mentions ADR or architecture documentation.
allowed-tools: Read
model: sonnet
---

# Architecture Decision Record (ADR) Skill

Guide for documenting significant architectural decisions.

## ADR Structure

### 1. Title
- Clear, descriptive title
- Noun phrase format
- Example: "Use FastAPI for backend API"

### 2. Status
- Proposed | Accepted | Deprecated | Superseded

### 3. Context
- What is the issue we're addressing?
- Why does it matter?
- What are the constraints?

### 4. Decision
- Clear statement of the decision
- "We will [decision]"
- Active voice, specific

### 5. Rationale
- Why this decision?
- What problem does it solve?
- How does it improve the system?

### 6. Consequences
**Positive:**
- Benefits and advantages
- Improved capabilities

**Negative:**
- Trade-offs and compromises
- Maintenance burden
- Learning curve

### 7. Alternatives Considered
- Alternative 1: [description]
  - Pros: ...
  - Cons: ...
- Alternative 2: [description]
  - Pros: ...
  - Cons: ...

### 8. Related Decisions
- Links to related ADRs
- Dependencies on other decisions

## When to Create ADR

✅ **Create ADR for:**
- Technology/framework selection
- Architectural patterns
- Data model decisions
- API design choices
- Deployment strategy
- Major refactoring

❌ **Don't create ADR for:**
- Bug fixes
- Small feature additions
- Code style changes
- Temporary workarounds

## Quality Checklist

- [ ] Title is clear and descriptive
- [ ] Context section explains the problem
- [ ] Decision statement is unambiguous
- [ ] Rationale is well-reasoned
- [ ] Consequences fully explored
- [ ] Alternatives evaluated fairly
- [ ] No jargon without explanation
- [ ] Future team can understand reasoning

See [TEMPLATE.md](TEMPLATE.md) for ADR template.
See [EXAMPLES.md](EXAMPLES.md) for example ADRs.
See [GUIDELINES.md](GUIDELINES.md) for detailed guidelines.
