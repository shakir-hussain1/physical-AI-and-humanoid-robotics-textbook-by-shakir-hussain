# ADR Writing Guidelines

Guidance for creating effective Architecture Decision Records that document significant decisions.

---

## What is an ADR?

An **Architecture Decision Record (ADR)** documents an important architectural decision that your team has made.

Not a memo or email, but a **formal record** that:
- Explains what decision was made
- Why it was made (rationale)
- What trade-offs were considered
- How it affects the system

**Purpose:** Help the team (and future maintainers) understand *why* the system is designed a certain way.

---

## When to Write an ADR

### ✅ Write an ADR for:

**Technology Decisions**
- Selecting a framework (FastAPI, Django, Flask)
- Choosing a database (PostgreSQL, MongoDB, Redis)
- Picking a cloud provider (AWS, GCP, Azure)
- Choosing libraries/packages (ORM, testing, etc.)

**Architectural Patterns**
- Layered vs clean architecture
- Monolith vs microservices
- Synchronous vs async programming
- Request-response vs event-driven

**Data Decisions**
- Database schema design approaches
- Caching strategy (in-memory, Redis, etc.)
- Data replication strategy
- Backup and recovery approach

**Development Approach**
- Code organization and structure
- Testing strategy (unit, integration, E2E)
- Git branching/versioning strategy
- Documentation approach

**Deployment & Operations**
- Where to deploy (cloud, on-prem, hybrid)
- Container strategy (Docker, Kubernetes)
- Continuous integration/deployment approach
- Monitoring and alerting strategy

**Guiding Principle:** ADRs are for decisions that:
1. Affect multiple systems or teams
2. Have long-term impact (hard to change later)
3. Involve significant trade-offs
4. Are strategic, not tactical

### ❌ Don't write an ADR for:

- Bug fixes or small patches
- Local implementation details (HashSet vs ArrayList)
- Code style preferences
- Temporary workarounds
- Feature additions (unless they change architecture)
- Developer preference without team impact

---

## ADR Process

### 1. Recognize the Decision

**When:**
- Architecture question comes up
- Multiple viable approaches exist
- Decision will impact many people
- Team debate happens

**Example:**
```
Engineer A: "Should we use FastAPI or Django?"
Engineer B: "Let's compare..."
Architect: "This needs an ADR"
```

### 2. Discuss & Evaluate

**What to discuss:**
- What problem are we solving?
- What options exist?
- What are pros/cons of each?
- What are team's constraints?
- What does team expertise suggest?

**Participants:**
- Technical decision-makers (architects, leads)
- Affected developers (who'll implement)
- Stakeholders (DevOps, QA, etc.)

**Duration:**
- Small decision: 1-2 hours discussion
- Large decision: 1-2 days evaluation (spike tasks)

### 3. Draft ADR

**Format:** Use the [TEMPLATE.md](./TEMPLATE.md) provided

**Key sections:**
1. Context (what problem)
2. Decision (what chose)
3. Rationale (why chosen)
4. Consequences (trade-offs)
5. Alternatives (what we considered)

### 4. Review & Discussion

**Who should review:**
- Architect/tech lead
- Affected team members
- Operations/DevOps (if infrastructure decision)
- Security (if security decision)

**What to check:**
- Is context clear?
- Is decision unambiguous?
- Are alternatives fairly evaluated?
- Are consequences realistic?
- Will team understand this in 2 years?

### 5. Approval

**Who approves:**
- Tech lead or architect (decision authority)
- Stakeholders who depend on decision

**Record approval:**
```
| Role | Name | Date | Notes |
|------|------|------|-------|
| Tech Lead | Alice | 2024-01-15 | Approved |
| DevOps | Bob | 2024-01-15 | Deployment ready |
```

### 6. Communicate

**Share with:**
- Entire team (all-hands, Slack, etc.)
- Dependent teams
- New team members (in onboarding)
- Archive in shared wiki/repository

**Message:**
```
FYI: Team decided to use FastAPI for backend.
See: history/adr/001-fastapi-decision.md
Questions? Ask [person].
```

### 7. Monitor & Review

**Periodic check-ins:**
- Is decision still working well?
- Are we achieving the expected benefits?
- Have circumstances changed?
- Should we reconsider?

**Trigger reviews if:**
- 6 months have passed (annual review)
- New information changes the equation
- Technology landscape evolves
- Team composition changes

---

## Writing Guidelines

### Structure & Format

**Use the template**, with sections:

1. **Title** - Clear, specific noun phrase
   - ✅ Good: "Use FastAPI for REST API"
   - ❌ Bad: "Web Framework Selection"

2. **Metadata** - Date, status, participants
   - Shows who made decision when
   - Helps track evolution

3. **Context** - What problem, why it matters
   - Describe situation requiring decision
   - Explain constraints and requirements
   - Answer: "Why are we making this decision?"

4. **Decision** - What's being decided
   - Use "We will..." format
   - Be specific (not "use a framework" but "use FastAPI")
   - Make it clear what was chosen

5. **Rationale** - Why this decision
   - Explain how decision solves the problem
   - Address constraints mentioned in context
   - Answer: "Why is this better?"

6. **Consequences** - Both good and bad
   - Positive: benefits, improvements
   - Negative: trade-offs, costs, challenges
   - Learning curve, maintenance burden
   - Be honest about downsides

7. **Alternatives** - What else was considered
   - At least 2 alternatives per decision
   - Fair description of each (pros AND cons)
   - Explain why chosen option is better
   - Show alternatives were seriously evaluated

8. **Related Decisions** - Links to related ADRs
   - What decisions depend on this?
   - What decisions does this depend on?
   - Shows architecture is interconnected

### Length Guidelines

- Small decision: 1-2 pages
- Medium decision: 2-4 pages
- Large/complex decision: 4-6 pages

**Aim for conciseness:** Too long = people won't read. Too short = missing important context.

### Language Guidelines

**Use clear, direct language:**

✅ **Good:**
```
FastAPI is async-first, handling 1000+ concurrent requests
efficiently. This solves our scalability need.
```

❌ **Bad:**
```
FastAPI might be better for concurrent things maybe.
```

**Avoid jargon without explanation:**

✅ **Good:**
```
ACID transactions (atomic, consistent, isolated, durable)
guarantee data integrity even if system crashes.
```

❌ **Bad:**
```
ACID compliance is important for databases.
```

**Be specific with numbers:**

✅ **Good:**
```
FastAPI handles 40% more requests per second than Django
in our load tests (5,000 req/s vs 3,500 req/s).
```

❌ **Bad:**
```
FastAPI is faster than Django.
```

### Tone Guidelines

**Be objective, not promotional:**

✅ **Good:**
```
Positive: Faster performance, async support, auto-documentation
Negative: Smaller community, requires learning async patterns
```

❌ **Bad:**
```
FastAPI is amazing and everyone should use it! Django is terrible.
```

**Be fair to alternatives:**

✅ **Good:**
```
Django Alternative:
Pros: Larger community, more packages, team familiarity
Cons: 40% slower, async support feels bolted-on
Why not chosen: Performance is critical for our use case
```

❌ **Bad:**
```
Django is slow and no one uses it.
```

---

## Content Guidelines

### Context Section

**Must answer:**
- What is the problem we're facing?
- Why does it matter?
- What are our constraints?
- What options are on the table?

**Example template:**
```
We [situation]. This affects [X users / Y teams / critical path].

Our constraints:
- Must handle [X concurrent users]
- Must integrate with [existing system]
- Team expertise in [technology]
- Must be deployed by [date]

Options under consideration:
1. [Option A] - [brief description]
2. [Option B] - [brief description]
3. [Option C] - [brief description]
```

### Rationale Section

**Explain the "why":**
- How does this solve the problem?
- How does it address constraints?
- What benefits does it provide?
- Why is it better than alternatives?

**Don't just list features:**

❌ **Bad rationale:**
```
FastAPI has async/await support, Pydantic validation,
automatic API documentation, and good performance.
```

✅ **Good rationale:**
```
FastAPI solves our scalability problem:
- Async/await handles 1000+ concurrent requests (requirement)
- Pydantic validation reduces boilerplate code (productivity)
- Auto-documentation reduces maintenance (team efficiency)
- 40% faster than alternative (performance requirement)

All align with our constraint of 4-week timeline.
```

### Consequences Section

**Be comprehensive:**

✅ **Good:**
```
Positive:
- Handles required concurrency
- Team learns modern async patterns (valuable long-term)
- Smaller codebase than alternatives

Negative:
- Team must learn async/await (3-5 days ramp-up)
- Smaller community than Django (fewer packages)
- Async debugging is more complex

Maintenance:
- Requires developers familiar with async
- Some uncommon edge cases in async implementation
```

❌ **Bad:**
```
Positive: Fast, good documentation
Negative: Small community
```

### Alternatives Section

**For each alternative:**

1. **Description** - What is it?
2. **Pros** - What's good about it?
3. **Cons** - What's bad about it?
4. **Why not chosen** - Why did we pick something else instead?

**Be fair:**

✅ **Fair:**
```
Django Alternative:
Pros: Larger community, more packages, team familiarity
Cons: Async support feels bolted-on, 40% slower
Why not chosen: Performance is critical; async requirements
favor FastAPI's native async/await support
```

❌ **Unfair:**
```
Django: Old technology, slow, no one uses it.
```

---

## ADR Quality Checklist

Before publishing an ADR, verify:

### Context & Decision
- [ ] Context explains the problem clearly
- [ ] Decision is specific (not vague)
- [ ] Decision addresses the problem stated in context
- [ ] Reader understands "why" this decision matters

### Rationale
- [ ] Explains how decision solves the problem
- [ ] Addresses constraints mentioned in context
- [ ] Rationale is compelling (would convince smart person)
- [ ] Not just listing features, but explaining benefits

### Consequences
- [ ] Positive consequences are concrete and specific
- [ ] Negative consequences are honest and realistic
- [ ] Both pros and cons are mentioned
- [ ] Maintenance burden is acknowledged
- [ ] Learning curve is realistic

### Alternatives
- [ ] At least 2 alternatives described
- [ ] Each alternative is fairly described
- [ ] Pros and cons of each are explained
- [ ] Explanation of why chosen option is better
- [ ] Reader understands alternatives were seriously considered

### Form & Language
- [ ] Title is clear and specific
- [ ] No jargon without explanation
- [ ] Language is active voice ("we will..." not "we might...")
- [ ] No obvious typos or grammar errors
- [ ] Numbers are specific, not vague ("1000 req/s" not "many requests")

### Completeness
- [ ] All required sections present
- [ ] Related decisions linked (if any)
- [ ] Approval from relevant stakeholders
- [ ] Clear status (Proposed / Accepted / Superseded)
- [ ] Ready to share with full team

---

## Managing ADR Evolution

### When Circumstances Change

If a decision is no longer optimal:

**Option 1: Update Status to "Deprecated"**
- Decision is no longer recommended
- But still in use in parts of system
- Document why deprecated
- Plan migration to better approach

**Option 2: Create New ADR "Supersedes"**
- New decision better addresses situation
- Old ADR stays for historical context
- New ADR links to and explains supersedes old
- Migration path documented

**Example:**
```
Original ADR-001 (2024-01): Use FastAPI
Status: Accepted

...6 months later...

New ADR-008 (2024-07): Migrate API to Go
Status: Accepted
Note: Supersedes ADR-001 due to performance requirements

ADR-001 updated:
Status: Superseded by ADR-008
Reason: Performance benchmarks showed Go necessary for 10K req/s goal
```

### Making ADR Corrections

If you notice errors or missing information:

1. Minor corrections (typos, grammar): Fix directly
2. Factual corrections: Update with note about change
3. Major rewrites: Create new version, explain changes
4. Post-decision insights: Add to "Implementation Notes" or "Monitoring"

---

## Communicating ADRs

### Sharing with Team

**When publishing ADR:**

```
Subject: ADR-001: Use FastAPI for REST API

Hi team,

We've made a decision on our REST API framework.
Please read: [link to ADR]

Key points:
- We're using FastAPI
- Solves our scalability problem
- Team will need to learn async patterns (3-5 days)
- Questions? Ask [tech lead name]

Next steps:
- Spike task (3 days) to validate approach
- Start implementation by [date]
```

### New Team Member Onboarding

**Include in onboarding:**
- "Read these ADRs to understand why system is designed this way"
- Point to related decisions
- Explain any historical context
- Answer questions about design choices

### Stakeholder Communication

**For non-technical stakeholders:**

❌ **Don't do:**
```
We chose FastAPI because it has better async/await support
and provides automatic OpenAPI documentation generation.
```

✅ **Do:**
```
We chose FastAPI because:
- It handles our expected user volume (1000+ concurrent users)
- Our team can implement features faster (less boilerplate code)
- It costs same as alternatives but performs better
- It's open source with no licensing costs
```

---

## Common Mistakes & How to Avoid Them

### ❌ Mistake 1: Deciding after implementation

**Problem:** ADR written long after decision made, loses context

**Solution:** Write ADR as decision is being made, not after

**Prevention:**
- Recognize when decision point happens
- Draft ADR during discussion
- Refine while fresh

### ❌ Mistake 2: Unfair to alternatives

**Problem:** Only lists cons of alternatives, cons of chosen

**Solution:** Be fair; every option has pros AND cons

**Prevention:**
```
For each alternative:
- List 2+ pros
- List 2+ cons
- Explain why chosen is better
```

### ❌ Mistake 3: Too vague

**Problem:** "We chose a framework" (which one?!)

**Solution:** Be specific - "We chose FastAPI v0.100.0"

**Prevention:**
- Specific technology names and versions
- Concrete numbers (not "fast", but "<200ms response")
- Clear decision statement

### ❌ Mistake 4: Missing context

**Problem:** Reader can't understand why decision matters

**Solution:** Explain the problem and constraints

**Prevention:**
- Ask: "Why is this decision needed?"
- Ask: "What would happen if we chose wrong?"
- Explain impact and importance

### ❌ Mistake 5: No monitoring plan

**Problem:** Decision made but never reviewed, become outdated

**Solution:** Plan for periodic review

**Prevention:**
- Include review schedule in ADR
- Define success metrics
- Set reminder to revisit

---

## ADR Repository Structure

**Recommended organization:**

```
history/adr/
├── INDEX.md                    # List of all ADRs
├── 001-fastapi-framework.md   # Technology decision
├── 002-postgresql-database.md  # Data decision
├── 003-layered-architecture.md # Pattern decision
├── 004-jwt-authentication.md   # Security decision
├── 005-aws-deployment.md       # Ops decision
├── 006-docker-containerization.md
└── TEMPLATES/
    ├── decision-template.md     # Template to use
    └── superseded-example.md    # Example of superseded ADR
```

**Why numbered?**
- Makes references easier (ADR-001 vs long filenames)
- Shows order decisions were made
- Historic record of evolution

---

## Key Principles

### 1. Document Decisions While Fresh

Don't wait; write ADR when decision is made.

### 2. Be Fair to Alternatives

Every option has trade-offs. Be honest about them.

### 3. Think Long-Term

Decisions should guide team for years. Explain future maintainers the "why."

### 4. Include Stakeholders

Get input from people affected by decision.

### 5. Communicate Clearly

Use language team understands. Avoid jargon.

### 6. Monitor & Review

Revisit decisions periodically. Update if circumstances change.

### 7. Keep ADRs as Living Documents

When implementation reveals new information, update ADR.

---

## Further Reading

- [TEMPLATE.md](./TEMPLATE.md) - Template to copy
- [EXAMPLES.md](./EXAMPLES.md) - Real examples to learn from
- [Architecture Decision Record guide (Akita)](https://akita.software/blog/adr-decision-log)
- [ADR GitHub topic](https://github.com/topics/adr) - Real ADRs from projects
