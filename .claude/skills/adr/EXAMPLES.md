# Architecture Decision Record Examples

Real-world examples of well-written ADRs for learning.

---

## Example 1: Technology Choice - REST API Framework

# ADR-001: Use FastAPI for REST API Backend

**Date:** 2024-01-15
**Status:** Accepted
**Deciders:** Tech Lead (Alice), Architect (Bob)
**Involved:** Backend team, DevOps, Security

---

## Context

We need to build a REST API backend for a new mobile application that will handle user authentication, data retrieval, and content management. The API must:

- Handle 1000+ concurrent requests
- Support real-time features (WebSockets)
- Integrate with PostgreSQL database
- Be maintainable and learnable by the team
- Deploy to AWS

Our team has Python expertise (5 developers), and we're evaluating between:
1. FastAPI (modern, async)
2. Django REST Framework (traditional, mature)
3. Flask (minimal, lightweight)

Time to market is critical - we need API ready in 4 weeks.

---

## Decision

We will use **FastAPI** as the REST API framework for all backend development.

---

## Rationale

FastAPI provides:

- **Native async/await support:** Handles concurrent requests efficiently
- **Automatic documentation:** Generates Swagger UI and OpenAPI schema
- **Type safety:** Pydantic integration validates all inputs/outputs
- **Performance:** Benchmarks show 2-3x faster than Django REST Framework
- **Learning curve:** Python syntax is familiar; async patterns are modern
- **Community growth:** Rapidly growing ecosystem with good tooling

This solves our scalability problem (1000+ concurrent requests) while keeping the team productive (familiar language, less boilerplate).

---

## Consequences

### Positive

- API can handle 1000+ concurrent requests efficiently
- Auto-generated documentation reduces maintenance burden
- Smaller codebase than Django (less boilerplate)
- Built-in request validation via Pydantic
- Real-time features (WebSockets) are easier to implement
- Team learned async patterns applicable to Python generally

### Negative

- Team must learn async/await programming patterns
- Smaller ecosystem than Django (fewer pre-built packages)
- Async debugging is more complex
- Some developers unfamiliar with async patterns initially

---

## Alternatives Considered

### Alternative 1: Django REST Framework

**Description:** Django is a mature, batteries-included framework with extensive ecosystem, built-in ORM, auth, and admin panel.

**Pros:**
- Larger community and ecosystem
- Existing team knowledge (migrating from old Django project)
- Built-in authentication and permissions
- Mature third-party packages
- Excellent documentation

**Cons:**
- Async support added later (feels bolted on)
- 40% slower than FastAPI in load tests
- More boilerplate code
- Includes features we don't need (admin panel, migrations)
- Overkill for API-only project

### Alternative 2: Flask + Custom Code

**Description:** Minimal framework where we build more functionality ourselves.

**Pros:**
- Lightweight, minimal overhead
- Full control over architecture
- Learning opportunity for custom solutions

**Cons:**
- Requires writing more custom code (auth, validation, etc.)
- Less mature for large projects
- Team would be maintaining more code
- Slower development velocity

---

## Related Decisions

- Depends on: ADR-002 (PostgreSQL database choice)
- Influences: ADR-004 (authentication implementation)
- Influences: ADR-005 (API versioning strategy)
- Related: ADR-006 (deployment on AWS)

---

## Implementation Notes

1. Create FastAPI project structure with proper separation of concerns
2. Set up asyncpg for async PostgreSQL access
3. Implement authentication middleware
4. Define Pydantic models for request/response validation
5. Add comprehensive error handling
6. Set up logging and monitoring
7. Deploy to AWS using Docker + ECS

**Timeline:** 4 weeks to production

---

## Monitoring

**Success criteria:**
- API handles 1000+ concurrent requests without degradation
- Response time under 200ms (p95)
- Team velocity maintains or improves
- Zero async-related production bugs

**Review:** Monthly performance metrics, 6-month team retrospective

---

## Approval

| Role | Name | Date | Feedback |
|------|------|------|----------|
| Tech Lead | Alice | 2024-01-15 | Approved, led spike |
| Architect | Bob | 2024-01-15 | Aligns with strategy |
| DevOps | Carol | 2024-01-16 | Deployment confirmed |

**Status:** Accepted ✓

---

---

## Example 2: Architectural Pattern - Layered Architecture

# ADR-003: Use Layered Architecture (Controller-Service-Repository)

**Date:** 2024-01-20
**Status:** Accepted

---

## Context

Our API is growing and needs clear organization. We have:

- 30+ endpoints that need maintenance
- 5 developers working on same codebase
- Need to prevent tight coupling between components
- Want easy testing and code reuse
- New developers need to understand code structure quickly

Questions:
- Should endpoints directly access database?
- Where should business logic live?
- How do we organize code?

---

## Decision

We will organize code into **three layers:**

1. **Controller layer:** HTTP endpoints, request routing
2. **Service layer:** Business logic, validation, orchestration
3. **Repository layer:** Database access, queries

Each layer depends only on layers below it.

---

## Rationale

**Separation of Concerns:**
- Controllers handle HTTP details, nothing else
- Services handle business logic, nothing else
- Repositories handle database queries, nothing else

**Benefits:**
- Easy to test: can mock dependencies in each layer
- Clear responsibility: know where to add code
- Easy to reuse: services can be called from multiple controllers
- Easy to maintain: changes in one layer don't break others

**Example:**
```python
# Controller layer (endpoint)
@app.post("/users")
async def create_user(request: CreateUserRequest, db: AsyncSession):
    service = UserService(UserRepository(db))
    user = await service.create_user(request.email, request.password)
    return UserResponse(user)

# Service layer (business logic)
class UserService:
    def __init__(self, repo: UserRepository):
        self.repo = repo

    async def create_user(self, email: str, password: str):
        # Validation
        if not self._is_valid_email(email):
            raise ValidationError("Invalid email")

        # Business logic
        user = User(email=email, password=hash_password(password))

        # Delegate to repository
        await self.repo.save(user)
        return user

# Repository layer (database)
class UserRepository:
    def __init__(self, db: AsyncSession):
        self.db = db

    async def save(self, user: User):
        self.db.add(user)
        await self.db.commit()
```

---

## Consequences

### Positive

- Easy to test each layer independently
- Code organization is predictable and clear
- Services can be reused from multiple endpoints
- New developers understand code structure quickly
- Changes in database layer don't affect business logic

### Negative

- More files/classes (verbosity)
- Small features have slight overhead (3 files instead of 1)
- Requires discipline (team must follow pattern)
- Learning curve for developers new to layered architecture

---

## Alternatives Considered

### Alternative 1: Anemic Domain Model
Business logic distributed across services, repositories simple.

**Pros:** Simple, flexible
**Cons:** Hard to understand where logic should go, testing harder

### Alternative 2: Clean/Hexagonal Architecture
More complex with additional layers (entities, use cases, adapters).

**Pros:** Very testable, loosely coupled
**Cons:** Over-engineered for our project size, too many layers

### Alternative 3: Monolithic Services
Everything in single endpoint function.

**Pros:** Quick to write initially
**Cons:** Untestable, unmaintainable, code duplication

**Chosen:** Layered architecture strikes right balance of structure vs. simplicity.

---

## Implementation

**File structure:**
```
src/
├── api/
│   └── controllers/
│       ├── user_controller.py
│       ├── course_controller.py
│       └── auth_controller.py
├── services/
│   ├── user_service.py
│   ├── course_service.py
│   └── auth_service.py
├── repositories/
│   ├── user_repository.py
│   ├── course_repository.py
│   └── base_repository.py
└── models/
    ├── user_model.py
    └── course_model.py
```

**Pattern to follow:**
- Controllers only: parse requests, call services, return responses
- Services only: implement business logic, use repositories for data
- Repositories only: database queries, return model instances

---

## Monitoring

**Team follows pattern consistently:**
- Code reviews check for pattern violations
- New developers trained on pattern in onboarding

**Adjust if:**
- Team struggles with pattern (simplify to 2 layers)
- Performance issues arise from extra abstraction (measure first)
- Need emerges for additional layer (change decision explicitly)

---

---

## Example 3: Data Strategy - Database Choice

# ADR-005: Use PostgreSQL as Primary Database

**Date:** 2024-02-01
**Status:** Accepted

---

## Context

We need a primary database for:
- User accounts and authentication data
- Course and content information
- User progress and learning history
- Analytics and reporting

Requirements:
- ACID transactions required
- SQL queries needed
- Moderate scale (1M users eventually)
- Good team knowledge
- Open source preferred

Options: PostgreSQL, MySQL, MongoDB

---

## Decision

We will use **PostgreSQL** as our primary relational database.

---

## Rationale

PostgreSQL provides:
- Strong ACID guarantees (data integrity)
- Advanced SQL features (useful for reporting)
- JSON data type (flexibility when needed)
- Great async support with asyncpg
- Wide adoption and mature ecosystem
- Strong community

---

## Consequences

### Positive
- Data integrity guaranteed
- Rich querying capabilities
- Scales to millions of records easily
- Team experienced with PostgreSQL
- Great async Python driver (asyncpg)

### Negative
- Schema migrations need planning
- Requires schema thinking upfront
- Less flexible than document databases
- Vertical scaling limited (eventually horizontal sharding needed)

---

## Alternatives

### Alternative 1: MongoDB
**Pros:** Flexible schema, easy horizontal scaling, JSON-native
**Cons:** Weaker guarantees, harder to query, eventual consistency

### Alternative 2: MySQL
**Pros:** Simpler, smaller, common knowledge
**Cons:** Fewer features than PostgreSQL, weaker JSON support

**Chosen:** PostgreSQL provides best balance of features, reliability, and team expertise.

---

---

## Example 4: Decision NOT Made (Anti-Pattern)

# ❌ Anti-Pattern: Vague Technology Decision

This is an example of what NOT to do:

```
# ADR-002: Choose Web Framework

Status: Proposed

Context: We need a web framework.

Decision: We will use a web framework.

Rationale: Frameworks help organize code.

Consequences:
- Good: Framework helps organize
- Bad: Requires learning

We considered:
- Option 1
- Option 2
- Option 3

Approval: Pending
```

**Why is this bad?**
- Doesn't specify WHICH framework (FastAPI? Django? Flask?)
- Rationale is too vague ("frameworks help organize")
- Doesn't explain trade-offs or why chosen framework is better
- Consequences don't mention specific impacts
- Doesn't explain why each alternative was rejected

**How to fix:**
- Be specific: "We will use FastAPI"
- Explain why: "...because it's async-first, provides auto-docs..."
- Compare trade-offs: "Django is more mature but 40% slower..."
- Concrete consequences: "Response time will be <200ms instead of 400ms..."

---

## Key Patterns in Good ADRs

### ✅ Good Elements

**Specific decisions:**
- "We will use JWT tokens for authentication"
- "We will use PostgreSQL with async driver (asyncpg)"
- "We will deploy to AWS using ECS/Fargate"

**Clear rationale:**
- "Because [decision] provides [benefit] for [situation]"
- "Solves [problem] better than [alternative]"
- "Aligns with [constraint] and [team expertise]"

**Realistic consequences:**
- Both positive (performance, maintainability)
- Both negative (learning curve, trade-offs)
- Specific and measurable

**Fair alternatives:**
- Each alternative has real pros/cons
- Explain why chosen option is better
- Show alternatives were seriously considered

**Approval trail:**
- Who approved? When?
- Any dissenting opinions?
- Future decision makers identified

---

## Common Decision Types

### 1. Technology Choice
- Framework selection (FastAPI vs Django)
- Database selection (PostgreSQL vs MongoDB)
- Library/package selection

### 2. Architectural Pattern
- Layered vs clean architecture
- Monolith vs microservices
- Synchronous vs async patterns

### 3. Development Approach
- Testing strategy (TDD, BDD, etc.)
- Code organization (package structure)
- Git branching strategy

### 4. Data Management
- Database schema design
- Caching strategy
- Data backup/recovery approach

### 5. Deployment & Operations
- Where to deploy (AWS, GCP, on-premises)
- Container strategy (Docker, Kubernetes)
- Monitoring and alerting approach

---

## When NOT to Write an ADR

❌ **Don't write ADR for:**
- Bug fixes ("Fixed null pointer exception")
- Small feature additions ("Added user avatar field")
- Code style changes ("Used snake_case for variable names")
- Temporary workarounds ("Added try/catch to suppress error")
- Local implementation details ("Used HashSet instead of ArrayList")

✅ **Do write ADR for:**
- Technology selections that affect multiple teams
- Architectural patterns that guide code organization
- Significant trade-offs with long-term impact
- Decisions that are hard to change later
- Decisions that other teams depend on

---

## ADR Metadata Fields

Every ADR should have at minimum:
- **Number/ID:** ADR-001, ADR-002 (sequential)
- **Title:** Clear decision statement
- **Date:** When decision made
- **Status:** Proposed | Accepted | Deprecated | Superseded
- **Deciders:** Who made decision
- **Context:** Why decision matters
- **Decision:** What's being decided
- **Rationale:** Why chosen
- **Consequences:** Positive & negative
- **Alternatives:** What was considered

Optional but helpful:
- **Involved:** Who was consulted
- **Implementation:** How to execute
- **Monitoring:** How to know if working
- **Approval:** Sign-off from stakeholders

---

## Tips for Writing Good ADRs

1. **Write as decisions are made** (not retroactively)
2. **Involve affected people** (architecture, dev teams, operations)
3. **Be fair to alternatives** (explain pros, not just cons)
4. **Be specific** (dates, numbers, actual technologies)
5. **Explain trade-offs** (every decision has costs)
6. **Keep it short** (1-3 pages is typical)
7. **Use examples** (make decisions concrete)
8. **Review & refine** (like code review, but for architecture)
9. **Make decisions when issues arise** (not speculative)
10. **Keep ADRs as living documents** (update when status changes)

---

## ADR Status Lifecycle

```
Proposed → Accepted → (stays accepted OR) Deprecated/Superseded

Proposed: Decision under consideration
Accepted: Decision made and approved
Deprecated: Decision no longer recommended (but may still be in use)
Superseded: Replaced by newer decision (link to ADR-NNN)
```

**Example transition:**
```
ADR-001 (FastAPI): Proposed → Accepted (January 2024)

...6 months later...

ADR-012 (Move to Go): Proposed → Accepted (July 2024)

ADR-001 status changes to: Superseded by ADR-012

Links updated:
- ADR-001: "Superseded by ADR-012: Move API backend to Go"
- ADR-012: "Supersedes ADR-001: Previous FastAPI approach"
```
