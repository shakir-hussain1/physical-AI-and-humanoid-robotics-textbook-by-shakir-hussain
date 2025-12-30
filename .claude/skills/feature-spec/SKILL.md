---
name: feature-spec
description: Create comprehensive feature specifications with clear requirements and acceptance criteria. Use when writing specifications, defining features, or when the user mentions spec or feature definition.
allowed-tools: Read
model: sonnet
---

# Feature Specification Skill

Guide for creating detailed, testable feature specifications.

## Specification Structure

### 1. Overview
- Clear, concise feature description
- Business value/motivation
- Success definition

### 2. Objectives
- Primary goals (2-5 items)
- Success criteria (3-5 measurable items)
- Non-goals/out-of-scope

### 3. Feature Details
- Core functionality
- User interactions
- System behavior
- Edge cases

### 4. Requirements
- Functional requirements (numbered list)
- Non-functional requirements (performance, security, etc.)
- Constraints and limitations

### 5. Acceptance Criteria
- Testable conditions for "done"
- Each criterion should be verifiable
- Include both happy path and edge cases

### 6. Data Models
- Request/response structures
- Data flow between components
- Schema changes if applicable

### 7. API/Interface
- Endpoints (if applicable)
- Parameters and validation rules
- Error handling and status codes

### 8. Testing Strategy
- Unit test coverage areas
- Integration test scenarios
- Edge cases to test

### 9. Known Limitations
- What won't be supported
- Future enhancements
- Deferred work

## Quality Checklist

- [ ] Overview is clear and concise
- [ ] All requirements are testable
- [ ] Acceptance criteria are specific
- [ ] No ambiguity in language
- [ ] Edge cases covered
- [ ] Data models complete
- [ ] API documented (if applicable)

See [TEMPLATE.md](TEMPLATE.md) for specification template.
See [EXAMPLES.md](EXAMPLES.md) for example specifications.
See [CHECKLIST.md](CHECKLIST.md) for detailed verification items.
