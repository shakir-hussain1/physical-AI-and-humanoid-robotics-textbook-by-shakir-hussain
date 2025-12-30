---
name: write-spec
description: Create a comprehensive feature specification
invokes: feature-spec
usage: "/write-spec User authentication with JWT tokens"
returns: Formatted specification with acceptance criteria
---

# /write-spec

Create comprehensive feature specifications with clear requirements and acceptance criteria.

## Usage

```
/write-spec [Feature description or name]
```

## Examples

### Simple Feature
```
/write-spec Add favorite button to course cards
```

### Complex Feature
```
/write-spec Implement RAG-powered semantic search across all courses
```

### Integration Feature
```
/write-spec Integrate Qdrant vector database for similarity search
```

### API Feature
```
/write-spec Create user profile API endpoints (GET, PUT, DELETE)
```

## What Gets Generated

### Overview Section
- One-sentence feature description
- Business value and motivation
- Success definition

### Objectives Section
- 2-5 primary goals
- 3-5 measurable success criteria
- Out of scope items

### Feature Details Section
- User interaction scenarios
- System behavior specification
- Edge cases and error handling

### Requirements Section
- Functional requirements (numbered list)
- Non-functional requirements (performance, security, etc.)
- Constraints and limitations

### Acceptance Criteria Section
- Happy path scenarios
- Error handling scenarios
- Performance criteria
- Accessibility requirements

### Data Models Section
- Request/response examples
- Database schema changes
- Data flow diagrams

### API/Interface Design Section
- REST endpoints defined
- Parameters and validation
- Error codes and responses
- UI components (if applicable)

### Testing Strategy Section
- Unit testing approach
- Integration testing scenarios
- Edge case coverage

### Known Limitations Section
- What won't be supported
- Deferred to future phases
- Technical constraints

## Output Format

```markdown
# Feature Name

## Overview
[Concise description of feature]

## Objectives

### Primary Goals
1. [Goal 1]
2. [Goal 2]
3. [Goal 3]

### Success Criteria
- [ ] [Criterion 1]
- [ ] [Criterion 2]
- [ ] [Criterion 3]

### Out of Scope
- [Item 1]
- [Item 2]

## Feature Details
[Detailed scenarios and behavior]

## Functional Requirements
| # | Requirement | Priority |
...

## Non-Functional Requirements
| Aspect | Requirement |
...

## Acceptance Criteria
### Happy Path
- [ ] [Criterion 1]

### Error Handling
- [ ] [Criterion 1]

## Data Models
[Request/response examples]

## API/Interface Design
[Endpoint specifications]

## Testing Strategy
[What to test]

## Known Limitations
[Constraints and deferred work]
```

## Specification Quality

Generated specifications include:
- ✅ Clear, testable requirements
- ✅ Concrete acceptance criteria
- ✅ Error handling scenarios
- ✅ Performance targets
- ✅ Security considerations
- ✅ Accessibility requirements
- ✅ Real working examples

## Example Specifications

### Simple Feature Spec
```
/write-spec Add dark mode toggle to settings
```

Returns spec for a simple UI feature with:
- Requirements for dark mode
- CSS considerations
- Acceptance criteria
- Testing approach

### Complex Feature Spec
```
/write-spec Build multi-step signup flow with email verification
```

Returns comprehensive spec for complex feature:
- Multiple user journey scenarios
- Email service integration
- Error recovery
- Security considerations
- Testing strategy

### API Feature Spec
```
/write-spec Create REST API for course search and filtering
```

Returns API specification:
- Endpoint definitions
- Query parameter documentation
- Response format with examples
- Error handling
- Rate limiting

## Using Specifications

Generated specs help with:
1. **Planning:** Clear requirements for implementation
2. **Testing:** Acceptance criteria to verify
3. **Communication:** Specification for stakeholders
4. **Design:** Understanding feature boundaries
5. **Implementation:** Clear starting point for development

## Workflow

1. **Write spec:** `/write-spec [feature description]`
2. **Review spec:** Check completeness and clarity
3. **Clarify:** Ask questions if needed
4. **Approve:** Get stakeholder sign-off
5. **Implement:** Use spec as reference
6. **Test:** Use acceptance criteria to verify

## Customization

After generation, customize:
- Adjust success criteria
- Add specific performance targets
- Include company/product context
- Add screenshots or mockups
- Refine technical details

## See Also

- [Feature Specification Skill](./claude/skills/feature-spec/)
- [Specification Template](./claude/skills/feature-spec/TEMPLATE.md)
- [Specification Examples](./claude/skills/feature-spec/EXAMPLES.md)
- [Writing Good Specifications](./guides/specifications.md)
