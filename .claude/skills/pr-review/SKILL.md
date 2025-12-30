---
name: pr-review-standard
description: Team-standard PR review approach with quality, security, and architecture checks. Use when reviewing pull requests, examining code changes, or when the user mentions code review or pull requests.
allowed-tools: Read, Grep
model: sonnet
---

# PR Review Standard Skill

Provides standardized PR review criteria and quality checks aligned with team standards.

## Review Categories

### Quality Review (40%)
- Code clarity and readability
- Proper naming conventions
- DRY principle (no duplication)
- Appropriate abstraction levels
- Proper error handling
- Code organization and structure

### Security Review (30%)
- No hardcoded secrets/credentials
- Input validation present
- Authentication/authorization checks
- SQL injection prevention
- XSS prevention
- Data privacy considerations

### Architecture Review (20%)
- Follows project patterns
- Minimal dependencies
- Proper separation of concerns
- Backward compatibility
- Performance implications
- Scalability considerations

### Testing Review (10%)
- Unit test coverage (>80%)
- Edge cases tested
- Integration tests present
- No skipped tests
- Proper test isolation

## Quality Checklist

**Critical Issues (Must Fix):**
- [ ] Security vulnerabilities present
- [ ] Data loss/corruption risk
- [ ] Breaking changes without migration
- [ ] Missing error handling
- [ ] No test coverage for changes

**Warnings (Should Fix):**
- [ ] Performance regression
- [ ] Code duplication
- [ ] Missing documentation
- [ ] Overly complex logic
- [ ] Poor naming

**Suggestions (Nice to Have):**
- [ ] Refactoring opportunities
- [ ] Better error messages
- [ ] Code style improvements
- [ ] Additional test cases

## Review Template

```
## Code Review Summary

**Quality:** 8.5/10
**Security:** 9/10
**Architecture:** 8/10
**Testing:** 8.5/10
**Overall:** 8.4/10

## Critical Issues
[List any critical issues]

## Warnings
[List warnings]

## Suggestions
[List suggestions]

## Summary
[Overall assessment]
```

See [CHECKLIST.md](CHECKLIST.md) for detailed review items.
See [EXAMPLES.md](EXAMPLES.md) for good/bad review examples.
See [TEAM_STANDARDS.md](TEAM_STANDARDS.md) for code standards.
