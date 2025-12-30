---
name: review-pr
description: Start professional PR review with team standards
invokes: code-reviewer
usage: "/review-pr PR#42" or provide code directly
returns: Quality, security, architecture assessment
---

# /review-pr

Conduct a professional code review using team standards and best practices.

## Usage

### By PR Number (GitHub)
```
/review-pr PR#123
```

### By GitHub URL
```
/review-pr https://github.com/owner/repo/pull/123
```

### Direct Code Input
```
/review-pr
[paste code here]
```

## What Gets Reviewed

The reviewer checks across four dimensions:

### Quality (40%)
- Code clarity and readability
- Naming conventions (functions, variables, classes)
- DRY principle (no duplication)
- Proper abstraction levels
- Error handling
- Code organization and structure

### Security (30%)
- No hardcoded secrets/credentials
- Input validation present
- Authentication/authorization checks
- SQL injection prevention
- XSS prevention
- Data privacy considerations

### Architecture (20%)
- Follows project patterns
- Minimal dependencies
- Proper separation of concerns
- Backward compatibility
- Performance implications
- Scalability considerations

### Testing (10%)
- Unit test coverage (>80%)
- Edge cases tested
- Integration tests present
- No skipped tests
- Proper test isolation

## Output Format

```
## Code Review Summary

Quality: 8.5/10
Security: 9/10
Architecture: 8/10
Testing: 8.5/10
Overall: 8.4/10

## Critical Issues
[Issues that must be fixed before merge]

## Warnings
[Issues that should be addressed]

## Suggestions
[Optional improvements]

## Summary
[Overall assessment and recommendation]
```

## Severity Levels

🔴 **Critical** (Must Fix)
- Security vulnerabilities
- Data loss/corruption risk
- Breaking changes without migration
- Missing error handling
- No test coverage for changes

🟡 **Warning** (Should Fix)
- Performance regression
- Code duplication
- Missing documentation
- Overly complex logic
- Poor naming

🟢 **Suggestion** (Nice to Have)
- Refactoring opportunities
- Better error messages
- Code style improvements
- Additional test cases

## Examples

### Review by PR Number
```
/review-pr PR#42
```

Reviewer fetches the PR, analyzes all changes, provides comprehensive feedback.

### Review Code Snippet
```
/review-pr

def calculate_distance(x1, y1, x2, y2):
    return ((x2-x1)**2 + (y2-y1)**2)**0.5
```

Reviewer analyzes the provided code directly.

## Integration

Works with:
- GitHub pull requests
- GitLab merge requests
- Gitea pull requests
- Direct code snippets

## See Also

- [PR Review Skill Documentation](./.claude/skills/pr-review/)
- [Team Standards](./claude/skills/pr-review/TEAM_STANDARDS.md)
- [Review Examples](./claude/skills/pr-review/EXAMPLES.md)
- [Detailed Checklist](./claude/skills/pr-review/CHECKLIST.md)
