# PR Review Detailed Checklist

Comprehensive verification items for pull request reviews using team standards.

## Code Quality (40 points)

### Readability & Style (10 points)
- [ ] Code is easy to understand at first reading
- [ ] Variable and function names are clear and descriptive
- [ ] Comments explain "why" not "what"
- [ ] No commented-out code blocks left
- [ ] Consistent indentation and formatting
- [ ] Lines are reasonably short (<100 chars where possible)
- [ ] No magic numbers (constants are named)
- [ ] Complex logic is broken into smaller functions

### DRY Principle (8 points)
- [ ] No copy-pasted code blocks
- [ ] Similar logic is extracted to shared functions
- [ ] Utility functions are reused across codebase
- [ ] Constants are defined once and reused

### Error Handling (8 points)
- [ ] All error cases are handled appropriately
- [ ] Exceptions are caught at right level
- [ ] Error messages are helpful and specific
- [ ] No silent failures (no bare except blocks)
- [ ] Proper logging for debugging

### Code Organization (6 points)
- [ ] Related code is grouped together
- [ ] Functions have single responsibility
- [ ] File structure makes sense
- [ ] Class hierarchies are logical
- [ ] Imports are organized
- [ ] No circular dependencies

### Testing Code Quality (8 points)
- [ ] Test names clearly describe what they test
- [ ] Tests are focused (test one thing)
- [ ] Setup/teardown is properly handled
- [ ] No hardcoded test data
- [ ] Tests are isolated from each other

## Security Review (30 points)

### Secrets & Credentials (8 points)
- [ ] No API keys hardcoded
- [ ] No passwords in code
- [ ] No private tokens visible
- [ ] .env files are in .gitignore
- [ ] No credentials in comments

### Input Validation (8 points)
- [ ] All user input is validated
- [ ] Type checking is performed
- [ ] Length limits are enforced
- [ ] Whitespace is handled properly
- [ ] Special characters are escaped

### Authentication & Authorization (6 points)
- [ ] Auth checks are present where needed
- [ ] Permission checks are correct
- [ ] No unauthorized data access
- [ ] Token/session handling is secure

### Injection Prevention (6 points)
- [ ] SQL queries use parameterized statements
- [ ] No string concatenation in SQL
- [ ] Command injection is prevented
- [ ] Path traversal is prevented

### Data Privacy (2 points)
- [ ] Sensitive data is not logged
- [ ] PII is handled according to policy

## Architecture (20 points)

### Project Patterns (6 points)
- [ ] Follows established project patterns
- [ ] Consistent with existing code style
- [ ] Uses project utilities/helpers
- [ ] Respects architectural boundaries

### Dependencies (4 points)
- [ ] New dependencies are justified
- [ ] No circular dependencies introduced
- [ ] Minimal external package additions
- [ ] Version pinning is reasonable

### Separation of Concerns (4 points)
- [ ] Business logic is separate from UI
- [ ] Data access is separate from logic
- [ ] Configuration is external
- [ ] No mixing of concerns

### Backward Compatibility (3 points)
- [ ] Breaking changes are documented
- [ ] Deprecated APIs still work
- [ ] Database migrations are provided
- [ ] API versioning is maintained

### Performance (2 points)
- [ ] No obvious performance problems
- [ ] N+1 query problems are avoided
- [ ] Caching is used where appropriate
- [ ] Large data structures are handled efficiently

### Scalability (1 point)
- [ ] Solution scales to expected load

## Testing (10 points)

### Coverage (4 points)
- [ ] New code has test coverage (>80%)
- [ ] Edge cases are tested
- [ ] Happy path is tested
- [ ] Error paths are tested

### Test Quality (3 points)
- [ ] Tests actually verify behavior
- [ ] No assertions are skipped
- [ ] Tests are repeatable/deterministic
- [ ] Fixtures are properly set up

### Integration Testing (2 points)
- [ ] Integration tests exist for new features
- [ ] Tests verify data flow
- [ ] External system interactions are tested

### Maintenance (1 point)
- [ ] Tests will be easy to maintain
- [ ] Test code is clean

---

## Severity Ratings

### 🔴 Critical (Must Fix Before Merge)
- Security vulnerabilities
- Data loss/corruption risk
- Breaking changes without migration
- Missing error handling
- No test coverage for changes

### 🟡 Warning (Should Fix)
- Performance regressions
- Code duplication
- Missing documentation
- Overly complex logic
- Poor naming conventions

### 🟢 Suggestion (Nice to Have)
- Refactoring opportunities
- Better error messages
- Code style improvements
- Additional test cases

---

## Review Workflow

1. **Initial Scan** (2 min)
   - Read PR description
   - Check file list
   - Note test coverage changes

2. **Code Review** (10-15 min)
   - Read code from top to bottom
   - Use checklist items as guide
   - Note issues as you find them

3. **Security Pass** (5 min)
   - Specifically look for secrets
   - Check input validation
   - Verify auth checks

4. **Architecture Pass** (5 min)
   - Check patterns
   - Verify separation of concerns
   - Look for red flags

5. **Testing Pass** (3 min)
   - Review test coverage
   - Check test quality
   - Verify edge cases

6. **Summarize** (3 min)
   - Compile findings
   - Assign severity levels
   - Write constructive feedback

---

## Tips for Thorough Reviews

- **Read the tests first** - they show intended behavior
- **Look for patterns** - find code that should be DRY
- **Think about edge cases** - what could break this?
- **Check the git history** - understand why code exists
- **Ask questions** - reviewers educate each other
- **Be constructive** - focus on the code, not the person
- **Review promptly** - feedback is most useful when fresh
