# PR Review Examples

Real-world examples of good and bad code reviews for learning.

## Example 1: Security Issue Review

### Bad Review
```
You have a security vulnerability. Fix it.

Overall: 5/10
```

### Good Review
```
## Code Review Summary

**Quality:** 7/10
**Security:** 3/10 ⚠️
**Architecture:** 7/10
**Testing:** 6/10
**Overall:** 5.75/10

## Critical Issues

### SQL Injection Vulnerability (Line 45)
The user_id is being concatenated directly into the SQL query:

```python
query = f"SELECT * FROM users WHERE id = {user_id}"
cursor.execute(query)
```

**Problem:** Malicious input like `1; DROP TABLE users;--` would delete data.

**Fix:** Use parameterized queries:
```python
query = "SELECT * FROM users WHERE id = ?"
cursor.execute(query, (user_id,))
```

**Why:** Parameterized queries separate data from SQL code, preventing injection.

## Warnings

### Hardcoded Database URL (Line 12)
The database connection string is in code:
```python
DB_URL = "postgresql://admin:password@localhost:5432/mydb"
```

Should be in `.env` file and loaded via environment variable for security.

## Summary

Great feature implementation, but **please fix the SQL injection before merging**. This is a critical security issue. Other than that, code quality and architecture are solid.
```

---

## Example 2: Code Quality Review (Good)

### Bad Code Review
```
Looks good! 9/10
```

### Good Code Review
```
## Code Review Summary

**Quality:** 9/10
**Security:** 9/10
**Architecture:** 8/10
**Testing:** 9/10
**Overall:** 8.75/10

## Warnings

### Repetitive Test Code (Lines 23-45)
Three test cases set up similar fixtures:

```python
def test_valid_email():
    user = User(email="test@example.com")

def test_invalid_email():
    user = User(email="invalid")

def test_none_email():
    user = User(email=None)
```

**Suggestion:** Use parameterized tests:
```python
@pytest.mark.parametrize("email,valid", [
    ("test@example.com", True),
    ("invalid", False),
    (None, False),
])
def test_email_validation(email, valid):
    result = is_valid_email(email)
    assert result == valid
```

This reduces duplication and is easier to maintain.

## Suggestions

1. **Error Message** (Line 62): "Error" is too generic. Use "Invalid email format: expected 'name@domain.com'" instead.

2. **Type Hints**: The `validate()` function lacks type hints. Would be helpful for clarity.

## Summary

Excellent PR! Code is clean, well-tested, and secure. Just suggested some minor improvements for maintainability. Ready to merge after addressing the repetitive test code comment.
```

---

## Example 3: Architecture Issue Review

### Bad Review
```
This doesn't follow patterns. Rewrite it.

2/10
```

### Good Review
```
## Code Review Summary

**Quality:** 6/10
**Security:** 8/10
**Architecture:** 3/10 ⚠️
**Testing:** 7/10
**Overall:** 6/10

## Critical Issues

### Violates Separation of Concerns (Lines 12-89)

The `UserService` class is handling three responsibilities:
1. User data access (database queries)
2. Business logic (validation, calculations)
3. API response formatting

**Current Structure:**
```python
class UserService:
    def create_user(self, data):
        # Validation
        if not data.get('email'):
            return {"error": "Email required"}

        # Database
        db.insert('users', data)

        # Response formatting
        return {"status": "success", "id": 123}
```

**Problem:** This violates our project's layered architecture pattern. Makes testing hard and changes ripple everywhere.

**Solution:** Use repository pattern like other services:
```python
# repo_layer.py
class UserRepository:
    def save(self, user):
        db.insert('users', user.dict())

# service_layer.py
class UserService:
    def __init__(self, repo):
        self.repo = repo

    def create_user(self, data):
        user = User(**data)  # Business logic
        user.validate()
        self.repo.save(user)
        return user

# api_layer.py
def create_user_endpoint(data):
    service = UserService(repo)
    user = service.create_user(data)
    return {"status": "success", "id": user.id}  # Response formatting
```

**Why:** Matches the pattern used in `CustomerService`, `OrderService`, etc. Makes testing easier because services can use mock repositories.

## Warnings

### Missing Integration Tests
You added a new payment processing flow but only have unit tests. Integration tests are needed to verify the full flow works end-to-end.

## Summary

The implementation is functional and secure, but the architecture needs refactoring to match project patterns. This will become a maintenance burden if merged as-is. Happy to discuss architecture if you have questions!
```

---

## Example 4: Positive Review (Minor Issues Only)

### Code Review
```
## Code Review Summary

**Quality:** 9.5/10
**Security:** 9.5/10
**Architecture:** 9/10
**Testing:** 9/10
**Overall:** 9.25/10

## Suggestions

### Optional Refactoring (Lines 34-41)
The date parsing logic could be extracted:

```python
# Current
date_str = item.get('created_at')
timestamp = datetime.fromisoformat(date_str).timestamp()

# Could be
timestamp = parse_iso_timestamp(item.get('created_at'))
```

Not urgent - just making the code slightly more reusable.

### Documentation (Line 5)
Consider adding a docstring explaining the performance optimization with the batch processing. Future maintainers will appreciate understanding why batches of 100 were chosen.

## Summary

Excellent work! The code is clean, well-tested, and secure. The performance optimization is well-implemented. Minor suggestions above but nothing blocking. Great PR!

**Ready to merge** ✓
```

---

## Review Pattern Analysis

### Anti-Pattern: Dismissive Review
```
"Looks fine" / "LGTM" / "9/10"
```
**Problem:** Doesn't help author improve. Doesn't document decisions.

### Good Pattern: Structured Review
```
- Clear severity levels
- Specific line numbers
- Explanation of "why" not just "no"
- Before/after examples
- Links to patterns
```

### Anti-Pattern: Personal Criticism
```
"This code is bad" / "You don't understand patterns"
```

### Good Pattern: Constructive Feedback
```
"This approach has X limitation. Y pattern would handle this better because..."
```

---

## Quick Reference for Common Issues

### Performance Issue Template
```
### Performance Concern (Line XX)
This query will N+1 for large datasets:
[code example]

Better approach:
[better code example]

Impact: Queries scale from O(n) to O(1) for typical usage.
```

### Security Issue Template
```
### Security Issue: [Type] (Line XX)
[Vulnerable code]

Problem: [Why it's vulnerable]
Fix: [Secure code]
Reference: [OWASP/Security policy link]
```

### Refactoring Suggestion Template
```
### Refactoring Opportunity (Lines XX-YY)
Similar pattern appears in XX.py line YY and ZZ.py line ZZ.

Could extract to shared utility:
[proposed helper]

Benefit: Reduces duplication and makes changes centralized.
```
