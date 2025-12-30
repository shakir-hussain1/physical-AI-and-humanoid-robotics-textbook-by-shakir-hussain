---
name: run-tests
description: Run test suite and fix any failures
invokes: test-runner
usage: "/run-tests" or "/run-tests tests/test_module.py"
returns: Test results, coverage report, fixed tests
---

# /run-tests

Run the test suite, analyze failures, and automatically fix common test issues.

## Usage

### Run All Tests
```
/run-tests
```

Runs entire test suite across all test files.

### Run Specific Test File
```
/run-tests tests/test_authentication.py
```

Runs tests only in specified file.

### Run Specific Test
```
/run-tests tests/test_authentication.py::test_login
```

Runs single test by function name.

### Run with Options
```
/run-tests --coverage --verbose
```

## What It Does

1. **Runs Tests:** Executes test suite (pytest, unittest, etc.)
2. **Analyzes Failures:** Understands why tests failed
3. **Reports Results:** Clear summary of pass/fail status
4. **Fixes Issues:** Attempts to fix common failure patterns
5. **Measures Coverage:** Reports code coverage percentage
6. **Suggests Improvements:** Recommends additional test cases

## Output Format

```
Test Results Summary
==================

Passed: 45/50 (90%)
Failed: 3
Skipped: 2

Coverage: 85%

Failed Tests:
1. test_authentication.py::test_invalid_password
   Error: AssertionError: 401 != 403

2. test_database.py::test_connection_timeout
   Error: Timeout after 5s

3. test_utils.py::test_hash_collision
   Error: 2 collisions found

Fixes Applied:
- [Fixed] test_authentication.py::test_invalid_password
  Changed: Expected 403 → 401 (correct per spec)

- [Fixed] test_database.py::test_connection_timeout
  Changed: Timeout from 5s → 10s (database initialization)

Still Failing:
- test_utils.py::test_hash_collision
  Needs review: Hash function has collision at indices X, Y
```

## Test Framework Support

Works with:
- **pytest** - Modern Python testing
- **unittest** - Built-in Python testing
- **Jest** - JavaScript testing
- **Mocha** - Node.js testing
- **RSpec** - Ruby testing
- **Jasmine** - JavaScript/Angular testing

## Common Issues Fixed

### Type Mismatches
```
Before: assert login_response.status == "401"
After:  assert login_response.status == 401
```

### Assertion Errors
```
Before: assert result > 100
After:  assert result >= 100  (based on spec)
```

### Timeout Issues
```
Before: @timeout(5s)
After:  @timeout(10s)  (for slow operations)
```

### Missing Fixtures
```
Before: user = User(...)  # User not in database
After:  user = User.create(...); session.commit()
```

### Flaky Tests
```
Before: time.sleep(1)  # Unreliable timing
After:  wait_for(condition, timeout=5)
```

## Coverage Goals

- **Minimum:** 80% for all code
- **Target:** 85% for new code
- **Critical paths:** 100% coverage
- **Ignored:** Generated code, utilities

## Examples

### Basic Test Run
```
/run-tests
```

### Test Specific Module
```
/run-tests tests/models/test_user.py
```

### Test with Verbose Output
```
/run-tests --verbose
```

### Run and Fix
```
/run-tests --fix
```

Automatically apply fixes to common issues.

## Workflow

1. Run tests: `/run-tests`
2. Review failures (if any)
3. Auto-fix common issues: `/run-tests --fix`
4. Commit fixed tests
5. Merge when all pass

## CI/CD Integration

Can be triggered automatically:
- On every commit
- Before pull request merge
- During deployment
- Scheduled nightly

## See Also

- [Test Runner Agent Documentation](./.claude/agents/test-runner.md)
- [Testing Best Practices](./guides/testing.md)
- [Test Coverage Report](./coverage/)
- [Failed Test Analysis](./debug/test-failures.md)
