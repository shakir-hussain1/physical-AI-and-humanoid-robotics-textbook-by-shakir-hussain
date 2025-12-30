---
name: debug
description: Start debugging workflow for errors and issues
invokes: debugger
usage: "/debug [error message or stack trace]"
returns: Root cause analysis, fix, prevention recommendations
---

# /debug

Debug errors and issues with systematic root cause analysis and fixes.

## Usage

### Paste Error Message
```
/debug TypeError: cannot read property 'name' of undefined
```

### Paste Stack Trace
```
/debug
Traceback (most recent call last):
  File "main.py", line 42, in process_user
    return user.name.upper()
AttributeError: 'NoneType' object has no attribute 'name'
```

### Describe Issue
```
/debug Database connection randomly drops after 10 minutes
```

### Provide Context
```
/debug
Error: TypeError: call a function
Stack trace: ...
Context: Happens during async request processing
```

## What It Does

1. **Error Analysis:** Understands error type and message
2. **Scope Narrowing:** Identifies where error occurs
3. **Root Cause:** Finds underlying problem
4. **Hypothesis Testing:** Validates theories
5. **Solution:** Provides fix with explanation
6. **Prevention:** Suggests how to prevent in future

## Error Categories Handled

### Configuration Errors
- Missing environment variables
- Invalid configuration values
- File path issues
- Permission errors

### API/Network Errors
- Connection refused
- Timeout errors
- DNS resolution failures
- TLS/SSL certificate issues

### Data Errors
- Type mismatches
- Data validation failures
- Encoding issues
- Null pointer exceptions

### Logic Errors
- Algorithm bugs
- State management issues
- Race conditions
- Off-by-one errors

### Async/Concurrency Errors
- Deadlocks
- Race conditions
- Event loop issues
- Promise rejections

## Output Format

```
Error Analysis
==============

Error Type: TypeError
Message: cannot read property 'name' of undefined

Root Cause:
User object is null because query returned no results
for email address. Code assumes user always exists.

Location:
File: src/services/user_service.py
Line: 42
Function: process_user()

Fix:
Add null check before accessing user.name:

Before:
  user = db.query(User).filter_by(email=email).first()
  return user.name.upper()

After:
  user = db.query(User).filter_by(email=email).first()
  if not user:
    raise UserNotFoundError(f"User with email {email} not found")
  return user.name.upper()

Prevention:
- Always check for null before accessing properties
- Use types/contracts to enforce non-null objects
- Write tests for edge cases (missing data, etc.)
- Use linters to catch potential null dereferences

Similar Errors:
- Line 58: user.email accessed without null check
- Line 103: user.profile accessed without null check
```

## Examples

### Network Timeout
```
/debug Connection timed out after 30 seconds to database.example.com:5432
```

Returns analysis of database connectivity issue and fixes.

### Type Error
```
/debug TypeError: string indices must be integers
```

Analyzes indexing error in string/array handling.

### Async/Promise Error
```
/debug UnhandledPromiseRejectionWarning: Error: database connection failed
```

Helps debug unhandled promise rejection.

### Configuration Error
```
/debug KeyError: 'OPENAI_API_KEY' environment variable not found
```

Identifies missing configuration and provides setup.

## Debugging Process

1. **Capture error:** Provide error message or stack trace
2. **Run debug:** `/debug [error details]`
3. **Review analysis:** Understand root cause
4. **Apply fix:** Implement suggested solution
5. **Test fix:** Verify error is resolved
6. **Implement prevention:** Follow recommendations

## Common Fixes

### Null Pointer
```
Error: Cannot read property 'name' of null
Fix: Add null check before property access
```

### Import/Module Not Found
```
Error: ModuleNotFoundError: No module named 'requests'
Fix: Install package (pip install requests)
```

### Connection Refused
```
Error: Connection refused to localhost:5432
Fix: Start PostgreSQL service or check hostname
```

### Type Mismatch
```
Error: TypeError: string expected, got int
Fix: Convert type or check function signature
```

### Configuration Missing
```
Error: KeyError: 'DATABASE_URL'
Fix: Set environment variable DATABASE_URL
```

## See Also

- [Debugger Agent Documentation](./.claude/agents/debugger.md)
- [Error Codes Reference](./docs/error-codes.md)
- [Common Issues & Solutions](./guides/troubleshooting.md)
- [Stack Trace Analysis Guide](./guides/debugging.md)
