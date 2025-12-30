# Team Code Standards

Coding standards and patterns followed by the Physical AI and Humanoid Robotics project team.

## Language & Style

### Python Standards

#### Code Style
- Follow PEP 8 with line length limit of 100 characters
- Use 4 spaces for indentation (not tabs)
- Use type hints for function signatures
- Use f-strings for string formatting (Python 3.6+)

#### Naming Conventions
```python
# Classes: PascalCase
class UserAuthentication:
    pass

# Functions/variables: snake_case
def get_user_by_id(user_id):
    pass

# Constants: UPPER_SNAKE_CASE
MAX_RETRIES = 3
DEFAULT_TIMEOUT = 30

# Private/internal: prefix with _
_internal_cache = {}

def _process_internal_data():
    pass
```

#### Imports
```python
# Order: standard library, third-party, local
import os
import sys
from typing import List, Dict, Optional

import numpy as np
import requests

from . import models
from .config import settings
```

#### Function Docstrings
```python
def calculate_distance(point_a: tuple, point_b: tuple) -> float:
    """
    Calculate Euclidean distance between two points.

    Args:
        point_a: Tuple of (x, y) coordinates
        point_b: Tuple of (x, y) coordinates

    Returns:
        Float representing the distance between points

    Raises:
        ValueError: If points don't have exactly 2 coordinates
    """
    if len(point_a) != 2 or len(point_b) != 2:
        raise ValueError("Points must be 2D coordinates")

    return ((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)**0.5
```

### TypeScript/JavaScript Standards

#### Code Style
- Use ESLint configuration from project
- Use 2 spaces for indentation
- Use const by default, let when needed, avoid var
- Use async/await over Promise chains
- Use arrow functions for callbacks

#### Naming Conventions
```typescript
// Classes and Interfaces: PascalCase
class UserService {
}

interface IUserRepository {
}

// Variables and functions: camelCase
let currentUser: IUser;

function getUserById(id: string): Promise<IUser> {
}

// Constants: UPPER_SNAKE_CASE
const MAX_RETRIES = 3;
const API_TIMEOUT = 5000;
```

#### Type Annotations
```typescript
// Always annotate function parameters and returns
function addNumbers(a: number, b: number): number {
    return a + b;
}

// Use proper types, not 'any'
interface ApiResponse {
    status: number;
    data: unknown; // Use unknown instead of any
    error?: string;
}
```

---

## Architecture Patterns

### Repository Pattern (Data Access)
```python
# ✅ Good: Separated concerns
class UserRepository:
    def get_by_id(self, user_id: int) -> Optional[User]:
        return db.query(User).filter(User.id == user_id).first()

    def save(self, user: User) -> None:
        db.session.add(user)
        db.session.commit()

class UserService:
    def __init__(self, repo: UserRepository):
        self.repo = repo

    def create_user(self, email: str) -> User:
        user = User(email=email)
        self.repo.save(user)
        return user

# ❌ Avoid: Business logic mixed with data access
def create_user(email: str):
    user = User(email=email)
    db.session.add(user)
    db.session.commit()
    return user
```

### Dependency Injection
```python
# ✅ Good: Dependencies injected
class OrderService:
    def __init__(self, user_repo: UserRepository, email_service: EmailService):
        self.user_repo = user_repo
        self.email_service = email_service

    def create_order(self, user_id: int, items: List[Item]):
        user = self.user_repo.get_by_id(user_id)
        # ...
        self.email_service.send_confirmation(user.email)

# ❌ Avoid: Global references and tight coupling
def create_order(user_id: int):
    user = db.query(User).get(user_id)  # Tight coupling
    send_email(user.email)  # Global reference
```

### Configuration Management
```python
# ✅ Good: Environment-based configuration
from pydantic import BaseSettings

class Settings(BaseSettings):
    database_url: str
    api_key: str
    debug: bool = False

    class Config:
        env_file = ".env"

settings = Settings()

# ✅ Good: Layered configuration
class DevelopmentConfig:
    DEBUG = True
    DATABASE_URL = "sqlite:///dev.db"

class ProductionConfig:
    DEBUG = False
    DATABASE_URL = os.environ['DATABASE_URL']

config = ProductionConfig() if ENV == 'production' else DevelopmentConfig()

# ❌ Avoid: Hardcoded values
API_KEY = "sk-12345..."  # NEVER
DATABASE_URL = "postgresql://user:pass@localhost/db"  # NEVER
```

### Error Handling
```python
# ✅ Good: Specific exceptions
class UserNotFoundError(Exception):
    pass

class InvalidCredentialsError(Exception):
    pass

def authenticate(username: str, password: str) -> User:
    user = db.query(User).filter_by(username=username).first()
    if not user:
        raise UserNotFoundError(f"User {username} not found")

    if not verify_password(password, user.password_hash):
        raise InvalidCredentialsError("Invalid password")

    return user

# ❌ Avoid: Generic exceptions
try:
    do_something()
except Exception:  # Too broad
    pass

# ❌ Avoid: Silently failing
try:
    do_something()
except:  # Bare except is bad
    pass
```

---

## Testing Standards

### Test Organization
```python
# ✅ Good: Clear test structure
def test_user_creation_with_valid_email():
    """Should create user when email is valid."""
    user = User.create(email="test@example.com")
    assert user.id is not None
    assert user.email == "test@example.com"

def test_user_creation_with_invalid_email():
    """Should raise error when email is invalid."""
    with pytest.raises(InvalidEmailError):
        User.create(email="not-an-email")

# ❌ Avoid: Unclear test names
def test_user():
    pass

def test_email():
    pass
```

### Mocking and Fixtures
```python
# ✅ Good: Reusable fixtures
@pytest.fixture
def user_repo():
    return UserRepository(test_db)

@pytest.fixture
def user_service(user_repo):
    return UserService(user_repo)

def test_service_creates_user(user_service):
    user = user_service.create_user("test@example.com")
    assert user.email == "test@example.com"

# ✅ Good: Clear mocking
def test_order_sends_confirmation(mocker):
    email_service = mocker.Mock(spec=EmailService)
    order_service = OrderService(repo, email_service)

    order_service.create_order(user_id=1, items=[])

    email_service.send_confirmation.assert_called_once()

# ❌ Avoid: Hard to understand mocks
mock_everything()
result = do_something()
# Unclear what was mocked or why
```

### Test Coverage
- **Minimum:** 80% code coverage
- **Target:** 85%+ for new code
- **Critical paths:** 100% coverage
- Focus on behavior, not implementation details

---

## API Design Standards

### REST Conventions
```python
# ✅ Good: RESTful endpoints
GET    /api/users              # List users
GET    /api/users/123          # Get specific user
POST   /api/users              # Create user
PUT    /api/users/123          # Update user
DELETE /api/users/123          # Delete user

# ❌ Avoid: Action-based endpoints
GET    /api/get_users
GET    /api/create_user
GET    /api/delete_user/123
```

### Request/Response Format
```python
# ✅ Good: Consistent response structure
{
    "status": 200,
    "data": {
        "id": 123,
        "name": "John",
        "email": "john@example.com"
    },
    "message": null
}

# Error response
{
    "status": 400,
    "data": null,
    "message": "Invalid email format"
}

# ❌ Avoid: Inconsistent formats
{
    "id": 123,
    "name": "John"
}

{
    "error": "Invalid email"
}
```

### Error Codes
| Code | Meaning | Use Case |
|------|---------|----------|
| 200 | OK | Successful request |
| 201 | Created | Resource successfully created |
| 400 | Bad Request | Invalid input/validation error |
| 401 | Unauthorized | Missing authentication |
| 403 | Forbidden | Authenticated but no permission |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource already exists |
| 500 | Internal Error | Server error |

---

## Git & Version Control

### Commit Messages
```
# ✅ Good: Clear, descriptive
feat: add user authentication with JWT tokens
fix: resolve N+1 query in user listing
refactor: extract email validation to utility
docs: update API documentation for auth endpoints

# ❌ Avoid: Vague or generic
update code
fix bug
changes
wip
```

### Branch Naming
- `feat/description` - New feature
- `fix/description` - Bug fix
- `refactor/description` - Refactoring
- `docs/description` - Documentation only

### PR Requirements
- ✅ All tests passing
- ✅ 80%+ code coverage for new code
- ✅ At least one code review approval
- ✅ Commit messages follow convention
- ✅ No merge conflicts
- ❌ No hardcoded secrets
- ❌ No commented-out code

---

## Performance Guidelines

### Python Performance
- Cache expensive operations with `functools.lru_cache`
- Use generators for large datasets
- Avoid N+1 queries (use eager loading/joins)
- Profile before optimizing

### Database Performance
- Indexes on frequently queried columns
- Connection pooling for multiple databases
- Query plans reviewed for complex queries
- Pagination for large result sets (limit 100)

### API Performance
- Response time target: <200ms (p95)
- Cache headers for static content
- Compression for large responses
- Async operations for I/O

---

## Security Standards

### OWASP Top 10 Compliance

1. **No SQL Injection:** Always use parameterized queries
2. **No XSS:** Escape/sanitize all user output
3. **Secure Authentication:** Use established libraries (passlib, python-jose)
4. **No Hardcoded Secrets:** Use environment variables
5. **Input Validation:** Validate all user inputs
6. **Output Encoding:** Encode data for target format
7. **Dependency Management:** Keep libraries updated
8. **Access Control:** Verify permissions on every operation
9. **Secure Communication:** HTTPS in production, validate SSL
10. **Error Handling:** Don't expose sensitive information

### Secrets Management
```python
# ✅ Good
api_key = os.environ.get('OPENAI_API_KEY')
if not api_key:
    raise ValueError("OPENAI_API_KEY not set")

# ❌ Never
api_key = "sk-12345..."  # In code
api_key = "sk-12345..."  # In comments
# api_key = "sk-12345..."  # Commented out

# ❌ Avoid
with open('secrets.txt') as f:
    api_key = f.read()  # Unencrypted file
```

### Logging Security
```python
# ✅ Good: Log errors, not secrets
logger.error(f"Auth failed for user {user_id}")

# ❌ Avoid: Logging sensitive data
logger.info(f"User password: {password}")
logger.debug(f"API Key: {api_key}")
logger.info(f"Credit card: {card_number}")
```

---

## Code Review Priorities

When reviewing code, check in this order:

1. **Security** - Are there vulnerabilities?
2. **Correctness** - Does it work as intended?
3. **Testing** - Is it adequately tested?
4. **Performance** - Are there obvious bottlenecks?
5. **Architecture** - Does it follow patterns?
6. **Style** - Is it clean and readable?

Remember: Perfect is the enemy of good. Not all issues need fixing before merge.
