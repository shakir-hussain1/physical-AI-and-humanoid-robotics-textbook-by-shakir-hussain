---
name: gen-docs
description: Generate documentation for code or features
invokes: doc-generator
usage: "/gen-docs API documentation for query endpoint"
returns: Formatted documentation, examples, guides
---

# /gen-docs

Generate professional documentation for code, APIs, and features.

## Usage

### Generate API Documentation
```
/gen-docs API documentation for /api/users endpoint
```

### Generate README
```
/gen-docs README for authentication module
```

### Generate User Guide
```
/gen-docs User guide for feature X
```

### Generate Code Examples
```
/gen-docs Code examples for RAG query system
```

### Generate Architecture Docs
```
/gen-docs Architecture documentation for data pipeline
```

## Documentation Types

### API Documentation
- Request/response examples
- Parameter descriptions
- Error codes and handling
- Authentication requirements
- Rate limiting info

### User Guides
- Step-by-step tutorials
- Feature explanations
- Common use cases
- Troubleshooting tips
- FAQ section

### Code Documentation
- Class/function docstrings
- Code examples
- Architecture diagrams
- Design patterns used
- Known limitations

### README Files
- Project overview
- Quick start guide
- Installation instructions
- Configuration options
- Contributing guidelines

### Integration Guides
- External service setup
- API key configuration
- Webhook setup
- Event handling
- Error handling patterns

## Output Formats

### Markdown
```
# API Endpoint: Create User

## Overview
Creates a new user account...

## Request
POST /api/users

### Parameters
| Name | Type | Required | Description |
...

## Response
```

### HTML
```
<h1>API Endpoint: Create User</h1>
<p>Creates a new user account...</p>
...
```

### PDF
Professional formatted documentation with branding.

### Confluence/Wiki
Wiki-formatted documentation.

## Generated Sections

### API Docs Include
- Endpoint summary
- HTTP method and path
- Parameters (required/optional)
- Request body example
- Success response example
- Error response examples
- Authentication method
- Rate limits
- Code examples (Python, JavaScript, cURL)

### Code Docs Include
- Class/function description
- Parameters with types
- Return value description
- Exceptions thrown
- Usage examples
- See also (related functions)
- Implementation notes

### Guide Docs Include
- Introduction
- Prerequisites
- Step-by-step instructions
- Expected results
- Troubleshooting section
- Related topics
- Links to more information

## Examples

### API Documentation
```
/gen-docs API documentation for RAG query endpoint
```

Generates complete documentation:
- Query endpoint overview
- Request format with examples
- Response format with examples
- Error handling
- Rate limits
- Code samples

### User Guide
```
/gen-docs Tutorial for using RAG search feature
```

Generates step-by-step guide:
- What is RAG search
- Getting started
- Common queries
- Best practices
- Advanced usage
- Troubleshooting

### Architecture Guide
```
/gen-docs Architecture documentation for authentication system
```

Generates architecture docs:
- System overview
- Component description
- Data flow diagrams
- Security considerations
- Integration points
- Future improvements

## Features

- **Auto-Detection:** Analyzes code structure
- **Examples:** Generates real working examples
- **Cross-References:** Links related documentation
- **Professional Formatting:** Clean, readable output
- **Multi-Language Support:** Code samples in multiple languages
- **ASCII Diagrams:** Architecture diagrams without external tools

## Quality Standards

Generated documentation includes:
- Clear, non-technical explanations
- Real working examples
- Proper formatting and structure
- Consistent style with team standards
- Complete parameter descriptions
- Practical use cases
- Troubleshooting section

## Workflow

1. **Request docs:** `/gen-docs [what you need]`
2. **Review generated docs:** Check content and examples
3. **Make adjustments:** Customize as needed
4. **Publish:** Commit to docs folder
5. **Link:** Add to README or index

## Integration

Works with:
- **Code:** Python, JavaScript, Go, Java, etc.
- **APIs:** REST, GraphQL, RPC
- **Databases:** SQL, NoSQL
- **Services:** External APIs, webhooks
- **Features:** User-facing features, admin tools

## See Also

- [Documentation Generator Agent](./.claude/agents/doc-generator.md)
- [Documentation Standards](./guides/documentation-standards.md)
- [API Documentation Guide](./guides/api-documentation.md)
- [Writing Technical Documentation](./guides/technical-writing.md)
