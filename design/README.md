# AI Design Context Management

This folder contains design documents and strategies for managing AI context across different tools and modes.

## Folder Purpose

Store design files and documentation to preserve context when switching between:
- Claude Opus (design phase)
- Claude Sonnet (implementation phase)
- GitHub Copilot modes (chat → agent)
- Different AI tools (Copilot → Cursor → Claude.ai)

## Context Preservation Strategies

### 1. Two-Phase Workflow
```
Phase 1: Design (Claude Opus/Ask Mode)
├── Create comprehensive system architecture
├── Define component interfaces
├── Specify data models and API contracts
└── Save output to design/*.md files

Phase 2: Implementation (Sonnet/Agent Mode)
├── Reference design documents
├── Implement components one by one
├── Maintain design consistency
└── Update design docs as needed
```

### 2. File Naming Convention
```
design/
├── {project-name}-architecture.md      # High-level system design
├── {project-name}-api-spec.md          # API endpoints and interfaces
├── {project-name}-data-models.md       # Database schemas and types
├── {project-name}-components.md        # Component specifications
└── {project-name}-decisions.md         # Design decisions and rationale
```

### 3. Context Window Management

#### Tool Comparison
| Tool | Context Window | Best For |
|------|----------------|----------|
| GitHub Copilot | ~8-16K tokens | Quick implementations, focused tasks |
| Cursor | ~100-200K tokens | Large refactoring, multi-file operations |
| Claude.ai Direct | ~200K tokens | Initial system design, complex architecture |

#### Overflow Prevention
- Break large designs into focused sections
- Reference specific parts: "Implement auth module from lines 50-100"
- Use structured prompts with key design points
- Create interface files first, then implementations

### 4. Mode Switching Best Practices

#### Before Switching (Ask Mode → Agent Mode)
1. **Save Design Output**: Copy design to markdown files
2. **Create Summary**: Ask AI to generate implementation checklist
3. **Structure References**: Break design into implementable chunks

#### Transitional Prompts
```markdown
"Continue implementing the {component} system. Key requirements:
- Use {pattern} design pattern
- Include {specific features}
- Follow architecture in design/{file}.md"
```

### 5. Context Loss Scenarios

#### Complete Loss
- Copilot Chat → Agent Mode switch
- Closing/reopening VS Code
- Switching between different AI tools

#### Mitigation Strategies
- Document decisions in code comments
- Use git commits as context markers
- Create skeleton files with design as comments
- Maintain implementation checklists

### 6. File Templates

#### Design Document Template
```markdown
# {Project} System Design

## Overview
Brief description of the system

## Architecture
High-level architecture diagram and explanation

## Components
### Component 1
- Responsibility:
- Interfaces:
- Dependencies:

## Data Models
Database schemas and type definitions

## API Specifications
Endpoint definitions and contracts

## Implementation Notes
Key decisions and constraints
```

#### Implementation Checklist Template
```markdown
# {Project} Implementation Checklist

## Phase 1: Core Components
- [ ] Component A (estimated: X hours)
- [ ] Component B (estimated: Y hours)

## Phase 2: Integration
- [ ] API layer
- [ ] Database layer

## Phase 3: Testing
- [ ] Unit tests
- [ ] Integration tests

## Design References
- Architecture: design/{project}-architecture.md
- APIs: design/{project}-api-spec.md
```

### 7. Tool-Specific Tips

#### GitHub Copilot
- Use @workspace to reference design files
- Break requests into small, focused tasks
- Reference specific files with #file:{filename}

#### Cursor
- Keep design docs open for context
- Use conversation history for complex refactoring
- Leverage large context for system-wide changes

#### Claude.ai Direct
- Best for initial design and architecture
- Export designs to files manually
- Use for complex decision-making

## Workflow Examples

### Example 1: New Feature Development
```
1. Design (Claude.ai): "Design a user authentication system"
2. Save: Copy output to design/auth-system.md
3. Implement (Copilot): "Implement UserService based on design/auth-system.md"
4. Test: Reference design for test scenarios
```

### Example 2: Large Refactoring
```
1. Analysis (Cursor): Review current codebase with large context
2. Design (Opus): Plan refactoring strategy
3. Save: Document strategy in design/refactoring-plan.md
4. Execute (Agent Mode): Implement changes step by step
```

## Best Practices Summary

1. **Always document**: Save design outputs to files
2. **Reference explicitly**: Use specific file/line references
3. **Break down tasks**: Avoid overwhelming context windows
4. **Use the right tool**: Match tool capabilities to task requirements
5. **Maintain consistency**: Reference original design throughout implementation
6. **Version control**: Use git to track design evolution

---

*Last updated: June 4, 2025*
