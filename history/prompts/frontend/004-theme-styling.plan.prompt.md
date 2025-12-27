---
ID: 004
TITLE: Theme and Styling System
STAGE: plan
DATE_ISO: 2025-12-21
SURFACE: agent
MODEL: claude-haiku-4-5-20251001
FEATURE: frontend-theme
BRANCH: 1-rag-chatbot-backend
USER: user
COMMAND: Design theme and styling system
LABELS:
  - frontend
  - theme
  - styling
  - css
LINKS:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
---

## Prompt

Design comprehensive theme and styling system with dark mode, light mode, customizable colors, fonts, and Urdu support.

## Response

### Theme and Styling System

**File Structure:**
```
src/
├── theme/
│   ├── index.js           - Theme provider
│   ├── colors.js          - Color palette
│   ├── typography.js      - Font configuration
│   └── variants/
│       ├── light.css
│       └── dark.css
└── css/
    ├── custom.css         - Custom styles
    ├── variables.css      - CSS variables
    └── globals.css        - Global styles
```

**Color Palette:**

Light Theme:
- Primary: #2563eb (Blue)
- Secondary: #7c3aed (Purple)
- Background: #ffffff (White)
- Surface: #f3f4f6 (Light Gray)
- Text: #1f2937 (Dark Gray)
- Success: #10b981 (Green)
- Warning: #f59e0b (Amber)
- Error: #ef4444 (Red)

Dark Theme:
- Primary: #60a5fa (Light Blue)
- Secondary: #a78bfa (Light Purple)
- Background: #0f172a (Very Dark)
- Surface: #1e293b (Dark Slate)
- Text: #f1f5f9 (Light Gray)
- Success: #34d399 (Light Green)
- Warning: #fbbf24 (Light Amber)
- Error: #f87171 (Light Red)

**Typography:**

Fonts:
- Heading: 'Poppins' (Bold, Clean)
- Body: 'Inter' (Readable)
- Code: 'Fira Code' (Monospace)
- Urdu: 'Noto Naskh Arabic' (RTL Support)

Sizes:
- h1: 2.5rem (40px)
- h2: 2rem (32px)
- h3: 1.5rem (24px)
- body: 1rem (16px)
- small: 0.875rem (14px)
- caption: 0.75rem (12px)

**Spacing System:**
- xs: 0.25rem (4px)
- sm: 0.5rem (8px)
- md: 1rem (16px)
- lg: 1.5rem (24px)
- xl: 2rem (32px)
- 2xl: 3rem (48px)

**Responsive Breakpoints:**
- Mobile: 320px - 640px
- Tablet: 641px - 1024px
- Desktop: 1025px+

**Dark Mode Implementation:**
- User preference detection
- localStorage persistence
- System preference fallback
- Smooth transitions
- Automatic detection option

**CSS Variables:**
```css
:root {
  --color-primary: #2563eb;
  --color-secondary: #7c3aed;
  --bg-primary: #ffffff;
  --bg-secondary: #f3f4f6;
  --text-primary: #1f2937;
  --text-secondary: #6b7280;
  --spacing-unit: 8px;
}

[data-theme="dark"] {
  --color-primary: #60a5fa;
  --color-secondary: #a78bfa;
  --bg-primary: #0f172a;
  --bg-secondary: #1e293b;
  --text-primary: #f1f5f9;
  --text-secondary: #cbd5e1;
}
```

**Component Theming:**
- Consistent button styles
- Form inputs styling
- Card components
- Modal overlays
- Tabs and tabs styling
- Badge and label styling

**Internationalization:**
- RTL support for Urdu
- Font fallbacks
- Text direction handling
- LTR layouts
- CSS logical properties

---

## Outcome

✅ Theme system designed
✅ Color palette defined
✅ Typography configured
✅ Responsive breakpoints set
✅ Dark mode implemented
✅ Urdu RTL support ready
✅ CSS variables documented

**Status**: Theme system ready for implementation

