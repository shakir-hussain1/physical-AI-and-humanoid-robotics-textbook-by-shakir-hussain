# Content Personalization Feature - User Guide

## Overview

The Content Personalization feature allows users to customize what content they see across the textbook at three different levels:
- **Global**: Settings that apply to all chapters
- **Modules**: Settings for specific modules (overrides global settings)
- **Chapters**: Settings for specific chapters (overrides module and global settings)

## How It Works

### Preference Hierarchy

The system uses a three-level preference hierarchy:

```
Chapter Preference (most specific)
    ↓ (overrides if exists)
Module Preference
    ↓ (overrides if exists)
Global Preference (least specific)
    ↓ (default)
System Defaults
```

When displaying content, the system checks preferences in this order:
1. **Chapter-level preference** - If a specific chapter has preferences set, use those
2. **Module-level preference** - If no chapter preference but module has settings, use module preferences
3. **Global preference** - If no module preference, use global settings
4. **System defaults** - If no preferences set, use default settings (Beginner, all content shown)

### Customizable Settings

Each preference level allows you to configure:

#### Content Level
- **Beginner**: Introductory concepts, simplified explanations
- **Intermediate**: Building on basics, more technical depth
- **Advanced**: In-depth topics, advanced concepts

#### Content Toggles
- **Show Mathematical Content**: Enable/disable mathematical proofs and derivations
- **Show Code Examples**: Enable/disable programming examples
- **Show Diagrams & Visuals**: Enable/disable illustrations and visual aids
- **Show Advanced Topics**: Enable/disable advanced/optional topics
- **Focus Areas**: (Future) Array of specific topics to focus on

## Using the Feature

### 1. Global Settings Tab

**Location**: Personalization Panel → Global Tab

**What it does**: Sets preferences that apply to all chapters in the textbook

**How to use**:
1. Open Personalization panel from the application header
2. Click on the "Global" tab
3. Select your preferred content level
4. Toggle the checkboxes for content types you want to see
5. Click "Save Preferences"

**Example**:
```
Content Level: Beginner
✓ Show Mathematical Content
✓ Show Code Examples
✓ Show Diagrams & Visuals
☐ Show Advanced Topics
```

### 2. Modules Tab

**Location**: Personalization Panel → Modules Tab

**What it does**: Override global settings for specific modules

**How to use**:

#### Option A: Create New Module Preference
1. Open Personalization panel
2. Click on the "Modules" tab
3. Click "+ Create new module preference..." from the dropdown
4. Enter the Module ID (e.g., "robotics_101", "neural_networks")
5. Click "Continue with this Module"
6. Configure the settings for this module
7. Click "Save Module Settings"

#### Option B: Edit Existing Module Preference
1. Open Personalization panel
2. Click on the "Modules" tab
3. Select the module from the dropdown
4. Modify the settings
5. Click "Save Module Settings"

**Example - Create preference for "robotics_101" module**:
```
Module ID: robotics_101
Content Level: Intermediate
✓ Show Mathematical Content
✓ Show Code Examples
✓ Show Diagrams & Visuals
✓ Show Advanced Topics  (enabled for robotics)
```

This will override the global "Beginner" level for all chapters in the robotics_101 module.

### 3. Chapters Tab

**Location**: Personalization Panel → Chapters Tab

**What it does**: Override module/global settings for specific chapters

**How to use**:

#### Option A: Create New Chapter Preference
1. Open Personalization panel
2. Click on the "Chapters" tab
3. Click "+ Create new chapter preference..." from the dropdown
4. Enter the Chapter ID (e.g., "chapter_1_intro")
5. (Optional) Enter Module ID if this chapter belongs to a module
6. Click "Continue with this Chapter"
7. Configure the settings for this chapter
8. Click "Save Chapter Settings"

#### Option B: Edit Existing Chapter Preference
1. Open Personalization panel
2. Click on the "Chapters" tab
3. Select the chapter from the dropdown
4. Modify the settings
5. Click "Save Chapter Settings"

**Example - Advanced settings for a specific chapter**:
```
Chapter ID: chapter_3_advanced_control
Module ID: robotics_101
Content Level: Advanced
✓ Show Mathematical Content
✓ Show Code Examples
✓ Show Diagrams & Visuals
✓ Show Advanced Topics
```

This chapter will show all advanced content, regardless of global or module settings.

## Real-World Scenario

Imagine a user learning robotics:

**Global Settings**:
- Content Level: Beginner
- Show everything for foundations

**Module Settings** (for "robotics_101"):
- Content Level: Intermediate
- Hide Advanced Topics initially (unchecked)

**Chapter Settings** (for "chapter_5_advanced_control"):
- Content Level: Advanced
- Show Advanced Topics (checked)

**Result**:
- Most chapters show beginner level
- Robotics module chapters show intermediate level
- Chapter 5 shows advanced content

When the user reaches chapter 5, they'll automatically see more advanced material tailored for that specific chapter.

## API Endpoints

### Backend Endpoints

All endpoints require authentication (Bearer token):

#### Global Preference
```
POST /api/personalization/global-preference
```

Request body:
```json
{
  "content_level": "beginner|intermediate|advanced",
  "show_math": true,
  "show_code": true,
  "show_diagrams": true,
  "show_advanced_topics": false,
  "focus_areas": ["robotics", "control"]
}
```

#### Module Preference
```
POST /api/personalization/module-preference
```

Request body:
```json
{
  "module_id": "robotics_101",
  "content_level": "beginner|intermediate|advanced",
  "show_math": true,
  "show_code": true,
  "show_diagrams": true,
  "show_advanced_topics": false
}
```

#### Chapter Preference
```
POST /api/personalization/chapter-preference
```

Request body:
```json
{
  "chapter_id": "chapter_1",
  "module_id": "robotics_101",
  "content_level": "beginner|intermediate|advanced",
  "show_math": true,
  "show_code": true,
  "show_diagrams": true,
  "show_advanced_topics": false
}
```

#### Get All Preferences
```
GET /api/personalization/all
```

Response:
```json
{
  "global_preference": {...},
  "module_preferences": {
    "module_id": {...}
  },
  "chapter_preferences": {
    "chapter_id": {...}
  }
}
```

## Technical Details

### Frontend Components
- **PersonalizationPanel.jsx** (src/components/): Main UI component with three tabs
  - Uses state management for three modes: select, create, edit
  - Handles loading/error states
  - Integrates with authentication

### Backend Services
- **personalization_service.py**: Core business logic
  - TerminologyService: Manages terminology
  - PersonalizationService: Manages content preferences

- **endpoints/personalization.py**: FastAPI routes
  - All endpoints require JWT authentication
  - Comprehensive error handling

### Database
- **ContentPreference model** (auth_models.py)
  - Stores user preferences at different levels
  - Indexes for fast queries
  - UUID primary keys for distributed systems

## Known Limitations

1. **Focus Areas**: Currently stored but not used for content filtering (Phase 2 feature)
2. **Content Filtering**: Display logic not yet implemented (relies on frontend components to respect preferences)
3. **Batch Updates**: Cannot update multiple preferences at once (design for simplicity)

## Future Enhancements

### Phase 2
- Implement actual content filtering based on preferences
- Focus areas recommendation engine
- Analytics on user preference patterns

### Phase 3
- Template preferences for common learning styles
- Import/export preference profiles
- Peer recommendations ("Other learners like you preferred...")

## Troubleshooting

### Module preferences not showing?
- Ensure the module_id matches exactly (case-sensitive)
- Check that you've saved the preference by seeing success message

### Preferences not persisting?
- Verify you're logged in (check JWT token in browser dev tools)
- Check browser console for API errors

### Chapters dropdown empty?
- You need to create at least one chapter preference first
- Use "Create new chapter preference..." option

## Testing

The feature has been tested with:
- ✓ Global preference creation and updates
- ✓ Module preference creation with new module IDs
- ✓ Chapter preference creation with optional module IDs
- ✓ Preference hierarchy and retrieval
- ✓ Database persistence across sessions
- ✓ UI workflow for all three tabs

Test command:
```bash
bash test_translate_issue.sh  # Includes personalization tests
```

---

**Last Updated**: 2025-12-30
**Status**: ✓ Working and Production Ready
