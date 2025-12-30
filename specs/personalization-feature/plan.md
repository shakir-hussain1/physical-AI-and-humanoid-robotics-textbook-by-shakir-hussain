# Personalization Feature - Implementation Plan

**Intelligent content filtering and customization based on user background profile**

**Date:** 2025-12-28
**Status:** Active
**Owner:** Development Team

---

## Executive Summary

**Project**: Content Personalization System
**Duration**: 3-4 weeks (180-200 hours)
**Team Size**: 2-3 developers (1 backend, 1 frontend, 1 full-stack optional)
**Effort Breakdown**:
- Phase 1 (Backend): 60 hours / 7-8 days
- Phase 2 (Frontend): 70 hours / 8-9 days
- Phase 3 (Integration & Testing): 40 hours / 5 days
- Phase 4 (Documentation & Deployment): 20 hours / 2-3 days

**Key Deliverables**:
- Personalization API endpoints (5 endpoints)
- Content section markup system
- React personalization panel component
- Database schema for preferences
- Analytics tracking
- 160+ implementation tasks

---

## 1. Scope & Dependencies

### In Scope
- Personalization button on all chapters
- Personalization panel with toggles and options
- Per-chapter preference storage
- Content filtering (show/hide sections)
- Preference persistence (DB + localStorage)
- Analytics tracking
- Mobile responsive UI
- Keyboard accessibility
- RAG integration for recommended content

### Out of Scope (Phase 2+)
- ML-based adaptive difficulty
- Automatic difficulty detection
- LLM content regeneration
- Video personalization
- Advanced analytics dashboards
- Personalization presets management
- Real-time collaborative personalization

### External Dependencies
- **Signup/Signin Feature** (CRITICAL): Must have completed
- **User Profiles**: Must have background data from signup
- **RAG System**: For content recommendations
- **Database**: PostgreSQL with user data
- **Frontend Framework**: React (existing)
- **Docusaurus**: Chapter structure (existing)

### Blocking Factors
- Signup/Signin must be complete before personalization development
- Chapter content must be marked with metadata (manageable effort)
- User profile data must be accessible from backend

---

## 2. Key Decisions & Rationale

### Decision 1: Section-Based Content Filtering (Not LLM Generation)

**Chosen Approach**: Pre-mark content sections in markdown with metadata tags
```markdown
<begin-section type="theory" difficulty="beginner,intermediate">
...content...
</end-section>
```

**Rationale**:
- **Simplicity**: No need for LLM; predictable performance
- **Reliability**: Consistent results; no hallucinations
- **Cost**: No LLM API costs
- **Control**: Authors control exactly what content shows
- **Performance**: Fast filtering on client side

**Alternatives Considered**:
- **Dynamic LLM Generation** (Phase 2): More powerful but complex, expensive, unpredictable
- **Single Content with Inline Markers**: Harder to maintain; sections harder to find
- **CSS-only Hiding**: Doesn't reduce page load; poor for large chapters

**Trade-offs**:
- Content requires manual markup (effort: 2-3 days)
- Can't automatically generate new explanations
- Limited to pre-written variants

**Future Migration Path**: Phase 2 can add LLM generation alongside markdown

### Decision 2: Preference Storage in Database + localStorage

**Chosen Approach**: Dual storage with sync
- **Database**: Source of truth for persistence across devices
- **localStorage**: Cache for fast offline access and instant UI updates

**Rationale**:
- **Performance**: localStorage <1ms; DB ~50ms
- **Offline Support**: Works without network
- **Sync**: Eventual consistency; last-write-wins
- **Reliability**: DB backup if localStorage cleared

**Alternatives Considered**:
- **Database Only**: Slow for initial page load (50ms latency)
- **localStorage Only**: Lost if user clears cache; not sync'd across devices
- **IndexedDB**: More complex; localStorage sufficient for this use case

**Trade-offs**:
- Added sync complexity
- Potential for conflicts (resolved with timestamps)
- Takes ~5KB localStorage per user

### Decision 3: Per-Chapter + Per-Module + Global Preference Hierarchy

**Chosen Approach**: Three-level preference system
- **Global**: Default preferences (applies to all chapters)
- **Per-Module**: Override global for entire module
- **Per-Chapter**: Most specific; overrides module and global

**Rationale**:
- **Flexibility**: Users can have module-specific settings
- **Usability**: Not forcing re-personalization for every chapter
- **Consistency**: Chapter settings inherit from module if not specified

**Alternatives Considered**:
- **Per-Chapter Only**: Too granular; users manually set for each chapter
- **Per-Module Only**: Less flexibility; can't customize individual chapters
- **Global Only**: Too coarse; users can't vary strategy per chapter

**Trade-offs**:
- Added database complexity
- Need to handle preference hierarchy/resolution
- More API endpoints

### Decision 4: Client-Side Rendering of Personalization

**Chosen Approach**: HTML sections marked with data attributes; JavaScript controls visibility

```html
<div data-personalize="section"
     data-type="theory"
     data-difficulty="intermediate,advanced"
     data-robot-types="humanoid">
  Content...
</div>
```

**Rationale**:
- **Performance**: Fast client-side filtering (no server round-trip)
- **Instant Updates**: Changes apply without page reload
- **Offline Support**: Works with cached HTML
- **Flexibility**: Easy to add new filters

**Alternatives Considered**:
- **Server-Side Rendering**: Slower; requires page reload on personalization change
- **SSR with Revalidation**: Complex; defeats instant update benefit

**Trade-offs**:
- Larger initial HTML (includes hidden sections)
- JavaScript required (acceptable for modern browsers)
- Needs accessibility testing for hidden content

### Decision 5: Integrate with Signup Profile Data

**Chosen Approach**: Pre-populate personalization from signup profile; auto-suggest settings

**Rationale**:
- **UX**: Users don't need to set preferences from scratch
- **Relevance**: Suggestions match user's stated background
- **Guidance**: Beginners get appropriate default level
- **Intelligence**: Preferences align with profile (humanoid interest → show humanoid examples)

**Trade-offs**:
- More complex preference initialization
- Need to handle profile updates → preference updates

---

## 3. Architecture Design

### System Components

```
┌─────────────────────────────────────────────────────────────────┐
│                      Frontend (React/Docusaurus)                 │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Chapter Component                                         │  │
│  │  ├─ Personalization Button ("🎯 Personalize Content")    │  │
│  │  ├─ Content Sections (marked with data-personalize)      │  │
│  │  └─ Analytics tracking on scroll/interaction             │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ PersonalizationPanel Component (Modal/Sidebar)           │  │
│  │  ├─ Difficulty Level Selector                            │  │
│  │  ├─ Content Type Toggles                                 │  │
│  │  ├─ Robot Type Checkboxes                                │  │
│  │  ├─ Learning Goal Selector                               │  │
│  │  ├─ Apply / Cancel Buttons                               │  │
│  │  └─ Preview of changes                                   │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ usePersonalization Hook                                  │  │
│  │  ├─ Load preferences from localStorage/API               │  │
│  │  ├─ Apply filters to DOM                                 │  │
│  │  ├─ Save preferences                                     │  │
│  │  └─ Sync with server                                     │  │
│  └──────────────────────────────────────────────────────────┘  │
└───────────────────────────↓────────────────────────────────────┘
                      HTTPS / JWT Cookies
┌───────────────────────────↓────────────────────────────────────┐
│                    FastAPI Backend                              │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Personalization Router (/api/personalization/*)          │  │
│  │  ├─ GET /chapters/{id}/personalization                   │  │
│  │  ├─ POST /chapters/{id}/personalization                  │  │
│  │  ├─ PUT /chapters/{id}/personalization                   │  │
│  │  ├─ GET /chapters/{id}/content?personalize=true          │  │
│  │  └─ GET /analytics/personalization                       │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Personalization Service                                  │  │
│  │  ├─ Preference resolution (hierarchy)                    │  │
│  │  ├─ Content filtering logic                              │  │
│  │  ├─ User profile data integration                        │  │
│  │  └─ Analytics aggregation                                │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ Content Service                                          │  │
│  │  ├─ Chapter metadata parsing                             │  │
│  │  ├─ Section extraction                                   │  │
│  │  ├─ Content filtering (based on preferences)             │  │
│  │  └─ Analytics event tracking                             │  │
│  └──────────────────────────────────────────────────────────┘  │
└───────────────────────────↓────────────────────────────────────┘
                      SQLAlchemy ORM
┌───────────────────────────↓────────────────────────────────────┐
│                    Database Layer                               │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ user_personalization_preferences table                   │  │
│  │  ├─ user_id (FK)                                         │  │
│  │  ├─ chapter_id                                           │  │
│  │  ├─ difficulty_level                                     │  │
│  │  ├─ show_theory, show_exercises, show_code, etc.         │  │
│  │  ├─ robot_types_enabled (JSON)                           │  │
│  │  ├─ learning_goal_filter                                 │  │
│  │  └─ updated_at                                           │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │ personalization_events table (analytics)                 │  │
│  │  ├─ user_id (FK)                                         │  │
│  │  ├─ chapter_id                                           │  │
│  │  ├─ event_type (button_click, panel_open, applied, etc.) │  │
│  │  ├─ preferences_changed_from → to                        │  │
│  │  └─ timestamp                                            │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

### Data Flow

**First-Time Personalization:**
```
1. User opens Chapter 1
2. Frontend loads chapter HTML
3. usePersonalization hook runs
4. Frontend checks localStorage - no preferences found
5. Frontend calls GET /api/chapters/{id}/personalization
6. Backend:
   - Retrieves user's profile (from signup)
   - Creates default preferences from profile
   - Returns available options + current (default) preferences
7. Frontend pre-populates panel with defaults
8. User adjusts settings in panel
9. User clicks Apply
10. Frontend saves to localStorage (instant)
11. Frontend calls POST /api/personalization (async)
12. Backend saves to database
13. Frontend applies filters to chapter content
14. Content sections fade/show based on preferences
15. Analytics event logged
```

**Returning to Personalized Chapter:**
```
1. User opens previously personalized Chapter 1
2. Frontend loads chapter HTML
3. usePersonalization hook checks localStorage
4. Preferences found in localStorage
5. Frontend applies filters immediately (no API call)
6. Button shows "🎯 Personalize (Modified)"
7. When user clicks button:
   - Panel opens with current preferences
   - Panel also fetches latest from server (background sync)
   - Conflicts resolved (server is source of truth)
```

---

## 4. Task Decomposition

### Phase 1: Backend Implementation (60 hours, 7-8 days)

**Database & Schema (Tasks 1-5, 8 hours)**
- Design user_personalization_preferences table schema
- Design personalization_events analytics table
- Write Alembic migrations
- Create database indexes
- Write seed data for testing

**API Endpoints (Tasks 6-15, 25 hours)**
- GET /chapters/{id}/personalization (get options + current preferences)
- POST /chapters/{id}/personalization (save preferences)
- PUT /chapters/{id}/personalization (update preferences)
- DELETE /chapters/{id}/personalization (reset to defaults)
- GET /chapters/{id}/content (return filtered content)
- GET /analytics/personalization (analytics data)
- Integration tests for each endpoint

**Business Logic (Tasks 16-25, 20 hours)**
- Implement preference resolution (hierarchy: chapter > module > global)
- Implement content filtering logic
- User profile integration (auto-suggest settings)
- Default preference initialization
- Preferences validation
- Unit tests

**Analytics & Logging (Tasks 26-30, 7 hours)**
- Analytics event schema
- Event logging on button clicks
- Event logging on preferences save
- Analytics aggregation queries
- Analytics endpoint implementation

### Phase 2: Frontend Implementation (70 hours, 8-9 days)

**Personalization Panel Component (Tasks 31-45, 20 hours)**
- Create PersonalizationPanel component (modal/sidebar)
- Difficulty level selector
- Content type toggles (theory, exercises, code, diagrams)
- Robot type checkboxes
- Learning goal selector
- Apply/Cancel/Reset buttons
- Preview of changes
- Styling and responsive design
- Unit tests

**usePersonalization Hook (Tasks 46-55, 15 hours)**
- Load preferences from localStorage
- Load preferences from API
- Sync preferences with server
- Apply filters to DOM (show/hide sections)
- Save preferences logic
- Preference hierarchy resolution
- localStorage sync logic
- Conflict resolution
- Unit tests

**Chapter Integration (Tasks 56-65, 15 hours)**
- Add personalization button to chapter component
- Personalization button state (default vs modified)
- Content sections markup (data-personalize attributes)
- Content filtering on page load
- Content filtering on preference change
- Smooth animations (fade in/out)
- Mobile responsive layout
- Keyboard accessibility
- Integration tests

**Analytics Frontend (Tasks 66-70, 10 hours)**
- Track button clicks
- Track panel opens/closes
- Track preferences changes
- Track scroll depth (time on section)
- Send analytics events to backend
- Local event queue (batch sending)

**Styling & UX (Tasks 71-75, 10 hours)**
- Component styling (match Docusaurus theme)
- Dark mode support
- Animations and transitions
- Mobile responsive design
- Accessibility (ARIA labels, keyboard nav)
- Cross-browser testing

### Phase 3: Integration & Testing (40 hours, 5 days)

**End-to-End Testing (Tasks 76-85, 15 hours)**
- Complete signup → personalize → view personalized content flow
- Multi-chapter personalization
- Preference sync between tabs
- Offline personalization with sync
- Cross-browser testing (Chrome, Firefox, Safari, mobile)
- Performance testing
- Load testing (concurrent users)

**Content Markup (Tasks 86-95, 15 hours)**
- Parse existing chapters and identify sections
- Mark sections with metadata (type, difficulty, language, robot types)
- Verify markup correctness
- Create markup guidelines for authors
- Example chapters fully marked
- Automated validation of markup

**Documentation (Tasks 96-105, 10 hours)**
- API documentation
- Frontend component documentation
- Content markup guide for authors
- Setup and deployment guide
- Troubleshooting guide
- Example implementations

### Phase 4: Documentation & Deployment (20 hours, 2-3 days)

**Final Testing (Tasks 106-110, 8 hours)**
- Security testing
- Performance validation
- Accessibility audit (WCAG 2.1 AA)
- Browser compatibility testing
- Mobile device testing

**Code Review & Cleanup (Tasks 111-115, 6 hours)**
- Backend code review
- Frontend code review
- Remove debug code/console logs
- Code formatting/linting
- Final quality checks

**Deployment Preparation (Tasks 116-120, 6 hours)**
- Pre-deployment checklist
- Database migration planning
- Rollback plan
- Monitoring and alerting setup
- Deployment runbook

---

## 5. Timeline & Milestones

### Capacity Planning
- **Backend Developer**: 80 hours/week (40 work + 30% overhead)
- **Frontend Developer**: 80 hours/week
- **Full-Stack Support** (Optional): 80 hours/week

### Realistic Schedule (with 2 developers)

**Week 1 (Days 1-5): Phase 1 - Backend**
- Day 1: Database design, migrations
- Day 2-3: API endpoints (GET, POST, PUT)
- Day 4-5: Business logic, preference resolution, testing

**Week 2 (Days 6-10): Phase 2 - Frontend**
- Day 1: Personalization panel component
- Day 2: usePersonalization hook
- Day 3-4: Chapter integration, content filtering
- Day 5: Styling and UX

**Week 3 (Days 11-15): Phase 3 - Integration**
- Day 1-2: End-to-end testing
- Day 3-4: Content markup across chapters
- Day 5: Documentation and examples

**Week 4 (Days 16-18): Phase 4 - Deployment**
- Day 1-2: Final testing, code review, cleanup
- Day 3: Deployment preparation, monitoring setup

**Total**: 3.5-4 weeks with 2 developers
**Buffer**: 15-20% built into estimates (already included in task hours)

### Key Milestones
- **Milestone 1** (End Week 1): Backend APIs complete and tested
- **Milestone 2** (End Week 2): Frontend complete; basic integration working
- **Milestone 3** (End Week 3): All chapters marked; full integration tested
- **Milestone 4** (End Week 4): Ready for production deployment

---

## 6. Risk Analysis

### Risk 1: Content Markup Too Time-Consuming (MEDIUM - Score 2.0)
**Probability**: 40% | **Impact**: 2-3 days (medium)

**Description**: Manually marking up 50+ chapters with metadata is slow and error-prone

**Mitigation**:
1. Create automated content parsing (90% automatic, 10% manual)
2. Build validation script to catch missing metadata
3. Start with 5-10 key chapters; use as examples
4. Create markup guidelines for future authors
5. Parallelize with development (don't block on markup)

**Contingency**: If markup takes >3 days, deploy with partially marked chapters; finish markup in Phase 2

### Risk 2: Performance Degradation with Large Content (MEDIUM - Score 1.8)
**Probability**: 30% | **Impact**: 2 days (medium)

**Description**: Filtering large chapters (10,000+ lines HTML) could cause layout shift or slow filtering

**Mitigation**:
1. Use efficient DOM selection (data attributes, classes)
2. Batch DOM updates (requestAnimationFrame)
3. Implement virtual scrolling for very large chapters
4. Load test with largest chapters
5. Profile JavaScript performance

**Contingency**: Implement lazy loading or pagination if performance issues

### Risk 3: Preference Sync Conflicts (LOW - Score 1.2)
**Probability**: 20% | **Impact**: 1 day (minor)

**Description**: User updates preferences on two devices simultaneously; conflict in sync

**Mitigation**:
1. Use timestamps for last-write-wins resolution
2. Show warning if local != server (offer sync)
3. Batch updates with debouncing
4. Add conflict detection and logging

**Contingency**: Manual sync resolution or reset to server version

### Risk 4: RAG Integration Delay (LOW - Score 1.0)
**Probability**: 15% | **Impact**: 1 day (minor)

**Description**: RAG system not ready for content recommendations (depends on RAG team)

**Mitigation**:
1. Build personalization independent of RAG
2. Add RAG integration as optional enhancement
3. Use basic keyword matching as fallback

**Contingency**: Deploy without RAG integration; add in Phase 2

### Risk 5: Browser Compatibility Issues (LOW - Score 0.8)
**Probability**: 20% | **Impact**: 0.5 days (minor)

**Description**: Some browsers don't support localStorage, data attributes, or animations

**Mitigation**:
1. Test on all target browsers from the start
2. Use polyfills for older browsers
3. Graceful degradation (fallback to default content if filtering fails)
4. Progressive enhancement approach

**Contingency**: Drop support for legacy browsers; upgrade to modern ones only

---

## 7. Testing Strategy

### Unit Tests
- Preference validation logic
- Content filtering algorithms
- Preference hierarchy resolution
- localStorage sync logic
- Analytics event creation

### Integration Tests
- Save and retrieve preferences
- Preference persistence
- Profile integration (auto-suggestions)
- API endpoint flows
- Database transactions

### System/E2E Tests
- Complete personalization workflow
- Multi-chapter scenarios
- Offline → online sync
- Cross-device sync
- Performance under load

### User Acceptance Tests
- Can personalize chapter content
- Preferences persist
- Content updates correctly
- Mobile UX works
- Accessibility compliant

---

## 8. Deployment Strategy

### Pre-Deployment Checklist
- [ ] All 120+ tasks completed
- [ ] Test coverage >80%
- [ ] Zero critical bugs
- [ ] Performance targets met (<100ms panel load)
- [ ] Accessibility audit passed (WCAG 2.1 AA)
- [ ] Database migrations tested
- [ ] Rollback plan documented
- [ ] Monitoring and alerts configured
- [ ] Team trained on new features

### Database Migration
- Run Alembic migrations in order
- Seed personalization_events table if needed
- Verify data integrity
- Backup before migration
- Rollback plan ready

### Deployment Steps
1. Deploy backend (new API endpoints)
2. Deploy frontend (new components)
3. Run database migrations
4. Monitor error rates and performance
5. Gradual rollout (10% → 25% → 50% → 100%)
6. Rollback if critical issues found

### Rollback Plan
- Reverse database migrations
- Deploy previous frontend version
- Deploy previous backend version
- Restore from backup if data corruption

---

## 9. Success Metrics

- ✅ All 120+ tasks completed on schedule
- ✅ 80%+ users interact with personalization
- ✅ <100ms panel load time
- ✅ <500ms content filtering
- ✅ 99.9% preference persistence
- ✅ 95%+ user satisfaction
- ✅ >80% test coverage
- ✅ Zero security vulnerabilities
- ✅ Accessibility score >90
- ✅ Performance score >85

---

## Related Decisions

- **ADR-009**: Content Section Metadata Format (TBD)
- **ADR-010**: Preference Storage Strategy (TBD)
- **ADR-011**: Client vs Server Content Filtering (TBD)

---

**Document Status:** Ready for Task Decomposition
**Last Updated:** 2025-12-28
**Next Review Date:** 2026-01-28
