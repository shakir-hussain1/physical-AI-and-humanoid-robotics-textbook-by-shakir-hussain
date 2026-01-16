# Tasks: ROS 2 Module Chapters (Module 1)

**Input**: Design documents from `/specs/001-ros2-module-chapters/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)
**Branch**: `001-ros2-module-chapters`

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup and Infrastructure

**Purpose**: Create Docusaurus directory structure and foundational templates

- [ ] T001 Create Module 1 directory structure at `docs/modules/module-1-ros2/`
- [ ] T002 [P] Create `docs/modules/module-1-ros2/_category_.json` for Docusaurus sidebar
- [ ] T003 [P] Create `docs/assets/module-1/diagrams/` directory
- [ ] T004 [P] Create `docs/assets/module-1/screenshots/` directory
- [ ] T005 [P] Create `docs/code-examples/module-1/chapter-01/` directory
- [ ] T006 [P] Create `docs/code-examples/module-1/chapter-02/` directory
- [ ] T007 [P] Create `docs/code-examples/module-1/chapter-03/` directory
- [ ] T008 [P] Create `docs/code-examples/module-1/chapter-04/` directory
- [ ] T009 Create chapter template with standard sections (learning objectives, common errors, exercises)
- [ ] T010 Set up ROS 2 Jazzy workspace for testing code examples

**Checkpoint**: Directory structure ready for content creation

---

## Phase 2: User Story 1 - Learning ROS 2 Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Students understand ROS 2 architecture and can create basic ROS 2 programs

**Independent Test**: Student completes Chapter 1-2 exercises and successfully runs a multi-node ROS 2 application

### Chapter 1: Introduction to ROS 2 (Week 3)

- [ ] T011 [US1] Write learning objectives and introduction for `docs/modules/module-1-ros2/chapter-01-intro.md`
- [ ] T012 [P] [US1] Create Mermaid diagram: ROS 2 Architecture Overview
- [ ] T013 [P] [US1] Create Mermaid diagram: DDS Middleware Communication Pattern
- [ ] T014 [P] [US1] Create Mermaid diagram: Workspace and Package Structure
- [ ] T015 [US1] Write Section 1: What is ROS 2? (History, evolution from ROS 1)
- [ ] T016 [US1] Write Section 2: ROS 2 Architecture Overview (Nodes, topics, DDS)
- [ ] T017 [US1] Write Section 3: Installation Guide - Ubuntu 22.04 native
- [ ] T018 [US1] Write Section 3b: Installation Guide - WSL2 for Windows
- [ ] T019 [US1] Write Section 3c: Installation Guide - Docker alternative
- [ ] T020 [US1] Write Section 4: Your First ROS 2 Workspace (colcon build)
- [ ] T021 [US1] Write Section 5: ROS 2 Command-Line Tools
- [ ] T022 [US1] Write Common Errors and Solutions section
- [ ] T023 [US1] Create Exercise 1.1 (Basic): Install ROS 2 and verify with ros2 doctor
- [ ] T024 [US1] Create Exercise 1.2 (Intermediate): Create and build a custom workspace
- [ ] T025 [US1] Create Exercise 1.3 (Advanced): Explore turtlesim with CLI tools
- [ ] T026 [US1] Write chapter summary and key takeaways
- [ ] T027 [P] [US1] Create `docs/code-examples/module-1/chapter-01/hello_ros2.py`

**Checkpoint**: Chapter 1 complete - Students can install and configure ROS 2

### Chapter 2: Nodes, Topics, Services, and Actions (Week 4)

- [ ] T028 [US1] Write learning objectives and introduction for `docs/modules/module-1-ros2/chapter-02-comms.md`
- [ ] T029 [P] [US1] Create Mermaid diagram: Topic-based Communication Flow
- [ ] T030 [P] [US1] Create Mermaid diagram: Service Request-Response Pattern
- [ ] T031 [P] [US1] Create Mermaid diagram: Action Server with Feedback
- [ ] T032 [P] [US1] Create Mermaid diagram: Launch File Node Graph
- [ ] T033 [US1] Write Section 1: ROS 2 Nodes Deep Dive (Lifecycle, executors)
- [ ] T034 [US1] Write Section 2: Topic Communication (Publishers, subscribers, QoS)
- [ ] T035 [US1] Write Section 3: Service Communication (Request-response)
- [ ] T036 [US1] Write Section 4: Action Communication (Feedback, cancellation)
- [ ] T037 [US1] Write Section 5: Launch Files (Python-based launch)
- [ ] T038 [US1] Write Section 6: Hands-on - Building a Sensor Pipeline
- [ ] T039 [US1] Write Common Errors and Solutions section
- [ ] T040 [P] [US1] Create `docs/code-examples/module-1/chapter-02/publisher_node.py`
- [ ] T041 [P] [US1] Create `docs/code-examples/module-1/chapter-02/subscriber_node.py`
- [ ] T042 [P] [US1] Create `docs/code-examples/module-1/chapter-02/service_server.py`
- [ ] T043 [P] [US1] Create `docs/code-examples/module-1/chapter-02/service_client.py`
- [ ] T044 [P] [US1] Create `docs/code-examples/module-1/chapter-02/action_server.py`
- [ ] T045 [P] [US1] Create `docs/code-examples/module-1/chapter-02/action_client.py`
- [ ] T046 [US1] Create `docs/code-examples/module-1/chapter-02/sensor_pipeline.launch.py`
- [ ] T047 [US1] Create Exercise 2.1 (Basic): Create a publisher that sends sensor data
- [ ] T048 [US1] Create Exercise 2.2 (Basic): Create a subscriber that processes messages
- [ ] T049 [US1] Create Exercise 2.3 (Intermediate): Build a service for robot commands
- [ ] T050 [US1] Create Exercise 2.4 (Advanced): Create an action server for navigation
- [ ] T051 [US1] Write chapter summary and key takeaways
- [ ] T052 [US1] Test all code examples end-to-end

**Checkpoint**: Chapter 2 complete - Students understand ROS 2 communication patterns

---

## Phase 3: User Story 2 - Building Humanoid Robot Descriptions (Priority: P2)

**Goal**: Students can create URDF descriptions for humanoid robots and visualize them

**Independent Test**: Student creates a complete URDF file for humanoid robot and visualizes in RViz2

### Chapter 3: URDF for Humanoid Robot Bodies (Week 5, Part 1)

- [ ] T053 [US2] Write learning objectives and introduction for `docs/modules/module-1-ros2/chapter-03-urdf.md`
- [ ] T054 [P] [US2] Create Mermaid diagram: URDF Link-Joint Tree Structure
- [ ] T055 [P] [US2] Create Mermaid diagram: Joint Types Overview
- [ ] T056 [P] [US2] Create Mermaid diagram: Humanoid Robot Link Hierarchy
- [ ] T057 [US2] Write Section 1: Introduction to Robot Description Formats (URDF vs SDF)
- [ ] T058 [US2] Write Section 2: URDF Fundamentals (Links, joints, materials)
- [ ] T059 [US2] Write Section 3: Building a Humanoid Torso and Arms
- [ ] T060 [US2] Write Section 4: Building Bipedal Legs with Joint Limits
- [ ] T061 [US2] Write Section 5: Adding a Sensor Head (Camera, IMU frames)
- [ ] T062 [US2] Write Section 6: Visualization with RViz2
- [ ] T063 [US2] Write Section 7: URDF Best Practices for Simulation
- [ ] T064 [US2] Write Common Errors and Solutions section
- [ ] T065 [US2] Create `docs/code-examples/module-1/chapter-03/humanoid_urdf/humanoid.urdf`
- [ ] T066 [US2] Create `docs/code-examples/module-1/chapter-03/launch/display.launch.py`
- [ ] T067 [US2] Create Exercise 3.1 (Basic): Build a simple robot arm URDF
- [ ] T068 [US2] Create Exercise 3.2 (Intermediate): Add a head with sensor frames
- [ ] T069 [US2] Create Exercise 3.3 (Advanced): Complete humanoid with 10+ links
- [ ] T070 [US2] Write chapter summary and key takeaways
- [ ] T071 [US2] Test URDF in RViz2 with joint_state_publisher
- [ ] T072 [US2] Verify URDF compatibility with Gazebo Harmonic

**Checkpoint**: Chapter 3 complete - Students can create and visualize humanoid URDF

---

## Phase 4: User Story 3 - AI-ROS Integration (Priority: P3)

**Goal**: Students can bridge AI systems with ROS 2 control pipelines

**Independent Test**: Student creates a ROS 2 service that accepts text commands and triggers robot actions

### Chapter 4: Bridging AI and ROS 2 Control (Week 5, Part 2)

- [ ] T073 [US3] Write learning objectives and introduction for `docs/modules/module-1-ros2/chapter-04-ai-bridge.md`
- [ ] T074 [P] [US3] Create Mermaid diagram: AI Command to ROS 2 Action Flow
- [ ] T075 [P] [US3] Create Mermaid diagram: State Machine for Behavior Execution
- [ ] T076 [P] [US3] Create Mermaid diagram: Custom Message/Service Architecture
- [ ] T077 [US3] Write Section 1: The AI-Robot Interface Challenge
- [ ] T078 [US3] Write Section 2: Custom Messages and Services (IDL definition)
- [ ] T079 [US3] Write Section 3: Action Servers for Complex Behaviors
- [ ] T080 [US3] Write Section 4: Building a Command Interpreter Node
- [ ] T081 [US3] Write Section 5: Design Patterns for AI Integration
- [ ] T082 [US3] Write Section 6: Preparing for VLA Systems (Module 4 preview)
- [ ] T083 [US3] Write Common Errors and Solutions section
- [ ] T084 [P] [US3] Create custom message definitions in `docs/code-examples/module-1/chapter-04/custom_msgs/`
- [ ] T085 [P] [US3] Create `docs/code-examples/module-1/chapter-04/command_interpreter.py`
- [ ] T086 [P] [US3] Create `docs/code-examples/module-1/chapter-04/behavior_action_server.py`
- [ ] T087 [US3] Create Exercise 4.1 (Basic): Define custom message for robot command
- [ ] T088 [US3] Create Exercise 4.2 (Intermediate): Build action server for multi-step behavior
- [ ] T089 [US3] Create Exercise 4.3 (Advanced): Create command interpreter with queue
- [ ] T090 [US3] Write chapter summary and key takeaways
- [ ] T091 [US3] Test all integration code end-to-end

**Checkpoint**: Chapter 4 complete - Students can integrate AI with ROS 2

---

## Phase 5: User Story 4 - Curriculum Alignment (Priority: P2)

**Goal**: Module aligns with Weeks 3-5 of curriculum and prepares students for Module 2

**Independent Test**: Content maps to weekly learning objectives with appropriate depth

### Module Integration

- [ ] T092 [US4] Create module index/overview page at `docs/modules/module-1-ros2/index.md`
- [ ] T093 [US4] Write Module 1 overview with learning path and prerequisites
- [ ] T094 [US4] Verify cross-chapter references are correct
- [ ] T095 [US4] Validate all Mermaid diagrams render correctly
- [ ] T096 [US4] Run Docusaurus build and fix any errors
- [ ] T097 [US4] Review reading level across all chapters (grade 10-12 target)
- [ ] T098 [US4] Verify URDF compatibility with Module 2 Gazebo content
- [ ] T099 [US4] Create glossary entries for Module 1 terms
- [ ] T100 [US4] Map content to Week 3-5 curriculum objectives

**Checkpoint**: Module 1 complete and integrated

---

## Phase 6: Polish and Quality Assurance

**Purpose**: Final review, documentation, and quality validation

- [ ] T101 [P] Review all code examples compile without errors
- [ ] T102 [P] Verify all terminal commands show expected output
- [ ] T103 [P] Check all installation steps work on fresh Ubuntu 22.04
- [ ] T104 [P] Validate WSL2 instructions work on Windows 11
- [ ] T105 Add troubleshooting for common student issues
- [ ] T106 Final pedagogy review - ensure progressive complexity
- [ ] T107 Peer review for technical accuracy
- [ ] T108 Final Docusaurus build verification

**Checkpoint**: Module 1 ready for publication

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately
- **Phase 2 (US1 - ROS 2 Fundamentals)**: Depends on Phase 1 completion
- **Phase 3 (US2 - URDF)**: Depends on Phase 1; can parallel with Phase 2 after T027
- **Phase 4 (US3 - AI Bridge)**: Depends on Phase 2 completion (needs node concepts)
- **Phase 5 (US4 - Integration)**: Depends on Phases 2-4 completion
- **Phase 6 (Polish)**: Depends on all content phases complete

### User Story Dependencies

- **US1 (ROS 2 Fundamentals)**: Foundation - no dependencies on other stories
- **US2 (URDF)**: Can reference US1 concepts, independently testable
- **US3 (AI Bridge)**: Requires US1 node/action understanding
- **US4 (Curriculum)**: Requires all other stories complete for review

### Parallel Opportunities

- All diagrams within a chapter marked [P] can be created in parallel
- All code examples within a chapter marked [P] can be created in parallel
- Chapters 1-2 installation/concepts can proceed before URDF work
- All Phase 6 tasks marked [P] can run in parallel

---

## Implementation Strategy

### MVP First (Chapters 1-2)

1. Complete Phase 1: Setup
2. Complete Phase 2: Chapter 1 + Chapter 2
3. **VALIDATE**: Test all code examples
4. Students can begin practicing ROS 2 fundamentals

### Full Module Delivery

1. Complete Setup + Chapter 1-2 → ROS 2 communication ready
2. Add Chapter 3 → URDF and visualization ready
3. Add Chapter 4 → AI integration patterns ready
4. Complete Integration → Module complete
5. Polish → Publication ready

---

## Validation Checklist

Before completing each chapter:

- [ ] All learning objectives addressed
- [ ] 3+ Mermaid diagrams included and render correctly
- [ ] All code examples compile and run on ROS 2 Jazzy
- [ ] All terminal commands show expected output
- [ ] Reading level appropriate (grade 10-12)
- [ ] Common errors section complete
- [ ] Exercises created with solutions
- [ ] Cross-references to other sections correct
- [ ] Glossary terms identified
- [ ] Docusaurus build succeeds

---

## Notes

- All Python code uses rclpy (ROS 2 Python client library)
- Target ROS 2 distribution: Jazzy (with Humble compatibility notes)
- All code must be complete and runnable - no pseudo-code
- URDF must be compatible with Gazebo Harmonic for Module 2
- AI integration patterns prepare students for Module 4 VLA content
- 108 total tasks across 6 phases
