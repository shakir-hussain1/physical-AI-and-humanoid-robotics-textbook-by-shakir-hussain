# Physical AI & Humanoid Robotics: Spec-Kit Plus Configuration

This directory contains the configuration for the Spec-Kit Plus system used in the "Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World" project.

## Structure

```
.specify/
├── memory/
│   └── constitution.md          # Project constitution and governance
├── templates/
│   ├── plan-template.md         # Template for implementation plans
│   ├── spec-template.md         # Template for feature specifications
│   └── tasks-template.md        # Template for implementation tasks
├── commands/                    # Command configuration files
└── README.md                   # This file
```

## Project Constitution

The project constitution (`.specify/memory/constitution.md`) defines the core principles and governance structure for the textbook project:

1. **Technical Excellence and Accuracy** - Ensuring highest standards in Physical AI and robotics content
2. **Educational Pedagogy** - Following evidence-based educational practices
3. **Accessibility and Inclusion** - Making content accessible to diverse learners
4. **Practical Application** - Bridging theory with real-world implementation
5. **Ethical Responsibility** - Considering ethical implications of humanoid robotics
6. **Collaborative Intelligence** - Human-AI collaboration in content creation
7. **Module-Based Structure** - Organized around six core modules
8. **Citation and Reference Standards** - Proper academic citations

## Core Modules

The textbook is structured around six core modules:

1. **Introduction to Physical AI & Embodied Intelligence**
2. **The Robotic Nervous System (ROS 2)** - Nodes, Topics, Services, Actions; rclpy Python integration; URDF for humanoid robots
3. **The Digital Twin** - Gazebo physics simulation; Unity visualization; Sensor simulation (LiDAR, depth cameras, IMUs)
4. **The AI-Robot Brain (NVIDIA Isaac)** - Isaac Sim; Synthetic data generation; Isaac ROS and Nav2
5. **Vision-Language-Action (VLA)** - LLM-based planning; Voice-to-action pipelines; Natural language → ROS action graphs
6. **Capstone: Autonomous Humanoid Robot** - Voice command reception; Navigation and obstacle avoidance; Object detection

## Templates

The templates provide standardized formats for project artifacts:

- **Plan Template**: Architecture and implementation planning
- **Spec Template**: Feature specifications and requirements
- **Tasks Template**: Implementation task breakdowns

## Usage

These templates and the constitution guide all content creation, technical implementation, and project governance for the textbook project. All contributions should align with the constitutional principles and use the appropriate templates for planning, specification, and task management.

## Governance

Changes to the constitution require:
1. Proposal with rationale documented in a Prompt History Record (PHR)
2. Review by project maintainers
3. Consensus approval for minor changes or majority for major changes
4. Update to all dependent artifacts

## Claude Code Skills

In addition to the Spec-Kit Plus configuration, this project also includes Claude Code skills in the `.skills/` directory for AI-assisted content generation and validation across all six modules.