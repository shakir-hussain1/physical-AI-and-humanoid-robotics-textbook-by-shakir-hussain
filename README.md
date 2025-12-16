Physical AI & Humanoid Robotics: Spec-Kit Plus Configuration
This directory contains the configuration for the Spec-Kit Plus system used in the "Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World" project.

Structure
.specify/
├── memory/
│   └── constitution.md          # Project constitution and governance
├── templates/
│   ├── plan-template.md         # Template for implementation plans
│   ├── spec-template.md         # Template for feature specifications
│   └── tasks-template.md        # Template for implementation tasks
├── commands/                    # Command configuration files
└── README.md                   # This file
Project Constitution
The project constitution (.specify/memory/constitution.md) defines the core principles and governance structure for the textbook project:

Technical Excellence and Accuracy - Ensuring highest standards in Physical AI and robotics content
Educational Pedagogy - Following evidence-based educational practices
Accessibility and Inclusion - Making content accessible to diverse learners
Practical Application - Bridging theory with real-world implementation
Ethical Responsibility - Considering ethical implications of humanoid robotics
Collaborative Intelligence - Human-AI collaboration in content creation
Module-Based Structure - Organized around six core modules
Citation and Reference Standards - Proper academic citations
Core Modules
The textbook is structured around six core modules:

Introduction to Physical AI & Embodied Intelligence
The Robotic Nervous System (ROS 2) - Nodes, Topics, Services, Actions; rclpy Python integration; URDF for humanoid robots
The Digital Twin - Gazebo physics simulation; Unity visualization; Sensor simulation (LiDAR, depth cameras, IMUs)
The AI-Robot Brain (NVIDIA Isaac) - Isaac Sim; Synthetic data generation; Isaac ROS and Nav2
Vision-Language-Action (VLA) - LLM-based planning; Voice-to-action pipelines; Natural language → ROS action graphs
Capstone: Autonomous Humanoid Robot - Voice command reception; Navigation and obstacle avoidance; Object detection
Templates
The templates provide standardized formats for project artifacts:

Plan Template: Architecture and implementation planning
Spec Template: Feature specifications and requirements
Tasks Template: Implementation task breakdowns
Usage
These templates and the constitution guide all content creation, technical implementation, and project governance for the textbook project. All contributions should align with the constitutional principles and use the appropriate templates for planning, specification, and task management.

Governance
Changes to the constitution require:

Proposal with rationale documented in a Prompt History Record (PHR)
Review by project maintainers
Consensus approval for minor changes or majority for major changes
Update to all dependent artifacts
Claude Code Skills
In addition to the Spec-Kit Plus configuration, this project also includes Claude Code skills in the .skills/ directory for AI-assisted content generation and validation across all six modules.
