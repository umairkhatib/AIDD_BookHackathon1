---
id: 2
title: Physical AI Humanoid Course Spec
stage: spec
date: 2026-01-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-humanoid
branch: 001-physical-ai-humanoid
user: user
command: /sp.specify
labels: [course,robotics,ai,ros2,simulation,education]
links:
  spec: specs/001-physical-ai-humanoid/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-physical-ai-humanoid/spec.md
 - specs/001-physical-ai-humanoid/checklists/requirements.md
tests:
 - none
---

## Prompt

/sp.specify

Course: Physical AI & Humanoid Robotics
Theme: Embodied Intelligence — AI Systems in the Physical World
Goal: Enable students to design, simulate, and control humanoid robots using modern AI and robotics stacks.

Platform:
- Documentation framework: Docusaurus (MDX)
- Content style: Technical, hands-on, spec-driven
- Simulation-first, real-world–ready mindset

Target audience:
- Advanced AI students
- Robotics engineers
- Developers transitioning from software AI to embodied AI

Global success criteria:
- Learner understands end-to-end humanoid AI architecture
- Learner can explain and implement perception → planning → action
- Learner completes a working simulated humanoid capstone
- All concepts demonstrated via ROS 2–based workflows

Constraints:
- Format: Docusaurus MDX modules
- Code language: Python (primary), ROS 2 tooling
- Simulations must be reproducible on consumer GPUs
- No vendor lock-in beyond free/open tooling

--------------------------------------------------
Module 1: The Robotic Nervous System (ROS 2)

Focus:
- Robot middleware and real-time control

Learning outcomes:
- Explain ROS 2 architecture (nodes, topics, services)
- Build Python ROS 2 nodes using rclpy
- Connect AI agents to low-level robot controllers
- Model humanoid robots using URDF

Success criteria:
- Learner creates a ROS 2 node controlling a simulated joint
- Learner understands message flow and robot state updates

Not building:
- Custom ROS 2 middleware
- Real hardware drivers

--------------------------------------------------
Module 2: The Digital Twin (Gazebo & Unity)

Focus:
- Physics simulation and virtual environments

Learning outcomes:
- Simulate gravity, collisions, and dynamics in Gazebo
- Build interactive robot environments
- Understand Unity's role in human-robot interaction
- Simulate sensors (LiDAR, depth cameras, IMUs)

Success criteria:
- Humanoid robot behaves realistically in simulation
- Sensors publish usable ROS 2 data streams

Not building:
- Custom physics engines
- Production-grade game environments

--------------------------------------------------
Module 3: The AI-Robot Brain (NVIDIA Isaac™)

Focus:
- Perception, navigation, and training acceleration

Learning outcomes:
- Use Isaac Sim for photorealistic simulation
- Generate synthetic training data
- Apply Isaac ROS for accelerated VSLAM
- Implement Nav2 for humanoid path planning

Success criteria:
- Robot autonomously navigates a simulated environment
- Learner understands perception → localization → navigation flow

Not building:
- Custom SLAM algorithms
- Low-level GPU kernels

--------------------------------------------------
Module 4: Vision-Language-Action (VLA)

Focus:
- LLM-driven cognitive robotics

Learning outcomes:
- Convert voice commands to actions using Whisper
- Use LLMs for task planning and decomposition
- Translate natural language goals into ROS 2 action sequences
- Integrate vision, planning, and manipulation

Capstone project:
- Autonomous humanoid performing complex tasks

## Response snapshot

Created a comprehensive specification for the Physical AI & Humanoid Robotics course. The specification includes four main modules covering ROS 2 fundamentals, simulation environments, perception/navigation with NVIDIA Isaac, and LLM-driven robotics. The spec defines clear user scenarios, functional requirements, and success criteria aligned with the course objectives. A quality checklist was also created to validate the specification meets all required standards.

## Outcome

- ✅ Impact: Created complete course specification with user stories, requirements, and success metrics
- 🧪 Tests: none
- 📁 Files: specs/001-physical-ai-humanoid/spec.md, specs/001-physical-ai-humanoid/checklists/requirements.md
- 🔁 Next prompts: Ready for /sp.plan to create implementation plan
- 🧠 Reflection: Successfully transformed course concept into structured specification with measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none