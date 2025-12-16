---
name: Create Physical AI Textbook Spec
model: claude-sonnet-4-5-20250929
description: Create comprehensive specification for Physical AI & Humanoid Robotics textbook with user scenarios, functional requirements, and success criteria.
---

## Prompt

"""Project

Title: Physical AI & Humanoid Robotics: Embodied Intelligence from Simulation to Reality

Type: Academic textbook + applied capstone guide
Domain: Physical AI, Robotics, Embodied Intelligence
Primary Use: Upper-level CS / Robotics course textbook with integrated RAG chatbot

Target Audience

Computer Science students (senior undergraduate / graduate)

AI engineers transitioning from software-only AI to robotics

Educators designing Physical AI & humanoid robotics curricula

Assumed Background

Python programming

Basic AI / ML concepts

No prior robotics experience assumed

Focus

AI systems operating in the physical world

Embodied intelligence under real-world constraints

Bridging:

Digital AI models

Robotics middleware

Physics-based simulation

Perception, planning, and action

Vision–Language–Action (VLA) systems for humanoid robots

Primary Goal

Enable readers to design, simulate, and deploy an autonomous humanoid robot that:

Receives natural language voice commands

Translates language into robotic actions

Navigates a physical environment

Perceives objects visually

Executes manipulation tasks using ROS 2–based control

📚 Book Structure (Chapter-Level Specification)
Chapter 1 — Introduction to Physical AI

Purpose: Establish conceptual foundations
Key Topics:

From digital AI to embodied intelligence

Physical laws as constraints on intelligence

Why humanoid robots matter

Overview of modern humanoid robotics ecosystem

Outcome:
Reader understands why Physical AI is fundamentally different from software-only AI.

Chapter 2 — Sensors and Perception in the Physical World

Key Topics:

Sensor modalities: RGB, depth, LiDAR, IMU, force/torque

Noise, latency, calibration, and drift

Sensor fusion fundamentals

Outcome:
Reader can explain how raw physical signals become usable AI inputs.

Chapter 3 — The Robotic Nervous System: ROS 2

Key Topics:

ROS 2 architecture and design philosophy

Nodes, topics, services, actions

Real-time constraints

Python-based ROS development using rclpy

Outcome:
Reader can design a modular robot control system using ROS 2.

Chapter 4 — Robot Body Representation (URDF & Kinematics)

Key Topics:

URDF and SDF formats

Links, joints, and coordinate frames

Humanoid kinematic chains

Collision vs visual models

Outcome:
Reader can describe and modify a humanoid robot's digital body.

Chapter 5 — Digital Twins and Physics Simulation (Gazebo & Unity)

Key Topics:

Physics simulation fundamentals

Gravity, collisions, friction

Sensor simulation

Gazebo vs Unity roles

Outcome:
Reader understands how simulation approximates real-world physics and where it fails.

Chapter 6 — NVIDIA Isaac Sim and Synthetic Data

Key Topics:

Isaac Sim architecture

Photorealistic simulation

Synthetic data generation

Domain randomization

Outcome:
Reader understands how large-scale simulated data enables Physical AI training.

Chapter 7 — AI Perception and Navigation (Isaac ROS & Nav2)

Key Topics:

Visual SLAM (VSLAM)

Localization and mapping

Path planning for humanoids

Hardware acceleration

Outcome:
Reader can explain how robots perceive and move autonomously.

Chapter 8 — Learning to Act: Reinforcement Learning for Robots

Key Topics:

RL in continuous control

Training in simulation

Policy transfer challenges

Safety constraints

Outcome:
Reader understands how robots learn physical behaviors.

Chapter 9 — Sim-to-Real Transfer

Key Topics:

Reality gap

Latency and sensor mismatch

Deployment on Jetson edge devices

Failure modes

Outcome:
Reader can identify and mitigate sim-to-real challenges.

Chapter 10 — Vision-Language-Action (VLA) Systems

Key Topics:

Language grounding

Action planning from natural language

Multimodal reasoning

Cognitive architectures

Outcome:
Reader understands how language becomes robotic behavior.

Chapter 11 — Conversational Robotics

Key Topics:

Voice interfaces using Whisper

Dialogue management

Multi-modal interaction (speech, vision, gesture)

Human-robot interaction design

Outcome:
Reader can design natural human-robot communication systems.

Chapter 12 — Capstone: The Autonomous Humanoid

Key Topics:

End-to-end system integration

Voice → Plan → Navigate → Perceive → Manipulate

Performance evaluation

System limitations

Outcome:
Reader can conceptually design a complete autonomous humanoid system.

Chapter 13 — Hardware Architecture & Lab Design

Key Topics:

RTX workstation requirements

Jetson edge computing

Sensor hardware

Cloud vs on-prem tradeoffs

Outcome:
Reader understands infrastructure requirements for Physical AI education.

Chapter 14 — The Future of Physical AI

Key Topics:

Scaling humanoid intelligence

Ethical and safety considerations (high-level)

Research frontiers

Industry applications

Outcome:
Reader gains strategic understanding of where Physical AI is heading.

✅ Success Criteria

Reader can explain:

Physical AI vs digital AI

ROS 2-based robot control

Simulation-to-real workflows

Vision-Language-Action pipelines

Book supports a working simulated humanoid capstone
"""

## Response snapshot

Created comprehensive specification for the Physical AI & Humanoid Robotics textbook project. The spec includes four primary user stories covering CS students, AI engineers, educators, and VLA system implementation. It defines 12 functional requirements aligned with the book's 14 chapters and learning objectives. Success criteria include measurable outcomes for understanding Physical AI concepts, ROS 2 implementation, and VLA system design. A quality checklist was also created to validate the specification.

## Outcome

- ✅ Impact: Created complete feature specification for Physical AI textbook project
- 🧪 Tests: none
- 📁 Files: specs/001-physical-ai-humanoid/spec.md, specs/001-physical-ai-humanoid/checklists/requirements.md
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Successfully transformed comprehensive project description into structured specification with clear user scenarios and measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A