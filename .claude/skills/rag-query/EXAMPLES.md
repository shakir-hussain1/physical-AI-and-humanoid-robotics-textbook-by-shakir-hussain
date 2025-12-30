# RAG Query Examples

## Example 1: Factual Query

**User Question:**
> "What is ROS2?"

**Query Formulation:**
- Intent: Factual definition
- Key terms: ROS2, definition, features
- Expected: Definition + key characteristics

**System Response:**
```json
{
  "answer": "ROS2 (Robot Operating System 2) is a modern, flexible,
    middleware framework for robotics software development. It provides
    a distributed communication system, standard interfaces, and tools
    for building complex robotic systems. Key features include DDS-based
    communication, improved real-time performance, better security, and
    enhanced support for multi-robot systems.",
  "sources": [
    {
      "url": "chapter-2-ros-middleware",
      "page_title": "ROS2: Modern Robotics Middleware",
      "relevance_score": 0.96
    },
    {
      "url": "chapter-2-ros-architecture",
      "page_title": "ROS Architecture and Design",
      "relevance_score": 0.88
    }
  ],
  "confidence": "high",
  "metadata": {
    "latency_ms": 2143,
    "grounding_score": 0.94
  }
}
```

**Interpretation:**
- ✅ High confidence (>0.8)
- ✅ Excellent grounding (0.94)
- ✅ Direct match with high relevance (0.96)
- **Action:** Use answer directly with citations

---

## Example 2: Conceptual Query

**User Question:**
> "Explain inverse kinematics in humanoid robots"

**Query Formulation:**
- Intent: Conceptual explanation with context
- Key terms: Inverse kinematics, humanoid robots, calculation
- Expected: Definition + methodology + examples

**System Response:**
```json
{
  "answer": "Inverse kinematics (IK) is the mathematical process of
    calculating joint angles required to position a robot's end-effector
    at a desired location and orientation. In humanoid robots, IK is
    crucial for reaching tasks and maintaining balance. The process
    typically involves: 1) Defining the desired end-effector pose,
    2) Solving a system of non-linear equations, 3) Selecting from
    multiple solutions based on constraints, 4) Validating for joint
    limits. Common algorithms include Jacobian-based methods, analytical
    solutions for specific joint configurations, and numerical iterative
    approaches like Newton-Raphson.",
  "sources": [
    {
      "url": "chapter-4-kinematics",
      "page_title": "Kinematics: Forward and Inverse",
      "relevance_score": 0.93
    },
    {
      "url": "chapter-4-robot-arms",
      "page_title": "Robot Arm Kinematics",
      "relevance_score": 0.87
    },
    {
      "url": "chapter-5-motion-control",
      "page_title": "Motion Control and Reaching",
      "relevance_score": 0.82
    }
  ],
  "confidence": "high",
  "metadata": {
    "latency_ms": 2856,
    "grounding_score": 0.89
  }
}
```

**Interpretation:**
- ✅ High confidence
- ✅ Good grounding (0.89)
- ✅ Multiple relevant sources
- **Action:** Answer is well-supported, cite primary source

---

## Example 3: How-To Query

**User Question:**
> "How do humanoid robots walk?"

**Query Formulation:**
- Intent: Methodology/process
- Key terms: Walking, humanoid robots, motion, dynamics
- Expected: Step-by-step process + physics

**System Response:**
```json
{
  "answer": "Humanoid robots achieve walking through a coordinated
    process of: 1) Trajectory Planning - generating desired foot and
    body trajectories using walking generators like Linear Inverted
    Pendulum (LIP) models, 2) Inverse Kinematics - calculating joint
    angles to follow planned trajectories, 3) Dynamics Control -
    computing forces and moments needed for motion using inverse dynamics,
    4) Balance Control - maintaining stability through zero-moment-point
    (ZMP) constraints, 5) Feedback Control - adjusting motion based on
    sensor feedback from joint encoders and IMUs. The process cycles
    typically at 1-10 kHz to maintain smooth motion and stability.",
  "sources": [
    {
      "url": "chapter-6-walking",
      "page_title": "Walking Motion Generation",
      "relevance_score": 0.95
    },
    {
      "url": "chapter-5-dynamics",
      "page_title": "Dynamics and Force Control",
      "relevance_score": 0.89
    },
    {
      "url": "chapter-7-balance",
      "page_title": "Balance and Stability Control",
      "relevance_score": 0.86
    }
  ],
  "confidence": "high",
  "metadata": {
    "latency_ms": 3012,
    "grounding_score": 0.91
  }
}
```

**Interpretation:**
- ✅ Well-grounded procedural answer
- ✅ Multiple supporting sources
- **Action:** Cite primary source, use as main reference

---

## Example 4: Medium Confidence Query

**User Question:**
> "What are the latest advances in humanoid robotics?"

**Query Formulation:**
- Intent: Recent developments
- Key terms: Latest, advances, humanoid robotics
- Challenge: Textbook may not cover latest research

**System Response:**
```json
{
  "answer": "According to the textbook, key developments in humanoid
    robotics include: advances in motor control using reinforcement
    learning, improved compliant actuators for force control, integration
    of computer vision with motion planning, and development of digital
    twins for simulation. However, the textbook's coverage may not include
    the very latest 2025 advances, as it was written in 2024.",
  "sources": [
    {
      "url": "chapter-9-learning",
      "page_title": "Learning-Based Robot Control",
      "relevance_score": 0.72
    },
    {
      "url": "chapter-3-simulation",
      "page_title": "Digital Twins and Simulation",
      "relevance_score": 0.68
    }
  ],
  "confidence": "medium",
  "metadata": {
    "latency_ms": 2645,
    "grounding_score": 0.71
  }
}
```

**Interpretation:**
- ⚠️ Medium confidence (0.5-0.8)
- ⚠️ Fair grounding (0.71)
- ⚠️ Sources are somewhat relevant (0.68-0.72)
- **Action:** Use answer, acknowledge textbook limitations, suggest external research

---

## Example 5: Comparative Query

**User Question:**
> "Compare digital twins to physical simulation"

**Query Formulation:**
- Intent: Comparison/contrast
- Key terms: Digital twins, physical simulation, differences
- Expected: Feature comparison

**System Response:**
```json
{
  "answer": "Digital twins and physical simulations are related but
    distinct concepts. Physical simulations use mathematical models of
    robot dynamics, physics, and control to predict behavior in software.
    Digital twins go further by incorporating real-time data from physical
    robots to create a synchronized virtual counterpart that mirrors the
    physical system's state and behavior. Key differences: (1) Real-time
    sync - DTs use live data, sims are offline; (2) Bidirectionality -
    DTs can feed predictions back to physical systems, (3) Update rate -
    DTs sync continuously, sims run discretely; (4) Purpose - sims predict,
    DTs monitor and optimize.",
  "sources": [
    {
      "url": "chapter-8-digital-twins",
      "page_title": "Digital Twins: Concept and Implementation",
      "relevance_score": 0.94
    },
    {
      "url": "chapter-3-simulation",
      "page_title": "Physics-Based Simulation",
      "relevance_score": 0.89
    }
  ],
  "confidence": "high",
  "metadata": {
    "latency_ms": 2489,
    "grounding_score": 0.88
  }
}
```

**Interpretation:**
- ✅ High confidence with strong sources
- ✅ Clear comparative structure
- **Action:** Use as primary comparison reference

---

## Example 6: Low Confidence Query

**User Question:**
> "What are the latest commercial humanoid robot products in 2025?"

**Query Formulation:**
- Intent: Current market information
- Key terms: Commercial, humanoid robots, 2025
- Challenge: Textbook is from 2024, doesn't cover current products

**System Response:**
```json
{
  "answer": "The textbook provides limited information about specific
    commercial humanoid robot products, focusing instead on underlying
    technologies and design principles. It mentions various research
    platforms and discusses humanoid robotics fundamentals, but does not
    cover current commercial offerings or market developments beyond 2024.
    For current market information, please consult recent industry reports
    and company announcements.",
  "sources": [],
  "confidence": "low",
  "metadata": {
    "latency_ms": 1234,
    "grounding_score": 0.35
  }
}
```

**Interpretation:**
- ❌ Low confidence (<0.5)
- ❌ Poor grounding (0.35)
- ❌ No relevant sources found
- **Action:** Acknowledge textbook doesn't cover this, suggest external sources

---

## Query Tips

### ✅ GOOD Queries
- "What is inverse kinematics?" (Specific term, clear intent)
- "Explain how humanoid robots maintain balance" (Context + action)
- "Compare ROS1 and ROS2" (Explicit comparison)
- "What are the components of a robot arm?" (List request, clear scope)

### ❌ POOR Queries
- "Tell me about stuff" (Too vague)
- "Humanoid robots" (Just a topic, not a question)
- "Everything about robotics" (Impossibly broad)
- "What's your opinion on robots?" (Not factual)

---

**Remember:** Always check confidence levels and grounding scores. High scores mean you can rely on the answer. Lower scores mean you should verify with external sources.
