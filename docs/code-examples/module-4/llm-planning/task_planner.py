#!/usr/bin/env python3
"""
LLM-Based Task Planner for Humanoid Robot
Module 4: VLA Systems - Chapter 15

Uses large language models to decompose natural language
instructions into executable robot action sequences.
"""

import json
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any
from enum import Enum
from abc import ABC, abstractmethod


class ActionType(Enum):
    """Available robot action types."""
    NAVIGATE = "navigate"
    GRASP = "grasp"
    PLACE = "place"
    LOOK_AT = "look_at"
    POINT = "point"
    SAY = "say"
    WAIT = "wait"


@dataclass
class RobotAction:
    """Single executable robot action."""
    action_type: ActionType
    parameters: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return {
            "action": self.action_type.value,
            "params": self.parameters
        }

    @classmethod
    def from_dict(cls, data: Dict) -> "RobotAction":
        return cls(
            action_type=ActionType(data["action"]),
            parameters=data.get("params", {})
        )


@dataclass
class TaskPlan:
    """Complete task plan."""
    actions: List[RobotAction]
    description: str = ""
    confidence: float = 1.0
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "actions": [a.to_dict() for a in self.actions],
            "description": self.description,
            "confidence": self.confidence,
            "warnings": self.warnings
        }


# Robot capability definitions for LLM grounding
ROBOT_CAPABILITIES = """
Available robot actions:
1. navigate(location: str) - Move robot to a location
   Locations: kitchen, living_room, bedroom, bathroom, office, hallway

2. grasp(object_name: str, grasp_type: str = "power") - Pick up an object
   Grasp types: power (default), precision, gentle
   Precondition: Object must be visible and reachable, hand must be empty

3. place(surface: str, position: str = "center") - Place held object
   Positions: left, center, right
   Precondition: Must be holding an object

4. look_at(target: str) - Turn head to look at target

5. point(target: str) - Point at target with arm

6. say(message: str) - Speak a message to the user

7. wait(seconds: float) - Wait for specified duration
"""

SAFETY_RULES = """
Safety constraints:
- Never throw objects
- Never make sudden movements near humans
- Always confirm before handling fragile items
- Maximum reach is 0.8 meters
- Cannot handle objects heavier than 2 kg
- Must navigate to objects that are not reachable
"""


class TaskPlanner(ABC):
    """Abstract base class for task planners."""

    @abstractmethod
    def plan(
        self,
        request: str,
        robot_state: Dict,
        scene: Dict
    ) -> TaskPlan:
        """Generate task plan from natural language request."""
        pass


class LLMTaskPlanner(TaskPlanner):
    """Task planner using OpenAI GPT models."""

    SYSTEM_PROMPT = f"""You are a robot task planner. Convert natural language
instructions into sequences of robot actions.

{ROBOT_CAPABILITIES}

{SAFETY_RULES}

Current robot state will be provided with each request.
Visible objects in the scene will be listed.

Output format: JSON object with:
{{
    "description": "Brief description of the plan",
    "actions": [
        {{"action": "action_name", "params": {{"param1": "value1"}}}},
        ...
    ],
    "warnings": ["any safety concerns or limitations"]
}}

Rules:
1. Only use actions from the capabilities list
2. Check that objects exist in the scene before referencing them
3. Navigate to unreachable objects before grasping
4. Always end with a say() action to confirm completion
5. If task is impossible, return empty actions with explanation in description
"""

    def __init__(self, api_key: str, model: str = "gpt-4o"):
        from openai import OpenAI
        self.client = OpenAI(api_key=api_key)
        self.model = model

    def plan(
        self,
        request: str,
        robot_state: Dict,
        scene: Dict
    ) -> TaskPlan:
        """Generate plan using LLM."""
        user_message = self._build_user_message(request, robot_state, scene)

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.SYSTEM_PROMPT},
                {"role": "user", "content": user_message}
            ],
            response_format={"type": "json_object"},
            temperature=0.1  # Low temperature for deterministic output
        )

        result = json.loads(response.choices[0].message.content)
        return self._parse_plan(result)

    def _build_user_message(
        self,
        request: str,
        robot_state: Dict,
        scene: Dict
    ) -> str:
        """Build user message with context."""
        state_str = f"""Robot State:
- Location: {robot_state.get('location', 'unknown')}
- Holding: {robot_state.get('holding', 'nothing')}
- Battery: {robot_state.get('battery', 100)}%"""

        objects = scene.get('objects', [])
        if objects:
            objects_str = "\n".join(
                f"- {obj['name']}: at {obj.get('location', 'unknown')}, "
                f"{'reachable' if obj.get('reachable', False) else 'not reachable'}"
                for obj in objects
            )
        else:
            objects_str = "No objects visible"

        return f"""Request: {request}

{state_str}

Visible Objects:
{objects_str}

Generate a plan to accomplish this request."""

    def _parse_plan(self, result: Dict) -> TaskPlan:
        """Parse LLM response into TaskPlan."""
        actions = []
        for action_dict in result.get("actions", []):
            try:
                action = RobotAction.from_dict(action_dict)
                actions.append(action)
            except (ValueError, KeyError) as e:
                print(f"Warning: Could not parse action: {action_dict}, error: {e}")

        return TaskPlan(
            actions=actions,
            description=result.get("description", ""),
            warnings=result.get("warnings", [])
        )


class ToolBasedPlanner(TaskPlanner):
    """Task planner using LLM function/tool calling."""

    TOOLS = [
        {
            "type": "function",
            "function": {
                "name": "navigate",
                "description": "Move the robot to a location",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {
                            "type": "string",
                            "description": "Target location",
                            "enum": ["kitchen", "living_room", "bedroom", "bathroom", "office"]
                        }
                    },
                    "required": ["location"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "grasp",
                "description": "Pick up an object. Object must be visible and reachable.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "object_name": {"type": "string", "description": "Name of object"},
                        "grasp_type": {
                            "type": "string",
                            "enum": ["power", "precision", "gentle"],
                            "description": "Type of grasp"
                        }
                    },
                    "required": ["object_name"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "place",
                "description": "Place held object on a surface",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "surface": {"type": "string", "description": "Where to place"},
                        "position": {
                            "type": "string",
                            "enum": ["left", "center", "right"]
                        }
                    },
                    "required": ["surface"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "look_at",
                "description": "Turn head to look at something",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "target": {"type": "string", "description": "What to look at"}
                    },
                    "required": ["target"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "say",
                "description": "Speak a message",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "message": {"type": "string", "description": "What to say"}
                    },
                    "required": ["message"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "wait",
                "description": "Wait for a duration",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "seconds": {"type": "number", "description": "Seconds to wait"}
                    },
                    "required": ["seconds"]
                }
            }
        }
    ]

    def __init__(self, api_key: str, model: str = "gpt-4o"):
        from openai import OpenAI
        self.client = OpenAI(api_key=api_key)
        self.model = model

    def plan(
        self,
        request: str,
        robot_state: Dict,
        scene: Dict
    ) -> TaskPlan:
        """Generate plan using tool calls."""
        system_prompt = f"""You are a robot planner. Use the available tools to
create a plan for the user's request.

Robot state: {json.dumps(robot_state)}
Scene: {json.dumps(scene)}

Call tools in sequence to accomplish the task. Always end with say() to confirm."""

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": request}
        ]

        actions = []
        max_iterations = 20

        for _ in range(max_iterations):
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                tools=self.TOOLS,
                tool_choice="auto"
            )

            message = response.choices[0].message

            if message.tool_calls:
                for tool_call in message.tool_calls:
                    action = RobotAction(
                        action_type=ActionType(tool_call.function.name),
                        parameters=json.loads(tool_call.function.arguments)
                    )
                    actions.append(action)

                    # Add to conversation
                    messages.append(message)
                    messages.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": f"Action {action.action_type.value} queued"
                    })

            if response.choices[0].finish_reason == "stop":
                break

        return TaskPlan(
            actions=actions,
            description=f"Plan for: {request}"
        )


class SafetyValidator:
    """Validate plans for safety."""

    KNOWN_ACTIONS = {a.value for a in ActionType}

    FORBIDDEN_PATTERNS = [
        "throw", "hit", "push_person", "run", "jump"
    ]

    DANGEROUS_OBJECTS = [
        "knife", "scissors", "glass", "hot", "electrical", "sharp"
    ]

    def validate(self, plan: TaskPlan, scene: Dict) -> Dict:
        """Validate plan for safety and feasibility."""
        issues = []
        warnings = []

        visible_objects = {obj["name"] for obj in scene.get("objects", [])}

        for i, action in enumerate(plan.actions):
            # Check action is known
            if action.action_type.value not in self.KNOWN_ACTIONS:
                issues.append(f"Action {i}: Unknown action type '{action.action_type}'")

            # Check for forbidden patterns
            action_str = json.dumps(action.to_dict()).lower()
            for pattern in self.FORBIDDEN_PATTERNS:
                if pattern in action_str:
                    issues.append(f"Action {i}: Contains forbidden pattern '{pattern}'")

            # Check object exists for grasp
            if action.action_type == ActionType.GRASP:
                obj_name = action.parameters.get("object_name", "")
                if obj_name and obj_name not in visible_objects:
                    warnings.append(f"Action {i}: Object '{obj_name}' not in visible objects")

                # Check for dangerous objects
                for dangerous in self.DANGEROUS_OBJECTS:
                    if dangerous in obj_name.lower():
                        warnings.append(f"Action {i}: Handling potentially dangerous object '{obj_name}'")

        return {
            "valid": len(issues) == 0,
            "issues": issues,
            "warnings": warnings
        }


class RobotTaskPlanner:
    """Complete task planning system with safety validation."""

    def __init__(self, api_key: str, use_tools: bool = True):
        if use_tools:
            self.planner = ToolBasedPlanner(api_key)
        else:
            self.planner = LLMTaskPlanner(api_key)
        self.validator = SafetyValidator()

    def plan_task(
        self,
        request: str,
        robot_state: Dict,
        scene: Dict
    ) -> Dict:
        """Plan and validate task."""
        # Generate plan
        plan = self.planner.plan(request, robot_state, scene)

        # Validate
        validation = self.validator.validate(plan, scene)

        # Combine warnings
        all_warnings = plan.warnings + validation["warnings"]

        return {
            "success": validation["valid"],
            "plan": plan.to_dict() if validation["valid"] else None,
            "issues": validation["issues"],
            "warnings": all_warnings,
            "description": plan.description
        }


# Example usage and testing
if __name__ == "__main__":
    print("LLM Task Planner Demo")
    print("=" * 40)

    # Mock data for testing without API
    mock_robot_state = {
        "location": "living_room",
        "holding": None,
        "battery": 85
    }

    mock_scene = {
        "objects": [
            {"name": "red_ball", "location": "table", "reachable": True},
            {"name": "blue_cube", "location": "shelf", "reachable": False},
            {"name": "cup", "location": "counter", "reachable": True}
        ]
    }

    # Example plan (simulated output)
    example_plan = TaskPlan(
        actions=[
            RobotAction(ActionType.LOOK_AT, {"target": "red_ball"}),
            RobotAction(ActionType.GRASP, {"object_name": "red_ball", "grasp_type": "power"}),
            RobotAction(ActionType.NAVIGATE, {"location": "kitchen"}),
            RobotAction(ActionType.PLACE, {"surface": "counter", "position": "center"}),
            RobotAction(ActionType.SAY, {"message": "I've placed the ball on the counter"})
        ],
        description="Pick up red ball and move to kitchen"
    )

    print("\nExample Task Plan:")
    print(f"Request: 'Take the red ball to the kitchen'")
    print(f"\nGenerated Plan:")
    for i, action in enumerate(example_plan.actions):
        print(f"  {i+1}. {action.action_type.value}({action.parameters})")

    # Validate
    validator = SafetyValidator()
    validation = validator.validate(example_plan, mock_scene)
    print(f"\nValidation: {'PASSED' if validation['valid'] else 'FAILED'}")
    if validation["warnings"]:
        print(f"Warnings: {validation['warnings']}")
    if validation["issues"]:
        print(f"Issues: {validation['issues']}")

    print("\n" + "=" * 40)
    print("To use with real LLM, provide OpenAI API key:")
    print("  planner = RobotTaskPlanner(api_key='your-key')")
    print("  result = planner.plan_task('Pick up the cup', robot_state, scene)")
