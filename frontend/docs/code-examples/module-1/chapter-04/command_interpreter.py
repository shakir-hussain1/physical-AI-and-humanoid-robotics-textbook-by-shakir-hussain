#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 4: Command Interpreter Node

Description: Translates natural language commands from AI systems into
             ROS 2 actions and commands for robot execution.

Architecture:
    AI System -> [Natural Language] -> Command Interpreter -> [ROS 2 Actions] -> Robot

Features:
    - Pattern-based command parsing
    - Command validation and safety checking
    - Action client management for behavior execution
    - Feedback relay to AI system

Requirements: ROS 2 Jazzy, rclpy, humanoid_ai_msgs package
Run: ros2 run humanoid_ai command_interpreter
"""

import re
import uuid
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Note: In a real implementation, these would be generated from the .msg/.srv/.action files
# from humanoid_ai_msgs.msg import AICommand, AICommandResult, RobotState
# from humanoid_ai_msgs.srv import InterpretCommand, ValidateCommand
# from humanoid_ai_msgs.action import ExecuteBehavior


# ============================================================
# Command Pattern Definitions
# ============================================================

class CommandType(Enum):
    """Types of commands the interpreter can handle."""
    MOTION = "motion"
    MANIPULATION = "manipulation"
    SPEECH = "speech"
    PERCEPTION = "perception"
    SYSTEM = "system"
    UNKNOWN = "unknown"


@dataclass
class ParsedCommand:
    """Structured representation of a parsed command."""
    command_id: str
    command_type: CommandType
    action: str
    parameters: Dict[str, Any]
    confidence: float
    original_utterance: str


class CommandPattern:
    """Pattern matching for natural language commands."""

    def __init__(self, pattern: str, command_type: CommandType,
                 action: str, param_extractors: Dict[str, int]):
        """
        Initialize a command pattern.

        Args:
            pattern: Regex pattern to match
            command_type: Type of command this pattern represents
            action: Action name for the robot
            param_extractors: Dict mapping param names to regex group indices
        """
        self.pattern = re.compile(pattern, re.IGNORECASE)
        self.command_type = command_type
        self.action = action
        self.param_extractors = param_extractors

    def match(self, utterance: str) -> Optional[Tuple[Dict[str, str], float]]:
        """
        Try to match the utterance against this pattern.

        Returns:
            Tuple of (parameters dict, confidence) if matched, None otherwise
        """
        match = self.pattern.search(utterance)
        if match:
            params = {}
            for param_name, group_idx in self.param_extractors.items():
                try:
                    value = match.group(group_idx)
                    if value:
                        params[param_name] = value
                except IndexError:
                    pass

            # Confidence based on how much of the utterance was matched
            match_ratio = len(match.group(0)) / len(utterance)
            confidence = min(0.95, 0.5 + match_ratio * 0.5)

            return params, confidence
        return None


# ============================================================
# Command Interpreter Node
# ============================================================

class CommandInterpreter(Node):
    """
    ROS 2 node that interprets natural language commands.

    This node:
    1. Subscribes to natural language input from AI systems
    2. Parses commands using pattern matching
    3. Validates commands for safety and feasibility
    4. Dispatches commands to appropriate action servers
    5. Publishes execution results back to AI systems

    Topics:
        ~/input (String): Natural language commands from AI
        ~/result (String): JSON-formatted execution results
        ~/status (String): Current interpreter status

    Services:
        ~/interpret (InterpretCommand): Synchronous command interpretation
        ~/validate (ValidateCommand): Command validation without execution

    Action Clients:
        /execute_behavior: Sends behaviors to robot control system
    """

    def __init__(self):
        super().__init__('command_interpreter')

        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()

        # Initialize command patterns
        self._init_patterns()

        # Command queue and state
        self.command_queue: List[ParsedCommand] = []
        self.current_command: Optional[ParsedCommand] = None
        self.is_executing = False

        # ========== Publishers ==========
        self.result_pub = self.create_publisher(
            String,
            '~/result',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '~/status',
            10
        )

        # ========== Subscribers ==========
        self.input_sub = self.create_subscription(
            String,
            '~/input',
            self.input_callback,
            10,
            callback_group=self.callback_group
        )

        # ========== Action Clients ==========
        # In production, this would use the actual ExecuteBehavior action
        # self.behavior_client = ActionClient(
        #     self,
        #     ExecuteBehavior,
        #     '/execute_behavior',
        #     callback_group=self.callback_group
        # )

        # ========== Timers ==========
        # Status publishing timer
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_status
        )

        # Command processing timer
        self.process_timer = self.create_timer(
            0.1,  # 10 Hz
            self.process_command_queue
        )

        self.get_logger().info('Command Interpreter initialized')
        self.get_logger().info('Listening for commands on ~/input')

    def _init_patterns(self):
        """Initialize command patterns for natural language parsing."""

        self.patterns: List[CommandPattern] = [
            # ===== Motion Commands =====
            CommandPattern(
                r"(?:walk|move|go)\s+(?:forward|ahead)\s*(\d+\.?\d*)?\s*(meters?|m|steps?)?",
                CommandType.MOTION,
                "walk_forward",
                {"distance": 1, "unit": 2}
            ),
            CommandPattern(
                r"(?:walk|move|go)\s+(?:backward|back)\s*(\d+\.?\d*)?\s*(meters?|m|steps?)?",
                CommandType.MOTION,
                "walk_backward",
                {"distance": 1, "unit": 2}
            ),
            CommandPattern(
                r"(?:turn|rotate)\s+(left|right)\s*(\d+\.?\d*)?\s*(degrees?|deg|°)?",
                CommandType.MOTION,
                "turn",
                {"direction": 1, "angle": 2, "unit": 3}
            ),
            CommandPattern(
                r"(?:stop|halt|freeze)",
                CommandType.MOTION,
                "stop",
                {}
            ),
            CommandPattern(
                r"(?:stand|stand\s+up|get\s+up)",
                CommandType.MOTION,
                "stand",
                {}
            ),
            CommandPattern(
                r"(?:sit|sit\s+down)",
                CommandType.MOTION,
                "sit",
                {}
            ),

            # ===== Manipulation Commands =====
            CommandPattern(
                r"(?:pick\s+up|grab|grasp)\s+(?:the\s+)?(\w+)",
                CommandType.MANIPULATION,
                "pick_up",
                {"object": 1}
            ),
            CommandPattern(
                r"(?:put\s+down|place|release)\s+(?:the\s+)?(\w+)?",
                CommandType.MANIPULATION,
                "place",
                {"object": 1}
            ),
            CommandPattern(
                r"(?:wave|wave\s+hand)",
                CommandType.MANIPULATION,
                "wave",
                {}
            ),
            CommandPattern(
                r"(?:point|point\s+at|point\s+to)\s+(?:the\s+)?(\w+)",
                CommandType.MANIPULATION,
                "point",
                {"target": 1}
            ),

            # ===== Speech Commands =====
            CommandPattern(
                r"(?:say|speak|tell)\s+[\"']?(.+?)[\"']?$",
                CommandType.SPEECH,
                "speak",
                {"text": 1}
            ),
            CommandPattern(
                r"(?:greet|say\s+hello|hello)",
                CommandType.SPEECH,
                "greet",
                {}
            ),

            # ===== Perception Commands =====
            CommandPattern(
                r"(?:look|look\s+at|observe)\s+(?:the\s+)?(\w+)",
                CommandType.PERCEPTION,
                "look_at",
                {"target": 1}
            ),
            CommandPattern(
                r"(?:find|search\s+for|locate)\s+(?:the\s+)?(\w+)",
                CommandType.PERCEPTION,
                "find_object",
                {"object": 1}
            ),
            CommandPattern(
                r"(?:scan|look\s+around|survey)",
                CommandType.PERCEPTION,
                "scan_environment",
                {}
            ),

            # ===== System Commands =====
            CommandPattern(
                r"(?:status|report\s+status|how\s+are\s+you)",
                CommandType.SYSTEM,
                "report_status",
                {}
            ),
            CommandPattern(
                r"(?:battery|check\s+battery|power\s+level)",
                CommandType.SYSTEM,
                "check_battery",
                {}
            ),
            CommandPattern(
                r"(?:emergency\s+stop|e-?stop)",
                CommandType.SYSTEM,
                "emergency_stop",
                {}
            ),
        ]

        self.get_logger().info(f'Loaded {len(self.patterns)} command patterns')

    def parse_command(self, utterance: str) -> ParsedCommand:
        """
        Parse a natural language utterance into a structured command.

        Args:
            utterance: Natural language input string

        Returns:
            ParsedCommand with extracted information
        """
        utterance = utterance.strip()

        best_match = None
        best_confidence = 0.0
        best_pattern = None

        # Try all patterns and find the best match
        for pattern in self.patterns:
            result = pattern.match(utterance)
            if result:
                params, confidence = result
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_match = params
                    best_pattern = pattern

        if best_pattern and best_confidence > 0.3:
            return ParsedCommand(
                command_id=f"cmd_{uuid.uuid4().hex[:8]}",
                command_type=best_pattern.command_type,
                action=best_pattern.action,
                parameters=best_match or {},
                confidence=best_confidence,
                original_utterance=utterance
            )

        # No pattern matched - return unknown command
        return ParsedCommand(
            command_id=f"cmd_{uuid.uuid4().hex[:8]}",
            command_type=CommandType.UNKNOWN,
            action="unknown",
            parameters={},
            confidence=0.0,
            original_utterance=utterance
        )

    def validate_command(self, command: ParsedCommand) -> Tuple[bool, List[str]]:
        """
        Validate a parsed command for safety and feasibility.

        Args:
            command: The parsed command to validate

        Returns:
            Tuple of (is_valid, list of error messages)
        """
        errors = []

        # Check if command type is known
        if command.command_type == CommandType.UNKNOWN:
            errors.append(f"Unknown command: '{command.original_utterance}'")
            return False, errors

        # Check confidence threshold
        if command.confidence < 0.5:
            errors.append(f"Low confidence ({command.confidence:.2f}). Please rephrase.")
            return False, errors

        # Validate motion commands
        if command.command_type == CommandType.MOTION:
            if "distance" in command.parameters:
                try:
                    distance = float(command.parameters["distance"])
                    if distance > 10.0:
                        errors.append(f"Distance {distance}m exceeds safe limit of 10m")
                    if distance < 0:
                        errors.append("Distance cannot be negative")
                except ValueError:
                    errors.append(f"Invalid distance value: {command.parameters['distance']}")

            if "angle" in command.parameters:
                try:
                    angle = float(command.parameters["angle"])
                    if abs(angle) > 180:
                        errors.append(f"Angle {angle}° exceeds safe limit of 180°")
                except ValueError:
                    errors.append(f"Invalid angle value: {command.parameters['angle']}")

        # Validate manipulation commands
        if command.command_type == CommandType.MANIPULATION:
            if command.action == "pick_up" and "object" not in command.parameters:
                errors.append("Pick up command requires an object target")

        return len(errors) == 0, errors

    def input_callback(self, msg: String):
        """Handle incoming natural language commands."""
        utterance = msg.data
        self.get_logger().info(f'Received command: "{utterance}"')

        # Parse the command
        command = self.parse_command(utterance)
        self.get_logger().info(
            f'Parsed: action={command.action}, '
            f'type={command.command_type.value}, '
            f'confidence={command.confidence:.2f}'
        )

        # Validate the command
        is_valid, errors = self.validate_command(command)

        if is_valid:
            # Add to queue for execution
            self.command_queue.append(command)
            self.get_logger().info(f'Command {command.command_id} queued for execution')
        else:
            # Report validation failure
            error_msg = "; ".join(errors)
            self.get_logger().warn(f'Command validation failed: {error_msg}')
            self.publish_result(command.command_id, False, error_msg)

    def process_command_queue(self):
        """Process commands from the queue."""
        if self.is_executing or not self.command_queue:
            return

        # Get next command
        command = self.command_queue.pop(0)
        self.current_command = command
        self.is_executing = True

        self.get_logger().info(f'Executing command {command.command_id}: {command.action}')

        # In production, this would send to the action server
        # For this example, we simulate execution
        self.simulate_execution(command)

    def simulate_execution(self, command: ParsedCommand):
        """
        Simulate command execution (for demonstration).

        In production, this would:
        1. Send goal to ExecuteBehavior action server
        2. Monitor feedback
        3. Handle completion/failure
        """
        import time

        # Simulate different execution times based on command type
        execution_times = {
            CommandType.MOTION: 2.0,
            CommandType.MANIPULATION: 1.5,
            CommandType.SPEECH: 0.5,
            CommandType.PERCEPTION: 1.0,
            CommandType.SYSTEM: 0.2,
        }

        exec_time = execution_times.get(command.command_type, 1.0)

        # Log execution details
        params_str = ", ".join(f"{k}={v}" for k, v in command.parameters.items())
        self.get_logger().info(
            f'Executing {command.action}({params_str}) - '
            f'estimated time: {exec_time}s'
        )

        # In real implementation, we'd wait for action feedback
        # Here we just mark as complete after a delay (using timer)
        self.create_timer(
            exec_time,
            lambda: self.complete_execution(command, True, "Execution completed"),
            callback_group=self.callback_group
        )

    def complete_execution(self, command: ParsedCommand, success: bool, message: str):
        """Handle command execution completion."""
        self.get_logger().info(
            f'Command {command.command_id} completed: '
            f'success={success}, message="{message}"'
        )

        self.publish_result(command.command_id, success, message)

        self.current_command = None
        self.is_executing = False

    def publish_result(self, command_id: str, success: bool, message: str):
        """Publish execution result."""
        # Format as simple JSON-like string
        result_str = (
            f'{{"command_id": "{command_id}", '
            f'"success": {str(success).lower()}, '
            f'"message": "{message}"}}'
        )

        result_msg = String()
        result_msg.data = result_str
        self.result_pub.publish(result_msg)

    def publish_status(self):
        """Publish interpreter status."""
        status = {
            "is_executing": self.is_executing,
            "queue_length": len(self.command_queue),
            "current_command": self.current_command.command_id if self.current_command else None
        }

        status_msg = String()
        status_msg.data = str(status)
        self.status_pub.publish(status_msg)


# ============================================================
# Main Entry Point
# ============================================================

def main(args=None):
    """Main entry point for the command interpreter node."""
    rclpy.init(args=args)

    # Create node
    interpreter = CommandInterpreter()

    # Use multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(interpreter)

    try:
        interpreter.get_logger().info('Command Interpreter running. Press Ctrl+C to exit.')
        executor.spin()
    except KeyboardInterrupt:
        interpreter.get_logger().info('Shutting down Command Interpreter...')
    finally:
        interpreter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
