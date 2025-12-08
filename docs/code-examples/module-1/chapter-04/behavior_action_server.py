#!/usr/bin/env python3
"""
Physical AI & Humanoid Robotics Textbook
Module 1, Chapter 4: Behavior Action Server

Description: Action server that executes complex, multi-step robot behaviors
             with progress feedback and cancellation support.

Architecture:
    Command Interpreter -> [ExecuteBehavior Goal] -> Behavior Server -> Robot Controllers

Features:
    - Phase-based behavior execution
    - Real-time progress feedback
    - Graceful cancellation handling
    - Error recovery mechanisms
    - State machine pattern

Requirements: ROS 2 Jazzy, rclpy
Run: ros2 run humanoid_ai behavior_action_server
"""

import time
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, field
from enum import Enum, auto
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# In production, these would be the generated message types
# from humanoid_ai_msgs.action import ExecuteBehavior


# ============================================================
# Behavior State Machine
# ============================================================

class BehaviorState(Enum):
    """States for behavior execution state machine."""
    IDLE = auto()
    INITIALIZING = auto()
    EXECUTING = auto()
    RECOVERING = auto()
    PAUSED = auto()
    FINALIZING = auto()
    COMPLETED = auto()
    FAILED = auto()
    CANCELLED = auto()


@dataclass
class BehaviorPhase:
    """Represents a single phase in a multi-phase behavior."""
    name: str
    description: str
    duration_estimate: float  # seconds
    execute: Callable[[], bool]  # Returns True on success
    can_recover: bool = True
    recovery_action: Optional[Callable[[], bool]] = None


@dataclass
class BehaviorResult:
    """Result of behavior execution."""
    success: bool
    termination_reason: str
    message: str
    execution_time: float
    outcome_data: Dict[str, Any] = field(default_factory=dict)
    error_code: int = 0
    error_details: str = ""


@dataclass
class BehaviorFeedback:
    """Feedback during behavior execution."""
    progress: float  # 0.0 to 1.0
    current_phase: str
    phase_number: int
    total_phases: int
    status_message: str
    execution_state: BehaviorState
    elapsed_time: float
    estimated_remaining: float
    confidence: float


# ============================================================
# Behavior Definitions
# ============================================================

class BehaviorLibrary:
    """
    Library of predefined robot behaviors.

    Each behavior is defined as a sequence of phases that the
    action server executes in order.
    """

    def __init__(self, node: Node):
        self.node = node
        self._behaviors: Dict[str, List[BehaviorPhase]] = {}
        self._register_behaviors()

    def _register_behaviors(self):
        """Register all available behaviors."""

        # ===== Walk Forward Behavior =====
        self._behaviors["walk_forward"] = [
            BehaviorPhase(
                name="prepare",
                description="Preparing for walk",
                duration_estimate=0.5,
                execute=lambda: self._simulate_phase("Shifting weight to stable stance")
            ),
            BehaviorPhase(
                name="execute_gait",
                description="Executing walking gait",
                duration_estimate=2.0,
                execute=lambda: self._simulate_phase("Walking forward")
            ),
            BehaviorPhase(
                name="stabilize",
                description="Stabilizing after walk",
                duration_estimate=0.5,
                execute=lambda: self._simulate_phase("Returning to neutral stance")
            ),
        ]

        # ===== Turn Behavior =====
        self._behaviors["turn"] = [
            BehaviorPhase(
                name="prepare",
                description="Preparing for turn",
                duration_estimate=0.3,
                execute=lambda: self._simulate_phase("Adjusting balance for turn")
            ),
            BehaviorPhase(
                name="rotate",
                description="Executing rotation",
                duration_estimate=1.0,
                execute=lambda: self._simulate_phase("Rotating body")
            ),
            BehaviorPhase(
                name="stabilize",
                description="Stabilizing after turn",
                duration_estimate=0.3,
                execute=lambda: self._simulate_phase("Stabilizing orientation")
            ),
        ]

        # ===== Pick Up Object Behavior =====
        self._behaviors["pick_up"] = [
            BehaviorPhase(
                name="locate",
                description="Locating target object",
                duration_estimate=0.5,
                execute=lambda: self._simulate_phase("Visual search for object")
            ),
            BehaviorPhase(
                name="approach",
                description="Moving arm to approach position",
                duration_estimate=1.0,
                execute=lambda: self._simulate_phase("Trajectory planning and execution")
            ),
            BehaviorPhase(
                name="grasp",
                description="Grasping object",
                duration_estimate=0.8,
                execute=lambda: self._simulate_phase("Closing gripper on object")
            ),
            BehaviorPhase(
                name="lift",
                description="Lifting object",
                duration_estimate=0.7,
                execute=lambda: self._simulate_phase("Lifting with force feedback")
            ),
            BehaviorPhase(
                name="secure",
                description="Securing grasp",
                duration_estimate=0.3,
                execute=lambda: self._simulate_phase("Verifying stable grasp")
            ),
        ]

        # ===== Wave Behavior =====
        self._behaviors["wave"] = [
            BehaviorPhase(
                name="raise_arm",
                description="Raising arm",
                duration_estimate=0.5,
                execute=lambda: self._simulate_phase("Moving arm to wave position")
            ),
            BehaviorPhase(
                name="wave_motion",
                description="Waving motion",
                duration_estimate=1.5,
                execute=lambda: self._simulate_phase("Executing wave pattern")
            ),
            BehaviorPhase(
                name="lower_arm",
                description="Lowering arm",
                duration_estimate=0.5,
                execute=lambda: self._simulate_phase("Returning arm to rest")
            ),
        ]

        # ===== Greet Behavior =====
        self._behaviors["greet"] = [
            BehaviorPhase(
                name="attention",
                description="Getting attention",
                duration_estimate=0.3,
                execute=lambda: self._simulate_phase("Turning to face user")
            ),
            BehaviorPhase(
                name="gesture",
                description="Greeting gesture",
                duration_estimate=0.8,
                execute=lambda: self._simulate_phase("Performing greeting wave")
            ),
            BehaviorPhase(
                name="speak",
                description="Speaking greeting",
                duration_estimate=1.0,
                execute=lambda: self._simulate_phase("Saying 'Hello!'")
            ),
        ]

        # ===== Scan Environment Behavior =====
        self._behaviors["scan_environment"] = [
            BehaviorPhase(
                name="center",
                description="Centering head",
                duration_estimate=0.3,
                execute=lambda: self._simulate_phase("Moving head to center")
            ),
            BehaviorPhase(
                name="scan_left",
                description="Scanning left",
                duration_estimate=1.0,
                execute=lambda: self._simulate_phase("Panning head left")
            ),
            BehaviorPhase(
                name="scan_right",
                description="Scanning right",
                duration_estimate=1.5,
                execute=lambda: self._simulate_phase("Panning head right")
            ),
            BehaviorPhase(
                name="return_center",
                description="Returning to center",
                duration_estimate=0.5,
                execute=lambda: self._simulate_phase("Centering head")
            ),
        ]

        # ===== Stop Behavior (immediate) =====
        self._behaviors["stop"] = [
            BehaviorPhase(
                name="halt",
                description="Halting all motion",
                duration_estimate=0.1,
                execute=lambda: self._simulate_phase("Emergency stop executed")
            ),
        ]

        # ===== Stand Behavior =====
        self._behaviors["stand"] = [
            BehaviorPhase(
                name="prepare",
                description="Preparing to stand",
                duration_estimate=0.5,
                execute=lambda: self._simulate_phase("Positioning for stand")
            ),
            BehaviorPhase(
                name="rise",
                description="Rising",
                duration_estimate=2.0,
                execute=lambda: self._simulate_phase("Extending legs")
            ),
            BehaviorPhase(
                name="balance",
                description="Balancing",
                duration_estimate=0.5,
                execute=lambda: self._simulate_phase("Achieving stable stance")
            ),
        ]

        self.node.get_logger().info(f'Registered {len(self._behaviors)} behaviors')

    def _simulate_phase(self, description: str) -> bool:
        """Simulate a phase execution (for demonstration)."""
        self.node.get_logger().debug(f'Phase: {description}')
        return True  # Always succeed in simulation

    def get_behavior(self, name: str) -> Optional[List[BehaviorPhase]]:
        """Get behavior phases by name."""
        return self._behaviors.get(name)

    def list_behaviors(self) -> List[str]:
        """List all available behavior names."""
        return list(self._behaviors.keys())


# ============================================================
# Behavior Action Server
# ============================================================

class BehaviorActionServer(Node):
    """
    ROS 2 Action Server for executing robot behaviors.

    This server:
    1. Receives behavior goals from clients (e.g., command interpreter)
    2. Executes behaviors as sequences of phases
    3. Provides real-time feedback on progress
    4. Handles cancellation requests gracefully
    5. Reports final results

    Action:
        /execute_behavior (ExecuteBehavior): Main behavior execution interface
    """

    def __init__(self):
        super().__init__('behavior_action_server')

        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()

        # Behavior library
        self.behavior_library = BehaviorLibrary(self)

        # Execution state
        self.state = BehaviorState.IDLE
        self.state_lock = Lock()
        self.current_goal: Optional[ServerGoalHandle] = None
        self.cancel_requested = False

        # ========== Action Server ==========
        # Note: In production, use the actual ExecuteBehavior action type
        # self._action_server = ActionServer(
        #     self,
        #     ExecuteBehavior,
        #     '/execute_behavior',
        #     execute_callback=self.execute_callback,
        #     goal_callback=self.goal_callback,
        #     cancel_callback=self.cancel_callback,
        #     callback_group=self.callback_group
        # )

        # For demonstration, we'll create a simple timer-based simulation
        self._demo_timer = self.create_timer(
            5.0,
            self._demo_behavior_execution
        )

        self.get_logger().info('Behavior Action Server initialized')
        self.get_logger().info(f'Available behaviors: {self.behavior_library.list_behaviors()}')

    def goal_callback(self, goal_request) -> GoalResponse:
        """
        Handle incoming goal requests.

        This callback decides whether to accept or reject new goals.
        """
        behavior_name = goal_request.behavior_name
        self.get_logger().info(f'Received goal request: {behavior_name}')

        # Check if behavior exists
        if self.behavior_library.get_behavior(behavior_name) is None:
            self.get_logger().warn(f'Unknown behavior: {behavior_name}')
            return GoalResponse.REJECT

        # Check if we're already executing
        with self.state_lock:
            if self.state not in [BehaviorState.IDLE, BehaviorState.COMPLETED,
                                   BehaviorState.FAILED, BehaviorState.CANCELLED]:
                self.get_logger().warn('Cannot accept goal: already executing')
                return GoalResponse.REJECT

        self.get_logger().info(f'Goal accepted: {behavior_name}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """
        Handle cancellation requests.

        Always accept cancellation to ensure responsive behavior.
        """
        self.get_logger().info('Received cancel request')
        self.cancel_requested = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Execute the behavior goal.

        This is the main execution loop that:
        1. Initializes the behavior
        2. Executes each phase
        3. Publishes feedback
        4. Handles cancellation
        5. Returns the result
        """
        self.current_goal = goal_handle
        self.cancel_requested = False

        # Extract goal parameters
        behavior_name = goal_handle.request.behavior_name
        behavior_id = goal_handle.request.behavior_id
        timeout = goal_handle.request.timeout_sec

        self.get_logger().info(f'Executing behavior: {behavior_name} (id: {behavior_id})')

        # Get behavior phases
        phases = self.behavior_library.get_behavior(behavior_name)
        if phases is None:
            return self._create_result(
                success=False,
                reason="FAILURE",
                message=f"Unknown behavior: {behavior_name}"
            )

        # Initialize execution
        start_time = time.time()
        total_phases = len(phases)

        with self.state_lock:
            self.state = BehaviorState.INITIALIZING

        # Execute each phase
        for i, phase in enumerate(phases):
            # Check for cancellation
            if self.cancel_requested:
                self.get_logger().info('Behavior cancelled by request')
                goal_handle.canceled()
                with self.state_lock:
                    self.state = BehaviorState.CANCELLED
                return self._create_result(
                    success=False,
                    reason="CANCELLED",
                    message="Behavior cancelled by user request"
                )

            # Check for timeout
            elapsed = time.time() - start_time
            if timeout > 0 and elapsed > timeout:
                self.get_logger().warn(f'Behavior timed out after {elapsed:.1f}s')
                with self.state_lock:
                    self.state = BehaviorState.FAILED
                return self._create_result(
                    success=False,
                    reason="TIMEOUT",
                    message=f"Behavior timed out after {elapsed:.1f}s"
                )

            # Update state
            with self.state_lock:
                self.state = BehaviorState.EXECUTING

            # Publish feedback
            progress = i / total_phases
            estimated_remaining = sum(p.duration_estimate for p in phases[i:])
            feedback = self._create_feedback(
                progress=progress,
                current_phase=phase.name,
                phase_number=i + 1,
                total_phases=total_phases,
                status_message=phase.description,
                state=self.state,
                elapsed=elapsed,
                remaining=estimated_remaining
            )
            goal_handle.publish_feedback(feedback)

            # Execute phase
            self.get_logger().info(f'Phase {i+1}/{total_phases}: {phase.name}')
            try:
                success = phase.execute()
                if not success and phase.can_recover and phase.recovery_action:
                    # Attempt recovery
                    with self.state_lock:
                        self.state = BehaviorState.RECOVERING
                    self.get_logger().info(f'Phase failed, attempting recovery...')
                    success = phase.recovery_action()

                if not success:
                    self.get_logger().error(f'Phase {phase.name} failed')
                    with self.state_lock:
                        self.state = BehaviorState.FAILED
                    return self._create_result(
                        success=False,
                        reason="FAILURE",
                        message=f"Phase '{phase.name}' failed"
                    )

            except Exception as e:
                self.get_logger().error(f'Phase {phase.name} raised exception: {e}')
                with self.state_lock:
                    self.state = BehaviorState.FAILED
                return self._create_result(
                    success=False,
                    reason="FAILURE",
                    message=f"Exception in phase '{phase.name}': {str(e)}",
                    error_code=1
                )

            # Simulate phase duration
            time.sleep(phase.duration_estimate)

        # Finalize
        with self.state_lock:
            self.state = BehaviorState.FINALIZING

        # Publish final feedback
        elapsed = time.time() - start_time
        final_feedback = self._create_feedback(
            progress=1.0,
            current_phase="complete",
            phase_number=total_phases,
            total_phases=total_phases,
            status_message="Behavior completed successfully",
            state=BehaviorState.COMPLETED,
            elapsed=elapsed,
            remaining=0.0
        )
        goal_handle.publish_feedback(final_feedback)

        # Mark goal as succeeded
        goal_handle.succeed()

        with self.state_lock:
            self.state = BehaviorState.COMPLETED

        self.get_logger().info(f'Behavior {behavior_name} completed in {elapsed:.2f}s')

        return self._create_result(
            success=True,
            reason="SUCCESS",
            message=f"Behavior '{behavior_name}' completed successfully",
            execution_time=elapsed
        )

    def _create_feedback(self, progress: float, current_phase: str,
                         phase_number: int, total_phases: int,
                         status_message: str, state: BehaviorState,
                         elapsed: float, remaining: float) -> Any:
        """Create a feedback message."""
        # In production, return actual ExecuteBehavior.Feedback message
        # For demonstration, return a dict
        return {
            'progress': progress,
            'current_phase': current_phase,
            'phase_number': phase_number,
            'total_phases': total_phases,
            'status_message': status_message,
            'execution_state': state.name,
            'elapsed_time_sec': elapsed,
            'estimated_remaining_sec': remaining,
            'confidence': 0.95
        }

    def _create_result(self, success: bool, reason: str, message: str,
                       execution_time: float = 0.0, error_code: int = 0) -> Any:
        """Create a result message."""
        # In production, return actual ExecuteBehavior.Result message
        # For demonstration, return a dict
        return {
            'success': success,
            'termination_reason': reason,
            'message': message,
            'actual_duration_sec': execution_time,
            'error_code': error_code
        }

    def _demo_behavior_execution(self):
        """Demonstrate behavior execution (for testing without action client)."""
        # This timer is just for demonstration/testing
        # In production, behaviors are triggered via the action interface
        pass


# ============================================================
# Main Entry Point
# ============================================================

def main(args=None):
    """Main entry point for the behavior action server."""
    rclpy.init(args=args)

    # Create server
    server = BehaviorActionServer()

    # Use multi-threaded executor for concurrent goal handling
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)

    try:
        server.get_logger().info('Behavior Action Server running. Press Ctrl+C to exit.')
        executor.spin()
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down Behavior Action Server...')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
