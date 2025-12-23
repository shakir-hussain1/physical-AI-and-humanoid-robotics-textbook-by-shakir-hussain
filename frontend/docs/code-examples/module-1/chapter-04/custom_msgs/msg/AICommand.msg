# Physical AI & Humanoid Robotics Textbook
# Module 1, Chapter 4: Custom Message Definition
# Description: Message type for AI-generated commands to robot control systems
# Usage: Used by command interpreter to translate natural language to robot actions

# Header with timestamp and frame
std_msgs/Header header

# Command identification
string command_id          # Unique ID for tracking (e.g., "cmd_001")
string command_type        # Type: "motion", "manipulation", "speech", "perception"

# Command parameters
string action              # The action to perform (e.g., "walk", "wave", "pick_up")
string[] parameters        # Action-specific parameters as key=value strings
float64[] numeric_values   # Numeric parameters (positions, velocities, etc.)

# AI context
float32 confidence         # AI confidence score [0.0, 1.0]
string source_utterance    # Original natural language input
string model_id            # ID of AI model that generated this command

# Execution hints
uint8 priority             # Priority level: 0=low, 1=normal, 2=high, 3=critical
bool interruptible         # Can this command be interrupted by higher priority?
float64 timeout            # Maximum execution time in seconds (0 = no limit)

# Constants for command types
uint8 PRIORITY_LOW = 0
uint8 PRIORITY_NORMAL = 1
uint8 PRIORITY_HIGH = 2
uint8 PRIORITY_CRITICAL = 3
