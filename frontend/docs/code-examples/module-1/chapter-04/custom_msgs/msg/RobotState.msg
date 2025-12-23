# Physical AI & Humanoid Robotics Textbook
# Module 1, Chapter 4: Robot State Message
# Description: Comprehensive robot state for AI context awareness

# Header with timestamp
std_msgs/Header header

# Overall state
uint8 operational_mode     # Current mode (see constants)
bool is_ready              # Ready to accept commands
bool has_error             # Error flag

# Position and orientation
geometry_msgs/Pose base_pose           # Robot base pose in world frame
geometry_msgs/Twist base_velocity      # Current velocity

# Joint information (summary)
uint32 num_joints          # Total number of joints
float64[] joint_positions  # Current joint positions (rad)
float64[] joint_velocities # Current joint velocities (rad/s)

# Battery and power
float32 battery_percentage # Battery level [0-100]
float32 power_consumption  # Current power draw (watts)

# Sensor status (bitfield)
uint32 sensor_status       # Bitmask of active sensors

# Current task info
string current_task_id     # ID of currently executing task
float32 task_progress      # Progress [0.0, 1.0]

# Safety
bool emergency_stop_active # E-stop engaged
uint8 safety_level         # Current safety level

# Mode constants
uint8 MODE_IDLE = 0
uint8 MODE_AUTONOMOUS = 1
uint8 MODE_TELEOPERATED = 2
uint8 MODE_LEARNING = 3
uint8 MODE_MAINTENANCE = 4

# Safety level constants
uint8 SAFETY_NORMAL = 0
uint8 SAFETY_REDUCED = 1
uint8 SAFETY_MINIMAL = 2
