# Physical AI & Humanoid Robotics Textbook
# Module 1, Chapter 4: Command Result Message
# Description: Result feedback from executed AI commands

# Header with timestamp
std_msgs/Header header

# Command reference
string command_id          # ID of the executed command

# Execution status
uint8 status               # Status code (see constants below)
string status_message      # Human-readable status description

# Results
bool success               # Overall success flag
string[] outputs           # String outputs from execution
float64[] measurements     # Numeric measurements/results

# Timing
builtin_interfaces/Duration execution_time  # How long the command took

# Error handling
uint32 error_code          # Error code if failed (0 = no error)
string error_details       # Detailed error description

# Status constants
uint8 STATUS_PENDING = 0
uint8 STATUS_EXECUTING = 1
uint8 STATUS_COMPLETED = 2
uint8 STATUS_FAILED = 3
uint8 STATUS_CANCELLED = 4
uint8 STATUS_TIMEOUT = 5
