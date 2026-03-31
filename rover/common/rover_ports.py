"""
Central definition of UDP ports and message topics for the rover stack.
All processes should import ports from here.
"""

# ---------------------------------------------------------------------
# Localhost address used by all modules
# ---------------------------------------------------------------------

HOST = "127.0.0.1"

# ---------------------------------------------------------------------
# SENSOR -> ARBITER
# ---------------------------------------------------------------------

VISION_PORT = 5005
LIDAR_PORT  = 5006
TELEOP_PORT = 5007

# ---------------------------------------------------------------------
# ARBITER -> MOTOR
# ---------------------------------------------------------------------

DRIVE_PORT = 5001 

# ---------------------------------------------------------------------
# EVENTS -> UI / JANUS
# ---------------------------------------------------------------------

EVENT_PORT = 5002

# ---------------------------------------------------------------------
# Optional debug / logging
# ---------------------------------------------------------------------

CONTROLLER_PORT = 5010

ODOM_PORT = 5019

DEBUG_PORT = 5030


PORT_NAMES = {
    VISION_PORT: "vision",
    LIDAR_PORT: "lidar",
    TELEOP_PORT: "teleop",
    DRIVE_PORT: "drive_cmd",
    EVENT_PORT: "events",
    DEBUG_PORT: "debug",
}
