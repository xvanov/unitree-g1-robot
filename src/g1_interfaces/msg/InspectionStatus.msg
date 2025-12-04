# InspectionStatus - Current state of the inspection robot
# Published on /g1/inspection/status

# State constants
uint8 STATE_IDLE = 0
uint8 STATE_CALIBRATING = 1
uint8 STATE_INSPECTING = 2
uint8 STATE_PAUSED = 3
uint8 STATE_BLOCKED = 4
uint8 STATE_WAITING_OPERATOR = 5
uint8 STATE_COMPLETE = 6
uint8 STATE_EMERGENCY_STOP = 7
uint8 STATE_RETURNING_HOME = 8

# Message fields
std_msgs/Header header
uint8 state                    # Current state (use STATE_* constants)
uint32 current_waypoint
uint32 total_waypoints
float32 completion_percentage
float32 battery_percentage
geometry_msgs/Pose current_pose
string status_message
