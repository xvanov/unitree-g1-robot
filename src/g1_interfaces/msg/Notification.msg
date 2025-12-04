# Notification - Operator notification message
# Published on /g1/notifications

# Notification type constants
uint8 TYPE_ROUTE_BLOCKED = 0
uint8 TYPE_LOCALIZATION_FAILED = 1
uint8 TYPE_COMPLETE = 2
uint8 TYPE_LOW_BATTERY = 3
uint8 TYPE_CALIBRATION_REQUIRED = 4
uint8 TYPE_DEFECT_FOUND = 5

# Severity constants
uint8 SEVERITY_INFO = 0
uint8 SEVERITY_WARNING = 1
uint8 SEVERITY_ERROR = 2
uint8 SEVERITY_CRITICAL = 3

# Message fields
std_msgs/Header header
uint8 notification_type        # Use TYPE_* constants
uint8 severity                 # Use SEVERITY_* constants
string message
