# DefectReport - Detected defect information
# Published on /g1/inspection/defects

# Defect type constants
uint8 TYPE_LOCATION_ERROR = 0    # Item in wrong position vs plan
uint8 TYPE_QUALITY_ISSUE = 1     # Scratches, damage, defects on finishes

# Message fields
std_msgs/Header header
string defect_id
uint8 defect_type              # Use TYPE_* constants
string description
geometry_msgs/Point location   # World coordinates
geometry_msgs/Point plan_location  # Plan coordinates
float32 confidence             # 0.0 to 1.0
string image_path
string annotated_image_path
