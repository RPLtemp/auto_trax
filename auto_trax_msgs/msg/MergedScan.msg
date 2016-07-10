# MergedScan.msg

Header header                        # Header for detected Objects
geometry_msgs/Point coordinate       # Center of Mass point from scan
sensor_msgs/LaserScan merged_scan    # Merged Laser Scan

float32 valid_angle_min              # Minimum valid angle
float32 valid_angle_max              # Maximum valid angle

float32 valid_range_min              # Minimum valid range
float32 valid_range_max              # Maximum valid range
