std_msgs/Header header              # Standard ROS message header, includes timestamp and frame_id
uint8[] servo_numbers                  # Servo number (e.g., 0 for left main, 1 for right pitch, etc.)
float32[] target_angles                # Target angle for the servo
float32[] durations                    # Duration to reach target angle in seconds
string[] easing_algorithms             # Easing algorithm to use (e.g., 'linear', 'cubic')
float32[] easing_in_factors            # Easing-in factor (if applicable)
float32[] easing_out_factors           # Easing-out factor (if applicable)
string movement_type                # Type of movement (e.g., 'thrust_unit', 'yaw_adjustment', 'glide_position')
builtin_interfaces/Time deadline    # Deadline or expiration time for this command
string operational_mode             # Operational mode (e.g., 'energy_efficient', 'emergency_correction', 'active_maneuvering')
uint8 priority                      # Priority level for this command (e.g., 0 for normal, 1 for high)
