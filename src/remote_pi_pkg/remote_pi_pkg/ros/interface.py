import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Float32MultiArray
from sensor_msgs.msg import Imu
from auv_custom_interfaces.msg import ServoMovementCommand
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class ROSInterface(Node):
    def __init__(self):
        super().__init__('gui_ros_interface')
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.mode = "OPERATION"
        self.canned_duration_factor = 1.0
        self.DURATION_STEP = 0.05  # Adjustable increment

        # Sensor readouts
        self.current_servo_angles = []  # List of 6 servo angles
        self.imu_reading = None         # sensor_msgs/Imu
        self.heading = None             # Float32 heading (degrees)
        self.euler = None               # Euler angles: [roll, pitch, yaw]

        # Last command sent (for status display)
        self.last_command = "NONE"

        # Publishers
        self.target_roll_pub = self.create_publisher(Float32, 'target_roll', 10)
        self.target_pitch_pub = self.create_publisher(Float32, 'target_pitch', 10)
        self.canned_pub = self.create_publisher(ServoMovementCommand, 'servo_interpolation_commands', 10)
        self.pid_pub = self.create_publisher(Bool, 'wing_pid_active', 10)
        
        # Lifecycle service client
        self.lifecycle_client = self.create_client(ChangeState, 'servo_driver_node/change_state')
        if not self.lifecycle_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("LIFECYCLE SERVICE NOT AVAILABLE. CONTINUING WITHOUT IT.")

        # Subscribers
        self.create_subscription(Float32MultiArray, 'current_servo_angles', self.servo_angles_callback, 10)
        self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.create_subscription(Float32, 'imu/heading', self.heading_callback, 10)
        self.create_subscription(Float32MultiArray, 'imu/euler', self.euler_callback, 10)

    def publish_roll(self):
        msg = Float32()
        msg.data = float(self.target_roll)
        self.target_roll_pub.publish(msg)
        self.get_logger().info(f"PUBLISHED ROLL: {self.target_roll}")

    def publish_pitch(self):
        msg = Float32()
        msg.data = float(self.target_pitch)
        self.target_pitch_pub.publish(msg)
        self.get_logger().info(f"PUBLISHED PITCH: {self.target_pitch}")

    def publish_canned(self):
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted_durations = [d * self.canned_duration_factor for d in base_durations]
        canned_commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [60.0, 50.0, 60.0, 110.0,
                              0.0, 50.0, 120.0, 110.0,
                              0.0, 90.0, 120.0, 90.0,
                              0.0, 130.0, 120.0, 30.0,
                              120.0, 110.0, 0.0, 50.0,
                              120.0, 90.0, 0.0, 90.0,
                              120.0, 50.0, 0.0, 110.0,
                              60.0, 50.0, 60.0, 110.0,
                              60.0, 90.0, 60.0, 90.0],
            'durations': adjusted_durations,
            'easing_algorithms': ['EXPONENTIAL', 'CUBIC', 'CUBIC', 'EXPONENTIAL', 
                                  'CUBIC', 'CUBIC', 'EXPONENTIAL', 'CUBIC', 'EXPONENTIAL'],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'THRUST_UNIT',
            'deadline': (self.get_clock().now() + rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'ENERGY_EFFICIENT',
            'priority': 0
        }
        msg = ServoMovementCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.servo_numbers = canned_commands['servo_numbers']
        msg.target_angles = canned_commands['target_angles']
        msg.durations = canned_commands['durations']
        msg.easing_algorithms = canned_commands['easing_algorithms']
        msg.easing_in_factors = canned_commands['easing_in_factors']
        msg.easing_out_factors = canned_commands['easing_out_factors']
        msg.movement_type = canned_commands['movement_type']
        msg.deadline = canned_commands['deadline']
        msg.operational_mode = canned_commands['operational_mode']
        msg.priority = canned_commands['priority']
        
        self.canned_pub.publish(msg)
        self.last_command = f"CANNED MOVEMENT PUBLISHED @ {self.get_clock().now().to_msg()}"
        self.get_logger().info("PUBLISHED CANNED MOVEMENT COMMAND.")

    def set_pid(self, activate: bool):
        msg = Bool()
        msg.data = activate
        self.pid_pub.publish(msg)
        state = "ACTIVATED" if activate else "DEACTIVATED"
        self.get_logger().info(f"{state} ROLL PID.")

    def servo_angles_callback(self, msg: Float32MultiArray):
        self.current_servo_angles = list(msg.data)
        self.get_logger().info(f"SERVO ANGLES: {self.current_servo_angles}")

    def imu_callback(self, msg: Imu):
        self.imu_reading = msg
        self.get_logger().info("RECEIVED IMU DATA.")

    def heading_callback(self, msg: Float32):
        self.heading = msg.data
        self.get_logger().info(f"HEADING: {self.heading}")

    def euler_callback(self, msg: Float32MultiArray):
        try:
            # Ensure Euler values are floats
            self.euler = [float(x) for x in msg.data]
        except Exception:
            self.euler = None
        self.get_logger().info(f"EULER ANGLES: {self.euler}")