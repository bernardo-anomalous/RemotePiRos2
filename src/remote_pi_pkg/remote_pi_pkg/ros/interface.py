import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Float32MultiArray
from sensor_msgs.msg import Imu
from auv_custom_interfaces.msg import ServoMovementCommand
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from geometry_msgs.msg import Vector3  
from std_msgs.msg import String   
from lifecycle_msgs.srv import GetState     



class ROSInterface(Node):
    def __init__(self):
        super().__init__('gui_ros_interface')
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.mode = "OPERATION"
        self.canned_duration_factor = 1.0
        self.DURATION_STEP = 0.2  # Adjustable increment

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
        self.create_subscription(Bool, 'wing_pid_active', self.wing_pid_status_callback, 10)

        
        # Lifecycle service client
        self.lifecycle_client = self.create_client(ChangeState, 'servo_driver_node/change_state')
        
        
        self.create_subscription(String, 'servo_driver_status', self.servo_status_callback, 10)

        if not self.lifecycle_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("LIFECYCLE SERVICE NOT AVAILABLE. CONTINUING WITHOUT IT.")

        # Subscribers
        self.create_subscription(Float32MultiArray, 'current_servo_angles', self.servo_angles_callback, 10)
        self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.create_subscription(String, 'imu/heading', self.heading_callback, 10)
        self.create_subscription(Vector3, 'imu/euler', self.euler_callback, 10)
        self.roll_pid_enabled = True                     # Tracks current Roll PID state
        self.pid_reattach_pending = False                # Flags whether we are waiting to re-enable PID

        # IMU Health Status Subscriber
        self.imu_health_status = "UNKNOWN"  # Default state until data arrives
        self.create_subscription(String, 'imu/health_status', self.imu_health_callback, 10)
        self.lifecycle_future = None
        self.lifecycle_future_transition_id = None
        self.create_timer(0.1, self.check_lifecycle_future)  # Check every 100ms
        self.get_state_future = None
        self.current_lifecycle_state = None
        self.create_timer(0.1, self.check_get_state_future)  # Check every 100 ms
        self.lifecycle_state_poll_timer = self.create_timer(1.0, self.poll_lifecycle_state)  # Every 1 second



        
    def imu_health_callback(self, msg):
        self.imu_health_status = msg.data
        self.get_logger().info(f"[ROSInterface] IMU Health Status: {self.imu_health_status}")



    def publish_roll(self):
        msg = Float32()
        msg.data = float(self.target_roll)
        self.target_roll_pub.publish(msg)
        # Removed logging for performance

    def publish_pitch(self):
        msg = Float32()
        msg.data = float(self.target_pitch)
        self.target_pitch_pub.publish(msg)
        # Removed logging for performance


    def publish_canned(self):
        # Step 1: Deactivate Roll PID immediately before sending the canned message
        #self.get_logger().info("[ROSInterface] Deactivating Roll PID before sending canned movement.")
        self.set_pid(False)
        self.roll_pid_enabled = False
        self.pid_reattach_pending = True  # Tell the system to re-enable later

        # Step 2: Build the canned command (your existing command logic preserved)
        base_durations = [0.2, 2.0, 0.1, 0.1, 2.5, 0.01, 0.1, 2.0, 2.0]
        adjusted_durations = [d * self.canned_duration_factor for d in base_durations]
        canned_commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [60.0, 50.0, 60.0, 110.0,
                            0.0, 50.0, 120.0, 110.0,
                            0.0, 90.0, 120.0, 90.0,
                            0.0, 130.0, 120.0, 30.0,
                            120.0, 130.0, 0.0, 30.0,
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

        # Step 3: Publish the canned command
        self.canned_pub.publish(msg)
        self.last_command = f"CANNED MOVEMENT PUBLISHED @ {self.get_clock().now().to_msg()}"
        #self.get_logger().info("[ROSInterface] Canned movement command sent.")


    def set_pid(self, activate: bool):
        msg = Bool()
        msg.data = activate
        self.pid_pub.publish(msg)
        state = "ACTIVATED" if activate else "DEACTIVATED"
        #self.get_logger().info(f"{state} ROLL PID.")

    def servo_angles_callback(self, msg: Float32MultiArray):
        self.current_servo_angles = list(msg.data)
        #self.get_logger().info(f"SERVO ANGLES: {self.current_servo_angles}")

    def imu_callback(self, msg: Imu):
        self.imu_reading = msg
        #self.get_logger().info("RECEIVED IMU DATA.")

    def heading_callback(self, msg: String):
        try:
            heading_str = msg.data
            # Parse heading like 'Heading: East, 123.45 degrees'
            heading_value = float(heading_str.split(',')[1].strip().split()[0])
            self.heading = heading_value
            self.get_logger().info(f"HEADING RECEIVED: {self.heading} degrees")
        except Exception as e:
            self.get_logger().error(f"Failed to parse heading string: {e}")

    def euler_callback(self, msg: Vector3):
        try:
            # Directly access Vector3 fields
            self.euler = [-float(msg.x), -float(msg.y), float(msg.z)]
            #self.get_logger().info(f"EULER ANGLES RECEIVED: {self.euler}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse euler data: {e}")
            

    def get_lifecycle_state(self):
        if not self.lifecycle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("SERVO DRIVER NODE NOT AVAILABLE")
            return None

        try:
            from lifecycle_msgs.srv import GetState
            self.get_state_client = self.create_client(GetState, 'servo_driver_node/get_state')

            if not self.get_state_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("GET_STATE SERVICE NOT AVAILABLE")
                return None

            request = GetState.Request()
            self.get_state_future = self.get_state_client.call_async(request)

        except Exception as e:
            self.get_logger().error(f"Error sending get_lifecycle_state request: {e}")


    def check_get_state_future(self):
        if self.get_state_future is not None and self.get_state_future.done():
            try:
                result = self.get_state_future.result()
                if result and result.current_state:
                    state_label = result.current_state.label.lower()
                    if state_label != self.current_lifecycle_state:
                        self.current_lifecycle_state = state_label
                        self.get_logger().info(f"[ROSInterface] Lifecycle state updated: {state_label}")
                else:
                    self.get_logger().warn("GET_STATE response empty or failed.")
            except Exception as e:
                self.get_logger().error(f"Exception in get_state future result: {e}")
            finally:
                self.get_state_future = None  # Always clear the future

    def call_lifecycle_service(self, transition_id):
        if not self.lifecycle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("LIFECYCLE SERVICE NOT AVAILABLE. COMMAND ABORTED.")
            return

        try:
            request = ChangeState.Request()
            request.transition.id = transition_id
            self.lifecycle_future = self.lifecycle_client.call_async(request)
            self.lifecycle_future_transition_id = transition_id
        except Exception as e:
            self.get_logger().error(f"Error during lifecycle service call: {e}")
    
    def servo_status_callback(self, msg):
        status_word = msg.data.split(":")[0].strip().lower()

        if status_word == "busy":
            if self.roll_pid_enabled:
                self.get_logger().info("[ROSInterface] Servo driver busy — deactivating Roll PID.")
                self.set_pid(False)
                self.roll_pid_enabled = False

        elif status_word == "nominal":
            if self.pid_reattach_pending:
                self.get_logger().info("[ROSInterface] Servo driver nominal — reactivating Roll PID (post-canned).")
                self.set_pid(True)
                self.roll_pid_enabled = True
                self.pid_reattach_pending = False


    def wing_pid_status_callback(self, msg):
        self.roll_pid_enabled = msg.data
        # Optionally, emit a signal to the GUI if needed
        
    def check_lifecycle_future(self):
        if self.lifecycle_future is not None and self.lifecycle_future.done():
            try:
                result = self.lifecycle_future.result()
                if result and result.success:
                    self.get_logger().info(f"LIFECYCLE COMMAND SUCCESSFUL: {self.lifecycle_future_transition_id}")
                else:
                    self.get_logger().warn("LIFECYCLE COMMAND FAILED OR NO RESPONSE.")
            except Exception as e:
                self.get_logger().error(f"Exception in lifecycle future result: {e}")
            finally:
                self.lifecycle_future = None
                self.lifecycle_future_transition_id = None


    def poll_lifecycle_state(self):
        # Only issue a request if there is NO pending future and NO previous call in progress
        if self.get_state_future is not None:
            # Skip polling because previous request hasn't completed
            return
        self.get_lifecycle_state()
