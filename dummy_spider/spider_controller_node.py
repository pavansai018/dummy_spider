import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Imu
import math

class SpiderController(Node):
    def __init__(self):
        super().__init__('spider_controller_node')
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.cmd_sub = self.create_subscription(String, '/spider/command', self.command_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # State & IMU variables
        self.current_state = "STOP"
        self.t = 0.0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.is_auto_rotating = False
        
        # Xacro Base Pose
        self.stand_pitch = -0.9
        self.stand_knee = 1.6

        # Startup Message
        print("\n" + "="*40)
        print("SPIDER_CONTROLLER_NODE STARTED")
        print("SYSTEM STATUS: IMU TRACKING READY")
        print("="*40 + "\n")

    def imu_callback(self, msg):
        # Convert Quaternion to Euler Yaw
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def command_callback(self, msg):
        cmd = msg.data
        if cmd.startswith("TURN_"):
            angle_map = {"TURN_0": 0.0, "TURN_90": 1.57, "TURN_-90": -1.57, "TURN_180": 3.1415}
            self.target_yaw = angle_map[cmd]
            self.is_auto_rotating = True
            self.get_logger().info(f"Targeting Orientation: {cmd}")
        else:
            self.is_auto_rotating = False
            self.current_state = cmd

    def control_loop(self):
        # Automatic Navigation (I, J, K, L)
        if self.is_auto_rotating:
            error = self.target_yaw - self.current_yaw
            # Normalize error to [-pi, pi]
            error = (error + math.pi) % (2 * math.pi) - math.pi
            
            if abs(error) < 0.07: # Threshold to stop
                self.get_logger().info("Target Angle Acquired. Stopping.")
                self.current_state = "STOP"
                self.is_auto_rotating = False
            else:
                self.current_state = "ROTATE_CCW" if error > 0 else "ROTATE_CW"

        if self.current_state == "STOP":
            self.publish_static_pose()
        else:
            self.publish_gait()

    def publish_gait(self):
        msg = Float64MultiArray()
        speed, step_h = 5.0, 0.45
        stride_x, stride_y, turn = 0.0, 0.0, 0.0
        
        # Logic: X = Forward/Back, Y = Side Legs lateral movement
        if self.current_state == "FORWARD": stride_x = -0.5
        elif self.current_state == "BACKWARD": stride_x = 0.5
        elif self.current_state == "LEFT": stride_y = 0.5
        elif self.current_state == "RIGHT": stride_y = -0.5
        elif self.current_state == "ROTATE_CW": turn = 0.6
        elif self.current_state == "ROTATE_CCW": turn = -0.6

        leg_phases = {'FL': 0, 'RR': 0, 'FR': math.pi, 'RL': math.pi}
        leg_order = ['FL', 'FR', 'RR', 'RL']
        full_cmd = []

        for leg in leg_order:
            phi = (self.t * speed + leg_phases[leg]) % (2 * math.pi)
            yaw, pitch, knee = 0.0, self.stand_pitch, self.stand_knee

            if stride_x != 0: # W and S (Front/Back)
                move = math.cos(phi) * stride_x
                yaw = move if leg in ['FL', 'RL'] else -move
            
            elif stride_y != 0: # A and D (Side Walk)
                # To move in Y direction, all legs swing in same lateral direction
                yaw = math.cos(phi) * stride_y 
            
            elif turn != 0: # I, J, K, L
                # Spin: All legs move circular
                yaw = -math.cos(phi) * turn

            if phi < math.pi: # Swing Phase (Lift leg)
                lift = math.sin(phi) * step_h
                pitch += lift
                knee -= lift

            full_cmd.extend([yaw, pitch, knee])

        msg.data = full_cmd
        self.cmd_pub.publish(msg)
        self.t += 0.02

    def publish_static_pose(self):
        msg = Float64MultiArray()
        msg.data = [0.0, self.stand_pitch, self.stand_knee] * 4
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = SpiderController()
    rclpy.spin(node)
    rclpy.shutdown()