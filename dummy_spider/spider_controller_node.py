import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Imu
import math
import signal
import sys

class SpiderController(Node):
    def __init__(self):
        super().__init__('spider_controller')
        self.get_logger().info("Spider Brain: Activated")

        # Publishers and subscribers
        self.pub_ = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.sub_cmd_ = self.create_subscription(String, '/spider/command', self.command_callback, 10)
        self.sub_imu_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Timer
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.update_gait)

        # State
        self.current_command = "STOP"
        self.yaw_current = 0.0
        self.yaw_target = None
        self.spin_active = False
        self.spin_done_ticks = 0  
        self.force_home_ticks = 15 

        # Stance parameters
        self.stand_pitch = -0.9
        self.stand_knee = 1.6
        self.step_height = 0.5
        self.stride_length = 0.4
        self.turn_angle = 0.5
        
        # Soft Start
        self.target_speed = 4.0
        self.current_speed = 0.0
        self.acceleration = 0.15 
        self.t = 0.0

        signal.signal(signal.SIGINT, self.shutdown_gracefully)

    def shutdown_gracefully(self, signum, frame):
        self.send_home_stance(force=True)
        rclpy.shutdown()
        sys.exit(0)

    def send_home_stance(self, force=False):
        msg = Float64MultiArray()
        for _ in ['FL','FR','RR','RL']:
            msg.data.extend([0.0, self.stand_pitch, self.stand_knee])
        self.pub_.publish(msg)
        if force:
            self.t = 0.0
            self.current_speed = 0.0

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw_current = math.atan2(siny_cosp, cosy_cosp)

    def command_callback(self, msg: String):
        cmd = msg.data
        if cmd != self.current_command:
            if cmd not in ["STOP", "HOME"]:
                self.t = 0.0
                self.current_speed = 0.0
        self.current_command = cmd

        if cmd in ["TURN_0", "TURN_90", "TURN_180", "TURN_-90"]:
            targets = {"TURN_0": 0.0, "TURN_90": -math.pi/2, "TURN_180": math.pi, "TURN_-90": math.pi/2}
            self.yaw_target = targets[cmd]
            self.spin_active = True
            self.spin_done_ticks = 0
        else:
            self.spin_active = False
            self.yaw_target = None
            if cmd in ["STOP","HOME"]:
                self.send_home_stance(force=True)

    def spin_to_yaw(self):
        yaw_diff = self.yaw_target - self.yaw_current
        yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))

        if abs(yaw_diff) < 0.06:
            self.spin_done_ticks += 1
            self.send_home_stance()
            if self.spin_done_ticks >= self.force_home_ticks:
                self.spin_active = False
                self.current_command = "STOP"
                self.t = 0.0
                self.current_speed = 0.0
            return

        if self.current_speed < self.target_speed:
            self.current_speed += self.acceleration

        direction = 1.0 if yaw_diff > 0 else -1.0
        leg_phases = {
            'FL': self.t * self.current_speed,
            'RR': self.t * self.current_speed,
            'FR': self.t * self.current_speed + math.pi,
            'RL': self.t * self.current_speed + math.pi
        }
        
        full_command = []
        for leg in ['FL','FR','RR','RL']:
            phase = leg_phases[leg] % (2*math.pi)
            yaw, pitch, knee = 0.0, self.stand_pitch, self.stand_knee
            if phase < math.pi:
                lift = math.sin(phase) * 0.45
                yaw = math.cos(phase) * self.turn_angle * direction 
                pitch += lift
                knee -= lift
            else:
                yaw = math.cos(phase) * self.turn_angle * direction 
            full_command.extend([yaw, pitch, knee])

        self.pub_.publish(Float64MultiArray(data=full_command))
        self.t += self.timer_period

    def update_gait(self):
        if self.spin_active:
            self.spin_to_yaw()
            return

        if self.current_command in ["STOP","HOME"]:
            self.send_home_stance()
            return

        if self.current_speed < self.target_speed:
            self.current_speed += self.acceleration

        if self.current_command in ["FORWARD","BACKWARD"]:
            direction = -1.0 if self.current_command == "FORWARD" else 1.0
            leg_phases = {
                'FL': self.t * self.current_speed,
                'RR': self.t * self.current_speed,
                'FR': self.t * self.current_speed + math.pi,
                'RL': self.t * self.current_speed + math.pi
            }
            leg_order = ['FL','FR','RR','RL']

        elif self.current_command in ["LEFT","RIGHT"]:
            direction = -1.0 if self.current_command == "LEFT" else 1.0
            leg_phases = {
                'FR': self.t * self.current_speed,
                'RL': self.t * self.current_speed,
                'RR': self.t * self.current_speed + math.pi,
                'FL': self.t * self.current_speed + math.pi
            }
            leg_order = ['FR','RR','RL','FL'] 

        full_command = []
        for leg in leg_order:
            phase = leg_phases[leg] % (2*math.pi)
            yaw, pitch, knee = 0.0, self.stand_pitch, self.stand_knee
            if phase < math.pi:
                lift = math.sin(phase) * self.step_height
                yaw = math.cos(phase) * self.stride_length * direction
                pitch += lift
                knee -= lift
            else:
                yaw = math.cos(phase) * self.stride_length * direction
            
            if leg in ['FR','RR']:
                yaw = -yaw
            full_command.extend([yaw, pitch, knee])

        self.pub_.publish(Float64MultiArray(data=full_command))
        self.t += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = SpiderController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()