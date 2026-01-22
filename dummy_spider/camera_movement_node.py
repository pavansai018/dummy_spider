import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SpiderSpin(Node):
    def __init__(self):
        super().__init__('spider_spin_node')
        
        # Publisher for position_controller
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/position_controller/commands', 
            10)
        
        self.timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(self.timer_period, self.spin_loop)
        
        self.t = 0.0
        
        # Static standing posture from Xacro initial values
        self.stand_pitch = -0.9
        self.stand_knee = 1.6

    def spin_loop(self):
        msg = Float64MultiArray()
        
        # --- SPIN PARAMETERS ---
        # Increase amplitude for a wider spin, increase speed for faster rotation
        amplitude = 0.5  
        speed = 2.0      
        
        # Calculate the yaw offset using a sine wave for a smooth back-and-forth spin
        # This keeps feet planted and rotates the torso.
        yaw_val = math.sin(self.t * speed) * amplitude
        
        # The command array must follow YAML order:
        # 1. FL (yaw, pitch, knee)
        # 2. FR (yaw, pitch, knee)
        # 3. RR (yaw, pitch, knee)
        # 4. RL (yaw, pitch, knee)
        
        commands = [
            yaw_val, self.stand_pitch, self.stand_knee, # FL
            yaw_val, self.stand_pitch, self.stand_knee, # FR
            yaw_val, self.stand_pitch, self.stand_knee, # RR
            yaw_val, self.stand_pitch, self.stand_knee  # RL
        ]

        msg.data = commands
        self.publisher_.publish(msg)
        self.t += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = SpiderSpin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()