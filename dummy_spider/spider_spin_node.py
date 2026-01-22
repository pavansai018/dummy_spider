import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SpiderRotate(Node):
    def __init__(self):
        super().__init__('spider_rotate_node')
        
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/position_controller/commands', 
            10)
        
        self.timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(self.timer_period, self.rotate_loop)
        
        self.t = 0.0
        # Values from your Xacro to maintain height
        self.stand_pitch = -0.9
        self.stand_knee = 1.6

    def rotate_loop(self):
        msg = Float64MultiArray()
        
        # --- ROTATION PARAMETERS ---
        speed = 4.0           # Speed of the leg cycle
        step_height = 0.45    # Height to lift feet to clear the ground
        turn_angle = 0.5      # How far the hip yaw rotates (in radians)
        
        # We group legs into diagonal pairs
        # Phase 0 to PI: Swing (Lift & Reset)
        # Phase PI to 2PI: Stance (Push the body)
        leg_phases = {
            'FL': (self.t * speed),
            'RR': (self.t * speed),
            'FR': (self.t * speed) + math.pi,
            'RL': (self.t * speed) + math.pi
        }

        # Order must match your YAML: FL, FR, RR, RL
        leg_order = ['FL', 'FR', 'RR', 'RL']
        full_command = []

        for leg in leg_order:
            phase = leg_phases[leg] % (2 * math.pi)
            
            # Default Stance
            yaw = 0.0
            pitch = self.stand_pitch
            knee = self.stand_knee

            if phase < math.pi:
                # SWING PHASE: Lift leg and move hip to "start" position
                lift = math.sin(phase) * step_height
                yaw = -math.cos(phase) * turn_angle
                
                pitch += lift  # Lift the leg up
                knee -= lift   # Flex knee to keep foot from hitting the ground
            else:
                # STANCE PHASE: Foot is on ground, rotating the body
                # The yaw moves in the opposite direction of the swing
                yaw = -math.cos(phase) * turn_angle
                # Pitch and Knee stay at standing defaults

            full_command.extend([yaw, pitch, knee])

        msg.data = full_command
        self.publisher_.publish(msg)
        self.t += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = SpiderRotate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()