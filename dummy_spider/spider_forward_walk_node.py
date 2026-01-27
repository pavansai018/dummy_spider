import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SpiderForwardWalk(Node):
    def __init__(self):
        super().__init__('spider_forward_walk')
        
        # Publisher to match position_controller
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/position_controller/commands', 
            10)
        
        self.timer_period = 0.02  # 50Hz for smooth movement
        self.timer = self.create_timer(self.timer_period, self.walk_loop)
        
        self.t = 0.0
        # Starting pose from Xacro
        self.stand_pitch = -0.9
        self.stand_knee = 1.6

    def walk_loop(self):
        msg = Float64MultiArray()
        
        # --- GAIT PARAMETERS ---
        speed = 4.0           # Cycle speed
        step_height = 0.5     # Vertical lift
        stride_length = 0.4   # Horizontal step distance
        
        # Diagonal pairings: Pair 1 (FL, RR) and Pair 2 (FR, RL)
        leg_phases = {
            'FL': (self.t * speed),
            'RR': (self.t * speed),
            'FR': (self.t * speed) + math.pi,
            'RL': (self.t * speed) + math.pi
        }

        # Order must match YAML: FL, FR, RR, RL
        leg_order = ['FL', 'FR', 'RR', 'RL']
        full_command = []

        for leg in leg_order:
            phase = leg_phases[leg] % (2 * math.pi)
            
            # Default values
            yaw = 0.0
            pitch = self.stand_pitch
            knee = self.stand_knee

            # The gait logic: 
            # Swing (0 to PI): Leg is in the air moving forward
            # Stance (PI to 2PI): Leg is on the ground pushing backward
            
            # Flip the sign of stride_length to change direction from back to front
            direction_multiplier = -1.0 

            if phase < math.pi:
                # SWING PHASE
                lift = math.sin(phase) * step_height
                # Calculate yaw to move leg forward in the air
                yaw = math.cos(phase) * stride_length * direction_multiplier
                pitch += lift
                knee -= lift
            else:
                # STANCE PHASE
                # Calculate yaw to push the ground backward
                yaw = math.cos(phase) * stride_length * direction_multiplier

            # Correction for Right side vs Left side coordinate frames
            if leg in ['FR', 'RR']:
                yaw = -yaw

            full_command.extend([yaw, pitch, knee])

        msg.data = full_command
        self.publisher_.publish(msg)
        self.t += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    node = SpiderForwardWalk()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()