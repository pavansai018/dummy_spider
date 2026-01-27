import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class HomePositionNode(Node):
    def __init__(self):
        super().__init__('home_position_node')
        self.pub_ = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.timer = self.create_timer(0.5, self.publish_home)

        # Spider stance
        self.stand_pitch = -0.9
        self.stand_knee = 1.6

    def publish_home(self):
        msg = Float64MultiArray()
        leg_order = ['FL','FR','RR','RL']
        full_command = []
        for _ in leg_order:
            full_command.extend([0.0, self.stand_pitch, self.stand_knee])
        msg.data = full_command
        self.pub_.publish(msg)
        self.get_logger().info("[HomePosition] Published home stance")

def main(args=None):
    rclpy.init(args=args)
    node = HomePositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
