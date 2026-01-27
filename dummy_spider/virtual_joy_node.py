import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys, select, termios, tty

class VirtualJoy(Node):
    def __init__(self):
        super().__init__('virtual_joy')
        self.publisher_ = self.create_publisher(String, '/spider/command', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        print("""
Spider Remote Control Active
---------------------------
W: Forward    S: Backward
A: Left       D: Right
I: Spawn (0°) K: 180°
J: -90°       L: 90°
H: Home
Space: Stop   Ctrl+C: Quit
---------------------------
""")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, sys.stdin.fileno(), self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                cmd = String()
                
                if key == 'w': cmd.data = "FORWARD"
                elif key == 's': cmd.data = "BACKWARD"
                elif key == 'a': cmd.data = "LEFT"
                elif key == 'd': cmd.data = "RIGHT"
                elif key == 'i': cmd.data = "TURN_0"
                elif key == 'k': cmd.data = "TURN_180"
                elif key == 'j': cmd.data = "TURN_-90"
                elif key == 'l': cmd.data = "TURN_90"
                elif key == 'h': cmd.data = "HOME"
                elif key == ' ': cmd.data = "STOP"
                elif key == '\x03': break
                
                if cmd.data:
                    print(f"[VirtualJoy] Key pressed: {key.upper()} -> Command: {cmd.data}")
                    self.publisher_.publish(cmd)
        except Exception as e:
            print(e)

def main():
    rclpy.init()
    node = VirtualJoy()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()