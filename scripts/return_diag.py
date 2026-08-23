import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class ReturnDiag(Node):
    def __init__(self):
        super().__init__(return_diag)
        self.state = UNKNOWN
        self.last_cmd = None
        self.cmd_count = 0
        self.create_subscription(String, /hw/nav/state, self.on_state, 10)
        self.create_subscription(Twist, /hw/cmd_vel, self.on_cmd, 10)
        self.create_timer(5.0, self.report)
        self.start = time.monotonic()
    
    def on_state(self, msg):
        self.state = msg.data
    
    def on_cmd(self, msg):
        self.cmd_count += 1
        self.last_cmd = (msg.linear.x, msg.angular.z)
    
    def report(self):
        elapsed = time.monotonic() - self.start
        lc = self.last_cmd or (0.0, 0.0)
        print(=== ReturnDiag @  + str(round(elapsed,1)) + s ===)
        print(State:  + self.state)
        print(Cmd count:  + str(self.cmd_count))
        print(Last cmd: linear= + str(round(lc[0],3)) +  angular= + str(round(lc[1],3)))

def main():
    rclpy.init()
    node = ReturnDiag()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
