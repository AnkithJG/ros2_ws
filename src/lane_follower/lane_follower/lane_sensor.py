import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class LaneSensor(Node):
    def __init__(self):
        super().__init__('lane_sensor')
        self.publisher = self.create_publisher(Float32, 'lane_offset', 10)
        self.timer = self.create_timer(0.1, self.publish_offset)
        self.time = 0.0

    def publish_offset(self):
        offset = 0.5 * math.sin(self.time)  # simulate drifting
        msg = Float32()
        msg.data = float(offset)
        self.publisher.publish(msg)
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = LaneSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
