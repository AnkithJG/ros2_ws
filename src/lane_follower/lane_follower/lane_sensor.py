import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

# Simulates a "camera" node publishing how far the car is from the lane center
class LaneSensor(Node):
    def __init__(self):
        super().__init__('lane_sensor')  # Name of the node

        # Publisher to send lane offset (positive = too far right, negative = too far left)
        self.publisher = self.create_publisher(Float32, 'lane_offset', 10)

        # Call publish_offset() every 0.1 seconds
        self.timer = self.create_timer(0.1, self.publish_offset)

        self.time = 0.0  # Used to simulate oscillating drift

    def publish_offset(self):
        # Fake offset as a sine wave: simulates drifting left/right between -0.5 and +0.5
        offset = 0.5 * math.sin(self.time)

        msg = Float32()
        msg.data = float(offset)

        self.publisher.publish(msg)  # Send message to /lane_offset
        self.time += 0.1  # Increment time to keep sine wave moving

def main(args=None):
    rclpy.init(args=args)        
    node = LaneSensor()           
    rclpy.spin(node)              
    node.destroy_node()           
    rclpy.shutdown()              
    
if __name__ == '__main__':
    main()
