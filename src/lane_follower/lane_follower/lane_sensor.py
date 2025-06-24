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
# This code defines a ROS2 node that simulates a lane sensor by publishing a sinusoidal offset value.
# The offset value simulates the drifting of the vehicle from the center of the lane.
# The node publishes this value to the 'lane_offset' topic at a rate of 10 Hz.
# The offset is calculated using a sine function, which varies over time to simulate the vehicle's movement.
# The node can be run in a ROS2 environment, and it will continuously publish the offset value until it is shut down.
# This can be useful for testing lane-following algorithms or for simulating sensor data in a controlled environment.
# The offset value is published as a Float32 message, which is a standard message type in ROS2 for representing floating-point numbers.
# The `publish_offset` method is called every 0.1 seconds, updating the offset based on the current time.
# The `main` function initializes the ROS2 node and starts spinning it to process callbacks.
# The node can be extended or modified to include more complex behavior or additional sensors as needed.
# The `LaneSensor` class inherits from `Node`, which is the base class for all      ROS2 nodes.
# The `create_publisher` method is used to create a publisher for the 'lane_offset' topic, allowing other nodes to subscribe to it and receive the lane offset data.
# The `