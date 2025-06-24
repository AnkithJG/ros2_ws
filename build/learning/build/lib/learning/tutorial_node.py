import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleSquareMover(Node):
    def __init__(self):
        super().__init__('turtle_square_mover')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move)
        self.stage = 0
        self.count = 0

    def move(self):
        msg = Twist()
        if self.stage == 0:  # Move forward
            msg.linear.x = 2.0
            msg.angular.z = 0.0
        elif self.stage == 1:  # Turn right
            msg.linear.x = 0.0
            msg.angular.z = -1.57
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher.publish(msg)
        self.count += 1

        # After some cycles, switch stage
        if self.count > 10:
            self.count = 0
            self.stage = (self.stage + 1) % 2

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSquareMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
