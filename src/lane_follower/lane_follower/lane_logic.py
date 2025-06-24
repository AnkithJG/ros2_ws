import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Float32, 'lane_offset', self.offset_callback, 10)
        self.timer = self.create_timer(0.1, self.follow_lane)

        self.offset = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        # PID constants
        self.Kp = 2.0
        self.Ki = 0.1
        self.Kd = 0.5

    def offset_callback(self, msg):
        self.offset = msg.data

    def follow_lane(self):
        error = self.offset
        self.integral += error * 0.1  
        derivative = (error - self.prev_error) / 0.1
        self.prev_error = error

        control = -(self.Kp * error + self.Ki * self.integral + self.Kd * derivative)

        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = control
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
