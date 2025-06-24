import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# Node that subscribes to lane offset and publishes velocity commands using a PID controller
class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower') 

        # Publisher to send velocity command
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to listen to lane offset values from lane_sensor node
        self.subscription = self.create_subscription(
            Float32,
            'lane_offset',
            self.offset_callback,
            10
        )

        # Timer to repeatedly call the control loop every 0.1 seconds
        self.timer = self.create_timer(0.1, self.follow_lane)

        self.offset = 0.0          # Latest lane offset value (from sensor)
        self.prev_error = 0.0      # Previous error (for derivative)
        self.integral = 0.0        # Accumulated error (for integral)

        self.Kp = 2.0              # Proportional gain
        self.Ki = 0.1              # Integral gain
        self.Kd = 0.5              # Derivative gain

    def offset_callback(self, msg):
        # Save the latest lane offset received
        self.offset = msg.data

    def follow_lane(self):
        error = self.offset                     # Error = how far off center we are
        self.integral += error * 0.1            # Accumulate error (dt = 0.1s)
        derivative = (error - self.prev_error) / 0.1  # Rate of change of error
        self.prev_error = error                 # Save current error for next round

        # PID formula: control = - (Kp*e + Ki*∫e + Kd*de/dt)
        control = -(self.Kp * error + self.Ki * self.integral + self.Kd * derivative)

        # Prepare a Twist message with forward velocity + steering correction
        msg = Twist()
        msg.linear.x = 1.0                      # Constant forward speed
        msg.angular.z = control                 # Turn rate from PID controller
        self.cmd_pub.publish(msg)               # Send command to /cmd_vel

def main(args=None):
    rclpy.init(args=args)         
    node = LaneFollower()         
    rclpy.spin(node)              
    rclpy.shutdown()              


if __name__ == '__main__':
    main()
