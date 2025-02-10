import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pi, hypot


class TurtlebotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        fast_timer_period = 0.1
        self.timer = self.create_timer(fast_timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.topic_callback,
            10)
        
        self.turtlebot_state = Pose() 
        self.declaration()

    def declaration(self):
        self.l = [2.1, 0.003, 0]
        self.a = [7, 0.01, 0]
        self.v = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_theta = 0

    def timer_callback(self):
        cmd = Twist()
        zero = 5.544445
        prev_error_distance = 2
        prev_error_theta = 0
        dt = 0.1

        if (self.v == 0):
            self.goal_x = zero + 2
            self.goal_y = zero
            self.goal_theta = 0
        elif (self.v == 1):
            self.goal_x = zero + 2
            self.goal_y = zero
            self.goal_theta = pi/2
        elif (self.v == 2):
            self.goal_x = zero + 2
            self.goal_y = zero + 2
            self.goal_theta = pi/2
        elif (self.v == 3):
            self.goal_x = zero + 2
            self.goal_y = zero + 2
            self.goal_theta = pi
        elif (self.v == 4):
            self.goal_x = zero
            self.goal_y = zero + 2
            self.goal_theta = pi
        elif (self.v == 5):
            self.goal_x = zero 
            self.goal_y = zero + 2
            if (self.turtlebot_state.theta > 0):
                self.goal_theta = 3 * pi/2
            else:
                self.goal_theta = - pi/2
        elif (self.v == 6):
            self.goal_x = zero 
            self.goal_y = zero
            self.goal_theta = - pi/2
        elif (self.v == 7):
            self.goal_x = zero 
            self.goal_y = zero
            self.goal_theta = 0
        
        error_distance = hypot((self.goal_x - self.turtlebot_state.x), (self.goal_y - self.turtlebot_state.y)) 
        error_theta = self.goal_theta - self.turtlebot_state.theta
        i_error_distance = prev_error_distance + error_distance * dt
        i_error_theta = prev_error_theta + error_theta * dt
       
        p_control_linear = self.l[0] * error_distance
        p_control_angular = self.a[0] * error_theta

        d_control_linear = self.l[1] * (prev_error_distance - error_distance) / dt
        d_control_angular = self.a[1] * (prev_error_theta - error_theta) / dt

        i_control_linear = self.l[2] * i_error_distance * dt
        i_control_angular = self.a[2] * i_error_theta * dt

        if (self.v % 2 == 0):
            cmd.linear.x = p_control_linear + d_control_linear + i_control_linear
            cmd.angular.z = 0.0

        elif (self.v % 2 == 1):
            cmd.linear.x = 0.0
            cmd.angular.z = p_control_angular + d_control_angular + i_control_angular

        self.publisher_.publish(cmd)

        if ((abs(error_theta) < 0.001) and (abs(error_distance) < 0.01)):
            self.v = (self.v + 1) % 8
            i_error_distance = 0
            i_error_theta = 0

        prev_error_distance = error_distance
        prev_error_theta = error_theta

    def topic_callback(self, msg:Pose):
        self.turtlebot_state.x = msg.x
        self.turtlebot_state.y = msg.y
        self.turtlebot_state.theta = msg.theta


def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()