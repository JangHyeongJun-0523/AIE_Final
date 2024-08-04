import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sqrt, atan2, pi

# positions (x, y, yaw)
positions = {
    "o": (0, 0, 0),
    "1": (1.5546208449304282, -0.4596189342427643, -1.437230999624169),
    "2": (1.0402942456593676, -0.40828154538591077, -1.6386375833115416),
    "3": (0.4565105916649657, -0.3396688475363866, -1.5449646256794836),
    "4": (1.5976331803669466, 0.5327394950611359, -1.615602490631877),
    "5": (1.0402942456593676, 0.5327394950611359, -1.615602490631877),
    "6": (0.4565105916649657, 0.5327394950611359, -1.5449646256794836),
}

key = input("Enter the key to get the (x, y, yaw) values: ")

# print (x, y, yaw)
if key in positions:
    x, y, yaw = positions[key]
    print(f"Key {key}: x = {x}, y = {y}, yaw = {yaw}")
else:
    print(f"Key {key} is not in the dictionary")

class PID:
    def __init__(self, P, I, D, max_state, min_state, dt):
        self.P = P
        self.I = I
        self.D = D
        self.max_state = max_state
        self.min_state = min_state
        self.pre_state = 0.0
        self.dt = dt
        self.integrated_state = 0.0
        self.pre_time = rclpy.clock.Clock().now().to_msg().sec

    def process(self, state):
        current_time = rclpy.clock.Clock().now().to_msg().sec
        dt = current_time - self.pre_time
        if dt == 0.:
            state_D = 0.
        else:
            state_D = (state - self.pre_state) / dt
        state_I = self.integrated_state + state * dt
        out = self.P * state + self.D * state_D + self.I * state_I
        if out > self.max_state:
            out = self.max_state
        elif out < self.min_state:
            out = self.min_state
        self.pre_state = state
        self.integrated_state = state_I
        self.pre_time = current_time
        return out
    
class MoveToGoalNode(Node):
    def __init__(self):
        super().__init__('move_to_goal_node')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.goal_x = 0.4565105916649657  # 목표 위치의 x 좌표
        self.goal_y = -0.3396688475363866  # 목표 위치의 y 좌표
        self.goal_theta = -1.5449646256794836  # 목표 위치의 theta 좌표
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.pid_linear = PID(P=1.0, I=0.0, D=0.0, max_state=1.0, min_state=-1.0, dt=0.1)
        self.pid_angular = PID(P=2.0, I=0.0, D=0.0, max_state=1.0, min_state=-1.0, dt=0.1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1초마다 호출

    def amcl_pose_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        _, _, self.position['theta'] = self.get_euler_from_quaternion(msg.pose.pose.orientation)
        print(self.position['x'], self.position['y'], self.position['theta'])

    def timer_callback(self):
        # 목표 위치와 현재 위치 간의 거리 및 각도 계산
        dx = self.goal_x - self.position['x']
        dy = self.goal_y - self.position['y']
        distance = sqrt(dx**2 + dy**2)
        target_angle = atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.position['theta'])
        goal_angle_error = self.normalize_angle(self.goal_theta - self.position['theta'])

        if distance > 0.1:
            linear_speed = self.pid_linear.process(distance)
            angular_speed = self.pid_angular.process(angle_error)
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.cmd_vel_publisher.publish(twist)
        elif abs(goal_angle_error) > 0.05:
            angular_speed = self.pid_angular.process(goal_angle_error)
            twist = Twist()
            twist.angular.z = angular_speed
            self.cmd_vel_publisher.publish(twist)
        else:
            # 목표 지점에 도달하면 이동 중지
            self.stop_robot()

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle
    
    def get_euler_from_quaternion(self, quaternion):
        import tf_transformations
        euler = tf_transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler
    
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
