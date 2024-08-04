import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sqrt, atan2, pi

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
        self.goal_theta = -1.5449646256794836  # 목표 위치의 theta 좌표
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.pid_angular = PID(P=2.0, I=0.0, D=0.0, max_state=1.0, min_state=-1.0, dt=0.1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1초마다 호출

    def amcl_pose_callback(self, msg):
        _, _, self.position['theta'] = self.get_euler_from_quaternion(msg.pose.pose.orientation)
        print(self.position['theta'])

    def timer_callback(self):
        # 목표 위치와 현재 위치 간의 거리 및 각도 계산
        target_angle = self.goal_theta
        angle_error = self.normalize_angle(target_angle - self.position['theta'])
        if abs(angle_error) > 0.05:
            angular_speed = self.pid_angular.process(angle_error)
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