import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_msgs.msg import GoalPose
from robot_msgs.msg import ObjectPose
from robot_msgs.msg import RobotStatus
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
        self.goal_subscriber = self.create_subscription(GoalPose, '/set_goal', self.set_goal, 10)
        self.object_subscriber = self.create_subscription(ObjectPose, '/detected_object', self.object_detection, 10)
        self.current_status_publisher = self.create_publisher(RobotStatus, '/robot_status',10)

        self.robot_id = 'Mk.2'
        self.status = 'wait'

        self.goal_x = 0.0  # 목표 위치의 x 좌표
        self.goal_y = 0.0  # 목표 위치의 y 좌표
        self.goal_theta = 0.0 #목표 각도

        self.move_flag = False
        self.rotate_flag = False
        self.initial_rotate_flag = False
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.pid_linear = PID(P=1.0, I=0.0001, D=0.0003, max_state=1.0, min_state=-1.0, dt=0.1)
        self.pid_angular = PID(P=2.0, I=0., D=0.0, max_state=1.0, min_state=-1.0, dt=0.1)

        self.is_object = False
        self.object_labels = []
        self.object_distances = []
        self.object_angles = []

        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1초마다 호출
    
    def publish_status(self):
        status_msg = RobotStatus()
        status_msg.robot_id = self.robot_id
        status_msg.status = self.status
        self.current_status_publisher.publish(status_msg)

    def object_detection(self, object_info):
        self.is_object = object_info.detected
        self.object_labels = object_info.labels
        self.object_ranges = object_info.ranges
        self.object_angles = object_info.angles
        # print(self.is_object, self.object_labels, self.object_ranges, self.object_angles)
    
    def set_goal(self, msg):
        self.move_flag = msg.move_flag
        self.rotate_flag = msg.rotate_flag
        self.initial_rotate_flag = msg.move_flag
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_theta = msg.theta

    def amcl_pose_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        _, _, self.position['theta'] = self.get_euler_from_quaternion(msg.pose.pose.orientation)
        print(self.position['x'], self.position['y'], self.position['theta'])

    def timer_callback(self):
        self.status = 'wait'

        if self.initial_rotate_flag is True:
            self.status = 'move'

            # 목표 위치와 현재 위치 간의 거리 및 각도 계산
            dx = self.goal_x - self.position['x']
            dy = self.goal_y - self.position['y']
            target_angle = atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.position['theta'])

            if abs(angle_error) > 0.3:
                angular_speed = self.pid_angular.process(angle_error)
                twist = Twist()
                twist.angular.z = angular_speed
                self.cmd_vel_publisher.publish(twist)
            else:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist)
                self.initial_rotate_flag = False

        elif self.move_flag is True:
            if self.is_object is True:
                if min(self.object_ranges) < 0.8:
                    print('prevent obstacle')
                    self.status = 'pause'
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_publisher.publish(twist)
                    return 0
            # 목표 위치와 현재 위치 간의 거리 및 각도 계산
            dx = self.goal_x - self.position['x']
            dy = self.goal_y - self.position['y']
            distance = sqrt(dx**2 + dy**2)
            target_angle = atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.position['theta'])

            if distance > 0.1:
                linear_speed = self.pid_linear.process(distance)
                angular_speed = self.pid_angular.process(angle_error)

                twist = Twist()
                twist.linear.x = linear_speed
                twist.angular.z = angular_speed
                self.cmd_vel_publisher.publish(twist)
            else:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist)
                self.move_flag = False
        
        elif self.rotate_flag is True:
            # 목표 위치와 현재 위치 간의 거리 및 각도 계산
                target_angle = self.goal_theta
                angle_error = self.normalize_angle(target_angle - self.position['theta'])
                if abs(angle_error) > 0.05:
                    angular_speed = self.pid_angular.process(angle_error)
                    twist = Twist()
                    twist.angular.z = angular_speed
                    self.cmd_vel_publisher.publish(twist)
                else:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_publisher.publish(twist)
                    self.rotate_flag = False
        
        self.publish_status()

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
        self.move_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()