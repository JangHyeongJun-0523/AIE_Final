import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_msgs.msg import ObjectPose
from robot_msgs.msg import GoalPose
import time
import tf_transformations
from math import sqrt, atan2, pi

from robot_msgs.action import MoveToGoal

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
    
    def inegral_and_time_reset(self):
        self.integrated_state = 0.0
        self.pre_time = rclpy.clock.Clock().now().to_msg().sec

class MoveManager(Node):
    def __init__(self, ac_server):
        super().__init__('move_manager')

        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose1', self.amcl_pose_callback, 10)
        self.object_subscriber = self.create_subscription(ObjectPose, '/detected_object', self.object_detection, 10)

        self.ac_server = ac_server

    def amcl_pose_callback(self, msg):
        self.ac_server.position['x'] = msg.pose.pose.position.x
        self.ac_server.position['y'] = msg.pose.pose.position.y
        _, _, self.ac_server.position['theta'] = self.get_euler_from_quaternion(msg.pose.pose.orientation)
        print(self.ac_server.position['x'], self.ac_server.position['y'], self.ac_server.position['theta'])
    
    def get_euler_from_quaternion(self, quaternion):
        euler = tf_transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler
    
    def object_detection(self, object_info):
        self.ac_server.tmp_is_object = False
        self.ac_server.tmp_object_labels = []
        self.ac_server.tmp_object_ranges = []
        self.ac_server.tmp_object_angles = []
        if object_info.detector_id == 1:
            self.ac_server.tmp_is_object = object_info.detected
            self.ac_server.tmp_object_labels = object_info.labels
            self.ac_server.tmp_object_ranges = object_info.ranges
            self.ac_server.tmp_object_angles = object_info.angles

class MoveActionServer(Node):
    def __init__(self):
        super().__init__('moving_action_node')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped1', 10)

        self.action_server = ActionServer(self, MoveToGoal,'/robot1_moving_path', self.execute_callback)

        self.goal_x = 0.0  # 목표 위치의 x 좌표
        self.goal_y = 0.0  # 목표 위치의 y 좌표
        self.goal_theta = 0.0 #목표 각도


        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        self.tmp_is_object = False
        self.tmp_object_labels = []
        self.tmp_object_ranges = []
        self.tmp_object_angles = []


        self.is_object = False
        self.object_labels = []
        self.object_ranges = []
        self.object_angles = []


    def execute_callback(self, goal_handle):
        ranges = [10000]
        feedback_msg = MoveToGoal.Feedback()
        feedback_msg.feedback_status = 'wait'

        goal_data = goal_handle.request
        print(len(goal_data.x))

        self.pid_linear = PID(P=1.0, I=0, D=0.00, max_state=1.0, min_state=-1.0, dt=0.1)
        self.pid_angular = PID(P=2.0, I=0., D=0.0, max_state=1.0, min_state=-1.0, dt=0.1)

        if goal_handle.request.x and goal_handle.request.y and goal_handle.request.theta:
            for x, y, theta in zip(goal_handle.request.x, goal_handle.request.y, goal_handle.request.theta):

                self.goal_x = x
                self.goal_y = y
                self.goal_theta = theta

                print(self.goal_x, self.goal_y, self.goal_theta)

                angle_error = 1000
                while abs(angle_error) > 0.3:
                    if len(self.tmp_object_ranges):
                        self.object_ranges = self.tmp_object_ranges
                    if not len(self.object_ranges):
                        print('range is none')
                        
                    if len(self.object_ranges):
                        print(self.object_ranges)

                        if min(self.object_ranges) < 0.8:
                            print('prevent obstacle')
                            self.status = 'pause'
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.cmd_vel_publisher.publish(twist)

                            self.pid_angular.inegral_and_time_reset()

                            feedback_msg.feedback_status = 'pause'
                            goal_handle.publish_feedback(feedback_msg)

                            time.sleep(0.1)

                            continue

                    dx = self.goal_x - self.position['x']
                    dy = self.goal_y - self.position['y']
                    target_angle = atan2(dy, dx)
                    angle_error = self.normalize_angle(target_angle - self.position['theta'])

                    angular_speed = self.pid_angular.process(angle_error)
                    twist = Twist()
                    twist.angular.z = angular_speed
                    self.cmd_vel_publisher.publish(twist)

                    feedback_msg.feedback_status = 'move'
                    goal_handle.publish_feedback(feedback_msg)

                    time.sleep(0.1)
                
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_publisher.publish(twist)
                self.initial_rotate_flag = False

                distance = 100000

                self.pid_linear.inegral_and_time_reset()
                self.pid_angular.inegral_and_time_reset()

                while distance > 0.1:
                    if len(self.tmp_object_ranges):
                        self.object_ranges = self.tmp_object_ranges
                    if not len(self.object_ranges):
                        print('range is none')
                        
                    if len(self.object_ranges):
                        print(self.object_ranges)

                        if min(self.object_ranges) < 0.8:
                            print('prevent obstacle')
                            self.status = 'pause'
                            twist = Twist()
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.cmd_vel_publisher.publish(twist)

                            feedback_msg.feedback_status = 'pause'
                            goal_handle.publish_feedback(feedback_msg)

                            self.pid_linear.inegral_and_time_reset()
                            self.pid_angular.inegral_and_time_reset()

                            time.sleep(0.1)

                            continue

                    dx = self.goal_x - self.position['x']
                    dy = self.goal_y - self.position['y']
                    distance = sqrt(dx**2 + dy**2)
                    target_angle = atan2(dy, dx)
                    angle_error = self.normalize_angle(target_angle - self.position['theta'])

                    linear_speed = self.pid_linear.process(distance)
                    angular_speed = self.pid_angular.process(angle_error)

                    twist = Twist()
                    twist.linear.x = linear_speed
                    twist.angular.z = angular_speed
                    self.cmd_vel_publisher.publish(twist)

                    feedback_msg.feedback_status = 'move'
                    goal_handle.publish_feedback(feedback_msg)

                    time.sleep(0.1)
        
        else:
            print("not_valid")

        goal_handle.succeed()
        result = MoveToGoal.Result()
        result.result_status = 'done'
        result.x = self.position['x']
        result.y = self.position['y']
        result.theta = self.position['theta']
        return result

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    move_action_server = MoveActionServer()
    manager = MoveManager(ac_server = move_action_server)

    executor.add_node(manager)
    executor.add_node(move_action_server)

    try:
        executor.spin()

    finally:
        executor.shutdown()
        manager.destroy_node()
        move_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()