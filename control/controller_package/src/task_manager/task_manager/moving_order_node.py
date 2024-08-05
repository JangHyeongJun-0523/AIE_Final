import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_msgs.msg import GoalPose
from robot_msgs.msg import RobotStatus
from robot_msgs.action import MoveToGoal
from task_manager.A_star import AStarPlanner
import math
import tf_transformations

class OrderMove(Node):
    def __init__(self):
        super().__init__('moving_order_node')
        
        self.goal_subscriber = self.create_subscription(GoalPose, '/set_goal', self.path_generator, 10)
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.robot1_status_publisher = self.create_publisher(RobotStatus, '/robot1_status', 10)
        self.action_client = ActionClient(self, MoveToGoal, '/moving_path')

        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    
    def amcl_callback(self, msg):
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        _, _, self.position['theta'] = self.get_euler_from_quaternion(msg.pose.pose.orientation)
        print(self.position['x'], self.position['y'], self.position['theta'])

    def get_euler_from_quaternion(self, quaternion):
        euler = tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler


    def path_generator(self, goal):
        goal_x = goal.x
        goal_y = goal.y
        a_star = AStarPlanner(resolution=1, rr=0.5, padding=3)
        rx, ry, tpx, tpy, tvec_x, tvec_y = a_star.planning(self.position['x'], self.position['y'], goal_x, goal_y)   

        if len(tpx) == 0:
            print('no path found')
            return 

        theta = self.calculate_theta(tvec_x, tvec_y)

        self.send_goal(tpx, tpy, theta)
    
    def calculate_theta(self, x, y):
        theta_list = []
        for gx, gy in zip(x, y):
            print(gx, gy)
            theta = math.atan2(gx, gy)
            theta_list.append(theta)

        return theta_list


    def send_goal(self, tpx, tpy, theta):
        goal_msg = MoveToGoal.Goal()
        goal_msg.x = tpx
        goal_msg.y = tpy
        goal_msg.theta = theta
        print(tpx, tpy, theta)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        msg = RobotStatus()
        msg.robot_id = 'Mk.1'
        msg.status = result.result_status
        self.robot1_status_publisher.publish(msg)

        print(result.result_status, result.x, result.y, result.theta)
        self.get_logger().info('Result: {0}'.format(result.result_status))
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        msg = RobotStatus()
        msg.robot_id = 'Mk.1'
        msg.status = feedback.feedback_status
        self.robot1_status_publisher.publish(msg)

        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback_status))



def main(args=None):
    rclpy.init(args=args)
    action_client = OrderMove()
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
