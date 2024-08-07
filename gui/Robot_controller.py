import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt, QTimer
from struct import *
import socket
import datetime

import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_msgs.msg import GoalPose
from robot_msgs.msg import RobotStatus
from threading import Thread

import mysql.connector

shelf_IPs = {
    "shelf1":'192.168.1.100',
}

positions = {
    "o": (0., 0., 0.),
    "Shelf1": (1.5546208449304282, -0.4596189342427643, -1.437230999624169),
    "Shelf2": (1.0402942456593676, -0.40828154538591077, -1.6386375833115416),
    "Shelf3": (0.4565105916649657, -0.3396688475363866, -1.5449646256794836),
    "Shelf4": (1.5976331803669466, 0.5327394950611359, -1.615602490631877),
    "Shelf5": (1.0402942456593676, 0.5327394950611359, -1.615602490631877),
    "Shelf6": (0.4565105916649657, 0.5327394950611359, -1.5449646256794836),
    "A1": (1.811784145, -0.4596189342427643, 0),
    "A2": (1.5546208449304282, -0.4596189342427643, 0),
    "A3": (1.297457545, -0.4596189342427643, 0),
    "A4": (1.0402942456593676, -0.40828154538591077, 0),
    "A5": (0.748402419, -0.40828154538591077, 0),
    "A6": (0.4565105916649657, -0.3396688475363866, 0),
    "A7": (0.228255296, -0.3396688475363866, 0),
    "B1": (1.811784145, 0.5327394950611359, 0),
    "B2": (1.5976331803669466, 0.5327394950611359, 0),
    "B3": (1.297457545, 0.5327394950611359, 0),
    "B4": (1.0402942456593676, 0.5327394950611359, 0),
    "B5": (0.748402419, 0.5327394950611359, 0),
    "B6": (0.4565105916649657, 0.5327394950611359, 0),
    "B7": (0.228255296, 0.5327394950611359, 0),
}


# class GoalLauncher(rp.node.Node):
#     def __init__(self):
#         super().__init__('goal_launcher')
#         self.subscription = self.create_subscription(
#             PoseWithCovarianceStamped,
#             'amcl_pose',
#             self.listener_callback,
#             10)
#         self.subscription
#         self.current_pose = None

#     def listener_callback(self, msg):
#         self.current_pose = msg.pose.pose
#         self.get_logger().info(f'Received pose: {self.current_pose}')
#         if self.gui:
#             self.gui.set_pose(self.current_pose)

#     def set_gui(self, gui):
#         self.gui = gui

class GoalLauncher(Node):
    def __init__(self):
        super().__init__('gui_goal_publiser')
        self.goal_publisher = self.create_publisher(GoalPose, '/set_goal', 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose1',
            self.listener_callback,
            10)
        self.robot1_state_subscription = self.create_subscription(RobotStatus, '/robot1_status', self.robot1_status_callback, 10)
        self.subscription
        self.robot1_state_subscription
        self.current_pose = None

    def robot1_status_callback(self, msg):
        print(msg.status)
        if self.gui:
            if msg.robot_id == 'Mk.1':
                self.gui.set_robot1_status(msg.robot_id, msg.status)


    def listener_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.get_logger().info(f'Received pose: {self.current_pose}')
        if self.gui:
            self.gui.set_pose(self.current_pose)

    def set_gui(self, gui):
        self.gui = gui
        
    def publish_goal(self, selected_robot, key):
        msg = GoalPose()
        msg.robot_id = selected_robot
        msg.x = positions[key][0]
        msg.y = positions[key][1]
        msg.theta = positions[key][2]

        self.goal_publisher.publish(msg)
        print(msg.robot_id, msg.x, msg.y, msg.theta)

from_class = uic.loadUiType("./gui/Robot_controller.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self, goal_publisher):
        super().__init__()
        self.setupUi(self)
        self.goal_publisher = goal_publisher

        self.Shelf1.clicked.connect(lambda: self.set_goal("Shelf1"))
        self.Shelf2.clicked.connect(lambda: self.set_goal("Shelf2"))
        self.Shelf3.clicked.connect(lambda: self.set_goal("Shelf3"))
        self.Shelf4.clicked.connect(lambda: self.set_goal("Shelf4"))
        self.Shelf5.clicked.connect(lambda: self.set_goal("Shelf5"))
        self.Shelf6.clicked.connect(lambda: self.set_goal("Shelf6"))

        self.select1_button.clicked.connect(lambda: self.select_robot("Mk.1"))
        self.select2_button.clicked.connect(lambda: self.select_robot("Mk.2"))
        self.select3_button.clicked.connect(lambda: self.select_robot("Mk.3"))
        self.select4_button.clicked.connect(lambda: self.select_robot("Mk.4"))
        
        image_path = './map/test_map.png'

        # QPixmap 객체 생성 및 QLabel에 설정
        self.pixmap = QPixmap(image_path)

        self.scaled_pixmap = self.pixmap.scaled(self.label.size(), aspectRatioMode=True)
        self.label.setPixmap(self.scaled_pixmap)

        self.selected_robot = 'Mk.0'

        self.shelf1_IR_sensor_value = None
        self.shelf1_connected = False
        self.shelf1_connect()

        self.format = Struct('@iii')


        self.timer = QTimer()
        self.timer.timeout.connect(self.update_shelf)
        self.timer.start(1000)
 
        self.current_pose = None

        self.connect_to_database()
        self.show_table()
    
    def connect_to_database(self):
        global connection, cursor
        try:
            connection = mysql.connector.connect(
            host='127.0.0.1',
            database='task_database',
            user='root',
            password='rlatp0923'
            )
            if connection.is_connected():
                cursor = connection.cursor()
                return connection
            
        except error as e:
            print(f"Error: {e}")

    def show_table(self):
        query = """
        SELECT * FROM task_table;
        """
        cursor.execute(query)
        columns = cursor.column_names
        results = cursor.fetchall()
        
        if results:
            self.display_table_data(self.tableWidget, columns, results)

    def display_table_data(self, table, columns, results):
        table.setRowCount(len(results))
        table.setColumnCount(len(columns))
        table.setHorizontalHeaderLabels(columns)
        for row_idx, row_data in enumerate(results):
            for col_idx, col_data in enumerate(row_data):
                table.setItem(row_idx, col_idx, QTableWidgetItem(str(col_data)))

    def select_robot(self, robot_id):
        self.selected_robot = robot_id

    
    def set_robot1_status(self, robot_id, status):
        self.robot1_id.setText(robot_id)
        self.robot1_status.setText(status)


    def update_position(self):
        if self.current_pose:
            self.draw_position()

    def set_pose(self, pose):
        self.current_pose = pose
        self.update_position()

    def draw_position(self):
        x = 530 - self.current_pose.position.x * 300   # Example scale and offset
        y = 110 - self.current_pose.position.y * 200  # Example scale and offset
        print(x, y)
        updated_pixmap = self.scaled_pixmap.copy()
        painter = QPainter(updated_pixmap)
        pen = QPen(Qt.red)
        pen.setWidth(8)
        painter.setPen(pen)
        painter.drawPoint(int(x), int(y))
        painter.end()

        self.label.setPixmap(updated_pixmap)

    def shelf1_connect(self):
        try:
            self.shelf1_socket = socket.socket()
            self.shelf1_socket.connect((shelf_IPs["shelf1"], 80))
            self.shelf1_id.setText('shelf 1')
            self.shelf1_connected = True
        except:
            self.shelf1_connected = True
    
    def update_shelf(self):
        self.updateData1(1, 34, 0)
    
    def updateData1(self, machine_id, pin, status):
        if self.shelf1_connected == True:
            data = self.format.pack(machine_id, pin, status)
            req = self.shelf1_socket.send(data)
            rev = self.format.unpack(self.shelf1_socket.recv(self.format.size))
            self.machine_id = rev[0]
            if rev[1] == 34:
                self.shelf1_IR_sensor_value = rev[2]
                if self.shelf1_IR_sensor_value < 3900:
                    self.shelf1_status.setText('loaded')
                else:
                     self.shelf1_status.setText('unloaded')            
            print(rev)
        

    def set_goal(self, key):
        self.goal_publisher.publish_goal(self.selected_robot, key)

    def __del__(self):
        self.shelf1_socket.close()
        self.shelf1_connected = False

def main(args=None):
    rp.init(args=args)
    goal_publisher = GoalLauncher()

    app = QApplication(sys.argv)
    myWindows = WindowClass(goal_publisher)
    myWindows.show()

    goal_publisher.set_gui(myWindows)

    def spin():
        rp.spin(goal_publisher)

    spin_thread = Thread(target=spin)
    spin_thread.start()

    def shutdown_ros():
        goal_publisher.destroy_node()
        rp.shutdown()
    
    app.aboutToQuit.connect(shutdown_ros)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()