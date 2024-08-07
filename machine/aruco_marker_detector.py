import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv2 import aruco

class ArucomarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.img_sub = self.create_subscription(CompressedImage, '/camera1', self.camera_callback, 10)

        self.camera_matrix = np.array([[501.71184364, 0, 319.93415777],
                          [0, 500.97002316, 245.20985799],
                          [0, 0, 1]])

        self.dist_coeffs = np.array([0.20046918, -0.5471712, -0.00194516, -0.00144732, 0.42454179])



        self.marker_size = 30
        self.marker_3d_edges = np.array([[0,0,0],
                                    [0,self.marker_size,0],
                                    [self.marker_size,self.marker_size,0],
                                    [self.marker_size,0,0]], dtype = 'float32').reshape((4,1,3))

    def camera_callback(self, data):
        # Decode compressed image
        np_arr = np.frombuffer(data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        parameters =  aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
        

        # 마커(marker) 검출
        corners, ids, rejectedCandidates = detector.detectMarkers(image)

        # 검출된 마커들의 꼭지점을 이미지에 그려 확인
        for corner in corners:
            corner = np.array(corner).reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corner

            topRightPoint    = (int(topRight[0]),      int(topRight[1]))
            topLeftPoint     = (int(topLeft[0]),       int(topLeft[1]))
            bottomRightPoint = (int(bottomRight[0]),   int(bottomRight[1]))
            bottomLeftPoint  = (int(bottomLeft[0]),    int(bottomLeft[1]))

            # PnP
            ret, rvec, tvec = cv2.solvePnP(self.marker_3d_edges, corner, self.camera_matrix, self.dist_coeffs)
            if(ret):
                x=round(tvec[0][0],0);
                y=round(tvec[1][0],0);
                z=round(tvec[2][0],0);
                rx=round(np.rad2deg(rvec[0][0]),0);
                ry=round(np.rad2deg(rvec[1][0]),0);
                rz=round(np.rad2deg(rvec[2][0]),0);
                # PnP 결과를 이미지에 그려 확인
                text1 = f"{x},{y},{z}"
                text2 = f"{rx},{ry},{rz}"
                cv2.putText(image, text1, (int(topLeft[0]-10),   int(topLeft[1]+10)), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255))
                cv2.putText(image, text2, (int(topLeft[0]-10),   int(topLeft[1]+40)), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255))

        cv2.imshow('aruco detector', image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init()
    node = ArucomarkerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
