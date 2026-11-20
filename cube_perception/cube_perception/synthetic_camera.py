import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

print("All modules imported!")

FRAME = 'camera_optical_frame'


class SyntheticCamera(Node):
    def __init__(self):
        super().__init__('synthetic_camera')

        # set default cube position
        self.declare_parameter('cube_x', 0.45)
        self.declare_parameter('cube_y', 0.10)
        self.declare_parameter('cube_size', 0.05)
        self.declare_parameter('cube_top_z', 0.05)

        # set default camera position and pixel intensity
        self.declare_parameter('cam_x', 0.40)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 0.80)
        self.declare_parameter('noise_sigma', 3.0)

        # pinhole model constant
        self.w, self.h = 640, 480
        self.fx = self.fy = 500.0
        self.cx, self.cy = 320.0, 240.0

        self.bridge = CvBridge()

        # create three publishers
        # /camera/image_raw (Image), /camera/camera_info (CameraInfo),
        # /cube_ground_truth (PoseStamped)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.cube_pub = self.create_publisher(PoseStamped, '/cube_ground_truth', 10)

        # create a timer that calls self.tick at 10 Hz
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        # get current cube dimensions
        cube_x = self.get_parameter('cube_x').value
        cube_y = self.get_parameter('cube_y').value
        cube_size = self.get_parameter('cube_size').value
        cube_top_z = self.get_parameter('cube_top_z').value

        # get current camera position
        cam_x = self.get_parameter('cam_x').value
        cam_y = self.get_parameter('cam_y').value
        cam_z = self.get_parameter('cam_z').value
        noise_sigma = self.get_parameter('noise_sigma').value


        # project the cube's top-face centre from base_link into pixels (u, v)
        # depth = cam_z - cube_top_z
        # express the cube position relative to the camera, in optical-frame axes
        # apply the pinhole model: u = fx * x / depth + cx, etc.

        depth = cam_z - cube_top_z
        
        x_opt = cube_x - cam_x
        y_opt = -(cube_y - cam_y)
        z_opt = -(cube_top_z - cam_z)

        u = self.fx*x_opt/depth + self.cx
        v = self.fy*y_opt/depth + self.cy

        # compute the cube's apparent half-width in pixels
        half_width = 0.5*self.fx*cube_size/depth

        # render the image
        # grey background (self.h x self.w x 3, uint8)
        # green filled square at (u, v)
        # gaussian noise with noise_sigma (careful with uint8 overflow!)
        img = np.full((self.h, self.w, 3), 120, np.uint8)
        cv2.rectangle(img, (int(u - half_width), int(v - half_width)),
                      (int(u + half_width), int(v + half_width)), (0, 200, 0), -1)
        noise = np.random.normal(0, noise_sigma, img.shape)
        img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        # publish the Image (bgr8). shared timestamp for the
        # image, camera_info and ground truth. frame_id = FRAME
        stamp = self.get_clock().now().to_msg()

        msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_optical_frame'
        self.image_pub.publish(msg)

        # publish CameraInfo: width, height, distortion_model, d, k, r, p
        # (zero distortion, so this is just the intrinsic matrix)
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = self.w, self.h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_pub.publish(info)

        # publish ground truth: PoseStamped in 'base_link' at the cube's
        # top-face centre, identity orientation
        gt = PoseStamped()
        gt.header.stamp = stamp
        gt.header.frame_id = 'base_link'
        gt.pose.position.x = cube_x
        gt.pose.position.y = cube_y
        gt.pose.position.z = cube_top_z
        gt.pose.orientation.w = 1.0
        self.cube_pub.publish(gt)


def main():
    # init, spin, shutdown
    rclpy.init()
    rclpy.spin(SyntheticCamera())
    rclpy.shutdown()