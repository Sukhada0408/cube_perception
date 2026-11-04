import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs 

print("All modules imported!")
class Detector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # declare parameters
        # object_top_z=0.05, hsv_lo=[40,100,50], hsv_hi=[80,255,255], min_area_px=50.0
        self.declare_parameter('object_top_z', 0.05)
        self.declare_parameter('hsv_lo', [40,100,50])
        self.declare_parameter('hsv_hi', [80,255,255])
        self.declare_parameter('min_area_px', 50.0)

        self.bridge = CvBridge()
        self.K = None

        # create a tf2_ros.Buffer and a TransformListener attached to this node
        # (store both on self, otherwise the listener gets garbage-collected)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        # subscribers for /camera/camera_info -> self.info_cb
        #                 /camera/image_raw  -> self.image_cb
        self.cam_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_cb,
            10
        )

        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            10
        )
        

        # publishers: /object_pose (PoseStamped), /detector/debug_image (Image)
        self.obj_pose = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.debug_img = self.create_publisher(Image, '/detector/debug_image', 10)


    def info_cb(self, msg):
        # store the intrinsic matrix
        self.K = msg.k

    def image_cb(self, msg):
        # guard: return if no CameraInfo yet; extract fx, fy, cx, cy from K
        if self.K is None:
            return
        
        fx = self.K[0]
        cx = self.K[2]
        fy = self.K[4]
        cy = self.K[5]

        # segmentation
        #   - ROS Image -> OpenCV BGR (cv_bridge)
        #   - BGR -> HSV, then cv2.inRange with the parameter thresholds
        #   - morphological opening to remove speckle
        self.hsv_lo = np.array(self.get_parameter('hsv_lo').value, dtype=np.uint8)
        self.hsv_hi = np.array(self.get_parameter('hsv_hi').value, dtype=np.uint8)

        img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_raw = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(hsv_raw, self.hsv_lo, self.hsv_hi)

        kernel = np.ones((3,3), np.uint8)
        mask_open = cv2.morphologyEx(mask_hsv, cv2.MORPH_OPEN, kernel)


        # find the largest contour, reject it if its area < min_area_px,
        # compute the centroid (u, v) from image moments
        self.min_area_px = self.get_parameter('min_area_px').value
        contours = cv2.findContours(mask_open, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

        if not contours:
            return

        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour)>=self.min_area_px:
            moment = cv2.moments(largest_contour)
            u = moment['m10']/moment['m00']
            v = moment['m01']/moment['m00']
        else:
            return
        
        # back-projection to the camera frame
        #   - Z = camera height in base_link - object_top_z
        #   - X = (u - cx) * Z / fx ; Y = (v - cy) * Z / fy
        #   - package as PointStamped (header = msg.header)

        source = msg.header.frame_id
        self.object_top_z = self.get_parameter('object_top_z').value

        # Wrap in try/except tf2_ros.TransformException, log a throttled warning on failure
        try:
            tf_cam = self.buffer.lookup_transform('base_link', source, Time())
            Z = tf_cam.transform.translation.z - self.object_top_z
            X = (u-cx)*Z/fx
            Y = (v-cy)*Z/fy

            cam_proj = PointStamped()
            cam_proj.header = msg.header
            cam_proj.point.x = X
            cam_proj.point.y = Y
            cam_proj.point.z = Z

            # transform into 'base_link' with the TF buffer.
            cam_base = self.buffer.transform(
            cam_proj,
            'base_link',
            timeout=Duration(seconds=0.1)
            )
        except (tf2_ros.TransformException) as ex:
            self.get_logger().warn(f"Could not transform: {ex}", throttle_duration_sec=2.0)


        # publish PoseStamped (frame 'base_link', stamp from the IMAGE header,
        # identity orientation: we don't estimate orientation)
        pose_est = PoseStamped()
        pose_est.header = cam_base.header
        pose_est.pose.position.x = cam_base.point.x
        pose_est.pose.position.y = cam_base.point.y
        pose_est.pose.position.z = cam_base.point.z
        pose_est.pose.orientation.w = 1.0
        self.obj_pose.publish(pose_est)

        # draw the centroid on the BGR image and publish it as the debug image
        cv2.circle(img_bgr, (int(u), int(v)), 1, (0, 0, 255), -1)
        im_msg = self.bridge.cv2_to_imgmsg(img_bgr, 'bgr8')
        im_msg.header = msg.header
        self.debug_img.publish(im_msg)


def main():
    # init, spin, shutdown
    rclpy.init()
    node = Detector()
    rclpy.spin(node)
    rclpy.shutdown()