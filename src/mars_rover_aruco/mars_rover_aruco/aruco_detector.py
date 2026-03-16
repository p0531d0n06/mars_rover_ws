"""
ArUco Detector Node
===================
Subscribes to the rover's depth camera (RGB + depth + camera_info) and
detects ArUco markers in each frame using OpenCV.

Publishes:
  /aruco/detections   (mars_rover_aruco/msg – we use a dict-style String for simplicity)
  /aruco/poses        (geometry_msgs/PoseArray) — 3D poses in camera_optical_frame
  /aruco/markers_viz  (visualization_msgs/MarkerArray) — RViz overlays

The node uses the depth image to refine the Z distance instead of relying
solely on the homography, giving accurate 3D pose in rough terrain lighting.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
import json


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('aruco_dictionary_id', 0)
        self.declare_parameter('marker_size', 0.20)
        self.declare_parameter('image_topic', '/depth_camera/image')
        self.declare_parameter('depth_topic', '')          # empty = no depth camera
        self.declare_parameter('camera_info_topic', '/depth_camera/camera_info')
        self.declare_parameter('camera_frame', 'depth_camera_optical_frame')
        self.declare_parameter('max_detection_range', 8.0)

        dict_id    = self.get_parameter('aruco_dictionary_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.cam_frame   = self.get_parameter('camera_frame').value
        self.max_range   = self.get_parameter('max_detection_range').value

        # ── OpenCV ArUco (4.6 legacy API) ─────────────────────────────
        dict_map = {
            0: cv2.aruco.DICT_4X4_50,
            1: cv2.aruco.DICT_5X5_100,
            2: cv2.aruco.DICT_6X6_250,
        }
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(dict_map[dict_id])
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.bridge     = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs   = None
        self.latest_depth  = None

        # ── Subscriptions ──────────────────────────────────────────────
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._camera_info_cb, img_qos)

        depth_topic = self.get_parameter('depth_topic').value
        if depth_topic:
            self.create_subscription(Image, depth_topic, self._depth_cb, img_qos)
        else:
            self.get_logger().info('No depth topic configured — using homography depth estimation.')

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self._image_cb, img_qos)

        # ── Publishers ─────────────────────────────────────────────────
        self.pub_poses   = self.create_publisher(PoseArray,    '/aruco/poses',       10)
        self.pub_detect  = self.create_publisher(String,       '/aruco/detections',  10)
        self.pub_viz     = self.create_publisher(MarkerArray,  '/aruco/markers_viz', 10)

        self.get_logger().info('ArUco detector ready.')

    # ── Callbacks ──────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info('Camera calibration received.')

    def _depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def _image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            return

        pose_array = PoseArray()
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = self.cam_frame

        viz_markers = MarkerArray()
        detections  = {}

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

        for i, marker_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0]
            rvec = rvecs[i][0]

            # Refine depth from depth image if available
            if self.latest_depth is not None:
                cx, cy = self._marker_centre_px(corners[i])
                depth_z = self._sample_depth(cx, cy)
                if depth_z is not None and not math.isnan(depth_z):
                    scale = depth_z / tvec[2] if tvec[2] > 0.01 else 1.0
                    tvec = tvec * scale

            dist = float(np.linalg.norm(tvec))
            if dist > self.max_range:
                continue

            # Convert rvec → quaternion
            rot_mat, _ = cv2.Rodrigues(rvec)
            quat = self._rot_to_quat(rot_mat)

            pose = Pose()
            pose.position    = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
            pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            pose_array.poses.append(pose)

            detections[int(marker_id)] = {
                'x': float(tvec[0]),
                'y': float(tvec[1]),
                'z': float(tvec[2]),
                'distance': dist,
            }

            # RViz sphere marker at detected position
            vm = Marker()
            vm.header = pose_array.header
            vm.ns = 'aruco_detections'
            vm.id = int(marker_id)
            vm.type = Marker.SPHERE
            vm.action = Marker.ADD
            vm.pose = pose
            vm.scale.x = vm.scale.y = vm.scale.z = 0.15
            vm.color.r = 0.0; vm.color.g = 1.0; vm.color.b = 0.0; vm.color.a = 0.9
            viz_markers.markers.append(vm)

            self.get_logger().info(
                f'Marker {marker_id} detected at ({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}) '
                f'dist={dist:.2f}m')

        self.pub_poses.publish(pose_array)
        self.pub_detect.publish(String(data=json.dumps(detections)))
        self.pub_viz.publish(viz_markers)

    # ── Helpers ────────────────────────────────────────────────────────
    @staticmethod
    def _marker_centre_px(corners):
        pts = corners[0]
        cx  = int(np.mean(pts[:, 0]))
        cy  = int(np.mean(pts[:, 1]))
        return cx, cy

    def _sample_depth(self, cx, cy, radius=3):
        """Sample median depth in a small patch around (cx, cy)."""
        h, w = self.latest_depth.shape
        x0 = max(0, cx - radius); x1 = min(w, cx + radius + 1)
        y0 = max(0, cy - radius); y1 = min(h, cy + radius + 1)
        patch = self.latest_depth[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.01)]
        return float(np.median(valid)) if len(valid) > 0 else None

    @staticmethod
    def _rot_to_quat(R):
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
