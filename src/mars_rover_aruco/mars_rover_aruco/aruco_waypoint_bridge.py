"""
ArUco Sequential Mission Node  (with self-exploration scan)
============================================================
State machine per waypoint:

  IDLE
   └─ (auto-start 10 s after launch)
  NAVIGATING  ← send Nav2 goal to pre-configured approach position
   └─ Nav2 succeeded  → SCANNING  (rotate 360° looking for marker)
   └─ Nav2 failed     → skip, advance
  SCANNING    ← rotate slowly, watch camera detections
   └─ Marker detected → APPROACHING
   └─ Full 360° done  → ARRIVED (not visible, log & advance)
  APPROACHING ← open-loop visual servo toward marker
   └─ distance ≤ arrival_dist → ARRIVED
   └─ marker lost              → ARRIVED
  ARRIVED     ← 3-second pause
   └─ advance index → NAVIGATING(next)
  RETURNING   ← Nav2 back to (0, 0, 0°)
  MISSION_COMPLETE
"""
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from action_msgs.msg import GoalStatus


class State:
    IDLE             = 'IDLE'
    NAVIGATING       = 'NAVIGATING'
    SCANNING         = 'SCANNING'
    APPROACHING      = 'APPROACHING'
    ARRIVED          = 'ARRIVED'
    RETURNING        = 'RETURNING'
    MISSION_COMPLETE = 'MISSION_COMPLETE'


# Rotation scan parameters
SCAN_ANGULAR_VEL = 0.35          # rad/s
SCAN_TOTAL_ANGLE = math.pi * 2   # full 360°
SCAN_TIMEOUT     = SCAN_TOTAL_ANGLE / SCAN_ANGULAR_VEL  # ~18 s


class ArucoWaypointBridge(Node):
    def __init__(self):
        super().__init__('aruco_waypoint_bridge')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('goal_frame',        'map')
        self.declare_parameter('arrival_distance',   1.2)
        self.declare_parameter('approach_speed',     0.15)
        self.declare_parameter('mission_sequence',   [0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

        self.goal_frame     = self.get_parameter('goal_frame').value
        self.arrival_dist   = self.get_parameter('arrival_distance').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.mission_seq    = list(self.get_parameter('mission_sequence').value)

        # Approach positions — 5 m in front of each marker, facing toward it
        # Computed as: marker_pos - (marker_pos / |marker_pos|) * 5 m
        self._waypoints = {
            0: {'x':  5.0, 'y':  0.0, 'yaw_deg':   0.0, 'name': 'Alpha'},    # marker (10, 0)
            1: {'x':  6.5, 'y':  6.5, 'yaw_deg':  45.0, 'name': 'Beta'},     # marker (10, 10)
            2: {'x':  0.0, 'y': 11.0, 'yaw_deg':  90.0, 'name': 'Gamma'},    # marker (0, 16)
            3: {'x': -6.5, 'y':  6.5, 'yaw_deg': 135.0, 'name': 'Delta'},    # marker (-10, 10)
            4: {'x': -6.5, 'y': -6.5, 'yaw_deg': 225.0, 'name': 'Epsilon'},  # marker (-10, -10)
            5: {'x':  6.5, 'y': -6.5, 'yaw_deg': 315.0, 'name': 'Zeta'},    # marker (10, -10)
            6: {'x':  0.0, 'y':-11.0, 'yaw_deg': 270.0, 'name': 'Eta'},     # marker (0, -16)
            7: {'x': 11.0, 'y':  0.0, 'yaw_deg':   0.0, 'name': 'Theta'},   # marker (16, 0)
            8: {'x':-11.0, 'y':  0.0, 'yaw_deg': 180.0, 'name': 'Iota'},    # marker (-16, 0)
            9: {'x':  2.5, 'y':  2.5, 'yaw_deg':  45.0, 'name': 'Kappa'},   # marker (6, 6)
        }

        # ── Mission state ──────────────────────────────────────────────
        self.state           = State.IDLE
        self._mission_index  = 0
        self._active_id      = None
        self._nav2_handle    = None
        self._latest_poses   = {}   # marker_id → detection info dict
        self._pause_timer    = None
        self._scan_elapsed   = 0.0  # seconds rotated so far

        # Return-to-start origin (rover spawns at 0, 0)
        self._start_x       = 0.0
        self._start_y       = 0.0
        self._start_yaw_deg = 0.0

        # ── Nav2 action client ─────────────────────────────────────────
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Subscriptions ──────────────────────────────────────────────
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        self.create_subscription(String,    '/aruco/detections', self._detection_cb, 10)
        self.create_subscription(PoseArray, '/aruco/poses',      self._poses_cb,     qos)

        # ── Publishers ─────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Main control loop (10 Hz) ──────────────────────────────────
        self.create_timer(0.1, self._control_loop)

        # ── Auto-start: wait for Nav2 then begin ───────────────────────
        self._startup_timer = self.create_timer(10.0, self._auto_start)
        self.get_logger().info(
            f'ArUco waypoint bridge ready. State: IDLE. '
            f'Mission sequence: {self.mission_seq}. Auto-starting in 10 s...')

    # ── Auto-start ────────────────────────────────────────────────────
    def _auto_start(self):
        self._startup_timer.cancel()
        self.get_logger().info('Auto-starting sequential mission.')
        self._navigate_next()

    # ── Navigate to next waypoint ─────────────────────────────────────
    def _navigate_next(self):
        # Cancel any leftover pause timer
        if self._pause_timer:
            self._pause_timer.cancel()
            self._pause_timer = None

        if self._mission_index >= len(self.mission_seq):
            self._return_to_start()
            return

        marker_id = self.mission_seq[self._mission_index]
        if marker_id not in self._waypoints:
            self.get_logger().warn(f'No waypoint for marker {marker_id}, skipping.')
            self._mission_index += 1
            self._navigate_next()
            return

        self._active_id = marker_id
        self.state = State.NAVIGATING
        wp = self._waypoints[marker_id]
        self.get_logger().info(
            f'[{self._mission_index + 1}/{len(self.mission_seq)}] '
            f'Navigating to "{wp["name"]}" (marker {marker_id}) '
            f'→ ({wp["x"]:.1f}, {wp["y"]:.1f})')

        if not self._nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 not available — retrying in 5 s.')
            self._pause_timer = self.create_timer(5.0, self._retry_once)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(wp['x'], wp['y'], wp['yaw_deg'])
        fut = self._nav2_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        fut.add_done_callback(self._goal_response_cb)

    def _retry_once(self):
        if self._pause_timer:
            self._pause_timer.cancel()
            self._pause_timer = None
        self._navigate_next()

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(
                f'Nav2 goal rejected for marker {self._active_id} — skipping.')
            self._skip_to_next()
            return
        self._nav2_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        self.get_logger().debug(
            f'Marker {self._active_id}: {dist:.2f} m remaining')

    def _result_cb(self, future):
        status = future.result().status
        wp_name = self._waypoints.get(self._active_id, {}).get('name', '?')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Nav2 reached approach for "{wp_name}". '
                f'Starting 360° scan for marker {self._active_id}...')
            self._start_scan()
        else:
            self.get_logger().warn(
                f'Nav2 goal failed (status {status}) for marker '
                f'{self._active_id} — skipping.')
            self._skip_to_next()

    # ── 360° rotation scan ────────────────────────────────────────────
    def _start_scan(self):
        self.state = State.SCANNING
        self._scan_elapsed = 0.0
        # Clear stale detections so we only react to fresh sightings
        self._latest_poses.pop(self._active_id, None)
        self.get_logger().info(
            f'Scanning for marker {self._active_id} '
            f'(timeout {SCAN_TIMEOUT:.0f} s)...')

    # ── Main control loop ─────────────────────────────────────────────
    def _control_loop(self):
        dt = 0.1  # timer period

        if self.state == State.SCANNING:
            # Check if marker appeared during rotation
            if self._active_id in self._latest_poses:
                self._stop()
                self.get_logger().info(
                    f'Marker {self._active_id} found after '
                    f'{self._scan_elapsed:.1f} s of scanning. '
                    f'Switching to fine approach.')
                self.state = State.APPROACHING
                return

            # Rotate and accumulate time
            self._scan_elapsed += dt
            if self._scan_elapsed >= SCAN_TIMEOUT:
                self._stop()
                wp_name = self._waypoints.get(self._active_id, {}).get('name', '?')
                self.get_logger().warn(
                    f'Full 360° scan done — marker {self._active_id} '
                    f'("{wp_name}") not visible. Declaring arrived.')
                self._declare_arrived()
                return

            twist = Twist()
            twist.angular.z = SCAN_ANGULAR_VEL
            self.cmd_pub.publish(twist)

        elif self.state == State.APPROACHING:
            if self._active_id not in self._latest_poses:
                self.get_logger().warn(
                    f'Lost marker {self._active_id} during approach — declaring arrived.')
                self._stop()
                self._declare_arrived()
                return

            info    = self._latest_poses[self._active_id]
            dist    = info.get('distance', 1.0)
            lateral = info.get('x', 0.0)

            if dist <= self.arrival_dist:
                self._stop()
                wp_name = self._waypoints.get(self._active_id, {}).get('name', '?')
                self.get_logger().info(
                    f'✓ Fine approach complete: marker {self._active_id} '
                    f'("{wp_name}") at {dist:.2f} m.')
                self._declare_arrived()
                return

            twist = Twist()
            twist.linear.x  = min(self.approach_speed, 0.05 * dist)
            twist.angular.z = -1.5 * (lateral / max(dist, 0.1))
            self.cmd_pub.publish(twist)

    # ── Arrival ───────────────────────────────────────────────────────
    def _declare_arrived(self):
        self.state = State.ARRIVED
        wp_name = self._waypoints.get(self._active_id, {}).get('name', '?')
        self.get_logger().info(
            f'★ Arrived at "{wp_name}" (marker {self._active_id}). '
            f'Pausing 3 s...')
        if self._pause_timer:
            self._pause_timer.cancel()
        self._pause_timer = self.create_timer(3.0, self._advance_once)

    def _advance_once(self):
        # One-shot: cancel immediately so it doesn't repeat
        if self._pause_timer:
            self._pause_timer.cancel()
            self._pause_timer = None
        self._mission_index += 1
        self._active_id = None
        self._navigate_next()

    def _skip_to_next(self):
        """Skip current waypoint without pause (Nav2 rejected/failed)."""
        self._mission_index += 1
        self._active_id = None
        self._navigate_next()

    # ── Return to start ───────────────────────────────────────────────
    def _return_to_start(self):
        self.state = State.RETURNING
        self.get_logger().info(
            f'*** All {len(self.mission_seq)} waypoints visited! '
            f'Returning to start ({self._start_x:.1f}, {self._start_y:.1f}, '
            f'{self._start_yaw_deg:.0f}°) ***')

        if not self._nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 not available for return — done.')
            self.state = State.MISSION_COMPLETE
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(
            self._start_x, self._start_y, self._start_yaw_deg)
        fut = self._nav2_client.send_goal_async(goal)
        fut.add_done_callback(self._return_goal_cb)

    def _return_goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Return goal rejected.')
            self.state = State.MISSION_COMPLETE
            return
        handle.get_result_async().add_done_callback(self._return_result_cb)

    def _return_result_cb(self, future):
        self.state = State.MISSION_COMPLETE
        status = future.result().status
        label = ('back at start position!' if status == GoalStatus.STATUS_SUCCEEDED
                 else f'return nav ended (status {status}).')
        self.get_logger().info(f'Mission complete — {label}')
        self._display_mission_complete()
        # Shut down after a short delay so the ASCII art flushes to the log
        self.create_timer(2.0, self._shutdown_cb)

    def _shutdown_cb(self):
        self.get_logger().info('Shutting down aruco_waypoint_bridge...')
        rclpy.shutdown()

    @staticmethod
    def _display_mission_complete():
        banner = """
+----------------------------------------------------------+
|                                                          |
|    .~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~.                    |
|    |                                |                    |
|    |    *  MISSION  COMPLETE  *     |                    |
|    |                                |                    |
|    |   All waypoints visited.       |                    |
|    |   Rover has returned to base.  |                    |
|    |                                |                    |
|    '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'                    |
|    |                                                     |
|    |                                                     |
|    |                                                     |
|   /|\\                                                    |
|  / | \\                                                   |
|                                                          |
+----------------------------------------------------------+
"""
        print(banner, flush=True)

    # ── Detection callbacks ────────────────────────────────────────────
    def _detection_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        for id_str, info in data.items():
            self._latest_poses[int(id_str)] = info

    def _poses_cb(self, msg: PoseArray):
        pass  # detections arrive via _detection_cb (JSON String)

    # ── Helpers ────────────────────────────────────────────────────────
    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _make_pose_stamped(self, x, y, yaw_deg):
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = self.goal_frame
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        yaw = math.radians(yaw_deg)
        ps.pose.orientation.z = math.sin(yaw / 2)
        ps.pose.orientation.w = math.cos(yaw / 2)
        return ps


def main(args=None):
    rclpy.init(args=args)
    node = ArucoWaypointBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
