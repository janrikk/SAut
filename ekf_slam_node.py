#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import csv
import os

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tralha.msg import ArucoDetection
from visualization_msgs.msg import Marker, MarkerArray

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def motion_model(x, u, dt):
    x_new = x.copy()
    v, w = u
    theta = x[2]
    x_new[0] += v * np.cos(theta) * dt
    x_new[1] += v * np.sin(theta) * dt
    x_new[2] = wrap_to_pi(theta + w * dt)
    return x_new

def jacobian_robot(x, u, dt):
    v, w = u
    theta = x[2]
    Jr = np.eye(3)
    Jr[0, 2] = -v * np.sin(theta) * dt
    Jr[1, 2] =  v * np.cos(theta) * dt
    return Jr

def get_color_from_uncertainty(u_norm):
    u_norm = min(max(u_norm, 0.0), 1.0)
    if u_norm < 0.33:
        ratio = u_norm / 0.33
        r = ratio
        g = 1.0
        b = 0.0
    elif u_norm < 0.66:
        ratio = (u_norm - 0.33) / 0.33
        r = 1.0
        g = 1.0 - 0.5 * ratio
        b = 0.0
    else:
        ratio = (u_norm - 0.66) / 0.34
        r = 1.0
        g = 0.5 - 0.5 * ratio
        b = 0.0
    return r, g, b

class EKFSLAMNode:
    def __init__(self):
        rospy.init_node('ekf_slam_node')

        self.odom_path_file = os.path.join(os.path.expanduser('~'), 'odom_path.csv')
        self.initial_pose_printed = False

        if rospy.has_param('~initial_pose'):
            ip = rospy.get_param('~initial_pose')
            x0, y0, th0 = ip['x'], ip['y'], ip['theta']
            self.state = np.array([x0, y0, th0])
            self.cov = np.eye(3) * 1e-3
            self.initialized_from_aruco = True
        else:
            self.state = np.array([0.0, 0.0, 0.0])
            self.cov = np.eye(3) * 1e-3
            self.initialized_from_aruco = False

        self.odom_state = self.state[:3].copy()

        self.Q = np.diag([0.001, 0.001, np.deg2rad(0.1)])**2
        self.R = np.diag([0.2, np.deg2rad(15)])**2

        self.landmark_dict = {}
        self.landmark_observations = {}
        self.n_landmarks = 0
        self.last_odom_time = None

        self.pose_pub = rospy.Publisher("/ekf_pose", PoseStamped, queue_size=10)
        self.pose_cov_pub = rospy.Publisher("/ekf_pose_cov", PoseWithCovarianceStamped, queue_size=10)
        self.path_pub = rospy.Publisher("/ekf_path", Path, queue_size=10)
        self.landmarks_pub = rospy.Publisher("/ekf_landmarks", MarkerArray, queue_size=10)
        self.odom_path_pub = rospy.Publisher("/odom_path", Path, queue_size=10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = "map"

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/aruco_measurements", ArucoDetection, self.aruco_callback)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("EKF SLAM Node iniciado")

    def ekf_predict(self, u, dt):
        x_prior = self.state[:3].copy()
        x_pred = motion_model(x_prior, u, dt)
        self.state[:3] = x_pred

        Jr = jacobian_robot(x_prior, u, dt)
        n = len(self.state)
        F = np.eye(n)
        F[:3, :3] = Jr
        self.cov = F @ self.cov @ F.T + np.pad(self.Q, ((0, n-3), (0, n-3)), 'constant')

        self.odom_state = motion_model(self.odom_state, u, dt)

        pose_msg_odom = PoseStamped()
        pose_msg_odom.header.stamp = rospy.Time.now()
        pose_msg_odom.header.frame_id = "map"
        pose_msg_odom.pose.position.x = self.odom_state[0]
        pose_msg_odom.pose.position.y = self.odom_state[1]
        pose_msg_odom.pose.position.z = 0.0

        quat_odom = quaternion_from_euler(0, 0, self.odom_state[2])
        pose_msg_odom.pose.orientation.x = quat_odom[0]
        pose_msg_odom.pose.orientation.y = quat_odom[1]
        pose_msg_odom.pose.orientation.z = quat_odom[2]
        pose_msg_odom.pose.orientation.w = quat_odom[3]

        self.odom_path_msg.poses.append(pose_msg_odom)
        self.odom_path_pub.publish(self.odom_path_msg)

        with open(self.odom_path_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([pose_msg_odom.pose.position.x, pose_msg_odom.pose.position.y, self.odom_state[2]])
        self.publish_pose()

    def initialize_landmark(self, landmark_id, range_meas, bearing_meas):
        x_r, y_r, theta_r = self.state[:3]
        lx = x_r + range_meas * np.cos(theta_r + bearing_meas)
        ly = y_r + range_meas * np.sin(theta_r + bearing_meas)
        self.state = np.append(self.state, [lx, ly])
        self.n_landmarks += 1
        old_cov = self.cov
        n_old = old_cov.shape[0]
        n_new = n_old + 2
        new_cov = np.zeros((n_new, n_new))
        new_cov[:n_old, :n_old] = old_cov
        new_cov[n_old:, n_old:] = np.eye(2) * 1.0
        self.cov = new_cov
        self.landmark_dict[landmark_id] = n_old
        rospy.loginfo(f"[LANDMARK INIT] ID={landmark_id} posição=({lx:.2f}, {ly:.2f})")

    def aruco_callback(self, msg):
        landmark_id = msg.id
        z = np.array([msg.range, msg.bearing])

        if not self.initialized_from_aruco:
            r, b = msg.range, msg.bearing
            x_r = -r * np.cos(b)
            y_r = -r * np.sin(b)
            theta_r = wrap_to_pi(np.pi + b)
            self.state[:3] = np.array([x_r, y_r, theta_r])
            self.initialized_from_aruco = True
            rospy.loginfo(f"[INIT] Pose inicial com ArUco {landmark_id}: ({x_r:.2f}, {y_r:.2f}, {np.rad2deg(theta_r):.1f}º)")
            return

        if landmark_id not in self.landmark_dict:
            self.initialize_landmark(landmark_id, msg.range, msg.bearing)
            self.landmark_observations[landmark_id] = 1
            self.publish_landmarks()
            return

        self.landmark_observations[landmark_id] += 1
        obs_count = self.landmark_observations[landmark_id]
        idx = self.landmark_dict[landmark_id]
        x_r, y_r, theta_r = self.state[:3]
        lx, ly = self.state[idx], self.state[idx + 1]
        dx = lx - x_r
        dy = ly - y_r
        q = dx**2 + dy**2
        expected_range = np.sqrt(q)
        expected_bearing = wrap_to_pi(np.arctan2(dy, dx) - theta_r)
        z_hat = np.array([expected_range, expected_bearing])
        y_k = z - z_hat
        y_k[1] = wrap_to_pi(y_k[1])

        decay = min(obs_count / 10.0, 1.0)
        base_r = 0.15 + 0.15 * msg.range
        base_b = np.deg2rad(8 + 5 * msg.range)
        range_noise = base_r * (1.0 - 0.7 * decay)
        bearing_noise = base_b * (1.0 - 0.7 * decay)
        R_dynamic = np.diag([range_noise, bearing_noise])**2

        n = len(self.state)
        H = np.zeros((2, n))
        H[0, 0] = -dx / expected_range
        H[0, 1] = -dy / expected_range
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1.0
        H[0, idx]     = dx / expected_range
        H[0, idx + 1] = dy / expected_range
        H[1, idx]     = -dy / q
        H[1, idx + 1] = dx / q

        S = H @ self.cov @ H.T + R_dynamic
        mahalanobis = y_k.T @ np.linalg.inv(S) @ y_k
        if mahalanobis > 9.21:
            return

        K = self.cov @ H.T @ np.linalg.inv(S)
        self.state += K @ y_k
        self.state[2] = wrap_to_pi(self.state[2])
        I = np.eye(n)
        self.cov = (I - K @ H) @ self.cov

        self.publish_pose()
        self.publish_landmarks()

    def publish_pose(self):
        if not self.initialized_from_aruco:
            return

        now = rospy.Time.now()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.state[0]
        pose_msg.pose.position.y = self.state[1]
        pose_msg.pose.position.z = 0.0
        theta = self.state[2]
        quat = quaternion_from_euler(0, 0, theta)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        self.pose_pub.publish(pose_msg)

        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = pose_msg.header
        pose_cov.pose.pose = pose_msg.pose
        pose_cov.pose.covariance[:6] = [
            self.cov[0,0], self.cov[0,1], 0.0,
            self.cov[1,0], self.cov[1,1], 0.0
        ]
        pose_cov.pose.covariance[35] = self.cov[2,2]
        self.pose_cov_pub.publish(pose_cov)

        t_map_odom = geometry_msgs.msg.TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.translation.x = -self.state[0]
        t_map_odom.transform.translation.y = -self.state[1]
        t_map_odom.transform.translation.z = 0.0
        inv_theta = -self.state[2]
        inv_quat = quaternion_from_euler(0, 0, inv_theta)
        t_map_odom.transform.rotation.x = inv_quat[0]
        t_map_odom.transform.rotation.y = inv_quat[1]
        t_map_odom.transform.rotation.z = inv_quat[2]
        t_map_odom.transform.rotation.w = inv_quat[3]
        self.br.sendTransform(t_map_odom)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.br.sendTransform(t)

        self.path_msg.header.stamp = now
        self.path_msg.poses.append(pose_msg)
        self.path_pub.publish(self.path_msg)

    def publish_landmarks(self):
        marker_array = MarkerArray()
        marker_id = 0
        for lm_id, idx in self.landmark_dict.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "landmarks"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.state[idx]
            marker.pose.position.y = self.state[idx + 1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            lm_cov = self.cov[idx:idx+2, idx:idx+2]
            uncertainty = np.trace(lm_cov)
            max_uncertainty = 1.0
            u_norm = min(uncertainty / max_uncertainty, 1.0)
            r, g, b = get_color_from_uncertainty(u_norm)
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
        self.landmarks_pub.publish(marker_array)

    def odom_callback(self, msg):
        if not self.initialized_from_aruco:
            return
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        now = msg.header.stamp.to_sec()
        if self.last_odom_time is None:
            self.last_odom_time = now
            return
        dt = now - self.last_odom_time
        self.last_odom_time = now
        u = [v, w]
        self.ekf_predict(u, dt)

    def on_shutdown(self):
        rospy.loginfo("Pose final do robô:")
        rospy.loginfo(f"x = {self.state[0]:.2f}, y = {self.state[1]:.2f}, θ = {np.rad2deg(self.state[2]):.1f}º")
        rospy.loginfo("Coordenadas dos ArUcos estimados:")
        for lm_id, idx in self.landmark_dict.items():
            x, y = self.state[idx], self.state[idx + 1]
            rospy.loginfo(f"ID {lm_id} → ({x:.2f}, {y:.2f})")

if __name__ == '__main__':
    try:
        open(os.path.join(os.path.expanduser('~'), 'odom_path.csv'), 'w').close()
        node = EKFSLAMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass