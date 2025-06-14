#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from tralha.msg import ArucoDetection

ARUCO_DICT = aruco.DICT_ARUCO_ORIGINAL
MARKER_LENGTH = 0.10
MIN_FRAMES_CONSISTENTES = 3

# --- Matriz da câmara ---
camera_matrix = np.array([
    [504.9629545390832, 0.0, 320.0448609274721],
    [0.0, 502.2914546831238, 240.8836661448442],
    [0.0, 0.0, 1.0]
])

dist_coeffs = np.zeros((5, 1))

class ArucoRangeBearing:
    def __init__(self):
        rospy.init_node('aruco_measure_node')
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        self.pub = rospy.Publisher("/aruco_measurements", ArucoDetection, queue_size=10)
        self.aruco_dict = aruco.Dictionary_get(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters_create()
        self.id_counter = {}
        self.valid_ids = set()
        rospy.loginfo("Aruco Measure inicializado")

    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Erro a converter imagem: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, MARKER_LENGTH, camera_matrix, dist_coeffs
            )

            detected_ids = set()

            for i in range(len(ids)):
                marker_id = int(ids[i])
                detected_ids.add(marker_id)

                # Contagem de frames
                self.id_counter[marker_id] = self.id_counter.get(marker_id, 0) + 1

                if self.id_counter[marker_id] >= MIN_FRAMES_CONSISTENTES:
                    self.valid_ids.add(marker_id)

                if marker_id not in self.valid_ids:
                    continue

                tvec = tvecs[i][0]
                x, y, z = tvec
                distance = np.linalg.norm(tvec)
                bearing = np.arctan2(-x, z)

                msg = ArucoDetection()
                msg.id = marker_id
                msg.range = distance
                msg.bearing = bearing
                self.pub.publish(msg)

                rospy.loginfo(f"[ID {marker_id}] Range = {distance:.3f} m | Bearing = {np.degrees(bearing):.1f}°")

            ids_atuais = set(self.id_counter.keys())
            desaparecidos = ids_atuais - detected_ids
            for lost_id in desaparecidos:
                del self.id_counter[lost_id]
                self.valid_ids.discard(lost_id)

        else:
            self.id_counter.clear()
            self.valid_ids.clear()

if __name__ == '__main__':
    try:
        ArucoRangeBearing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
