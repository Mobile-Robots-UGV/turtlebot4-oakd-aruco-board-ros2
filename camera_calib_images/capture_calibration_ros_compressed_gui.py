#!/usr/bin/env python3
import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

TOPIC = "/robot_09/oakd/rgb/image_raw/compressed"
OUT_DIR = os.path.expanduser("~/camera_calib_images/calib_images")

# Match your printed ChArUco board
CHARUCO_COLS = 11
CHARUCO_ROWS = 8
SQUARE_LENGTH = 0.015
MARKER_LENGTH = 0.011
ARUCO_DICT_NAME = "DICT_4X4_50"

# Save threshold
MIN_CORNERS_TO_SAVE = 25

DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
}

class CompressedCalibrationCapture(Node):
    def __init__(self):
        super().__init__("compressed_calibration_capture_gui")
        os.makedirs(OUT_DIR, exist_ok=True)

        self.subscription = self.create_subscription(
            CompressedImage,
            TOPIC,
            self.callback,
            10
        )

        self.frame = None
        self.saved = 0

        dict_id = DICT_MAP[ARUCO_DICT_NAME]
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.board = cv2.aruco.CharucoBoard(
            (CHARUCO_COLS, CHARUCO_ROWS),
            SQUARE_LENGTH,
            MARKER_LENGTH,
            self.aruco_dict
        )

        self.qr_detector = cv2.QRCodeDetector()

        self.get_logger().info(f"Subscribed to {TOPIC}")
        self.get_logger().info(f"Saving to {OUT_DIR}")

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.frame = img

def detect_qr(display, qr_detector):
    qr_texts = []
    try:
        retval, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(display)
        if retval and points is not None and decoded_info is not None:
            for i, pts in enumerate(points):
                pts = pts.astype(int)
                for j in range(4):
                    p1 = tuple(pts[j][0] if pts.ndim == 3 else pts[j])
                    p2 = tuple(pts[(j + 1) % 4][0] if pts.ndim == 3 else pts[(j + 1) % 4])
                    cv2.line(display, p1, p2, (255, 0, 255), 2)

                txt = decoded_info[i] if i < len(decoded_info) else ""
                if txt:
                    qr_texts.append(txt)
                    anchor = tuple(pts[0][0] if pts.ndim == 3 else pts[0])
                    cv2.putText(display, f"QR: {txt}", anchor,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    except Exception:
        pass
    return qr_texts

def main():
    rclpy.init()
    node = CompressedCalibrationCapture()

    cv2.namedWindow("Charuco Capture", cv2.WINDOW_NORMAL)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)

            if node.frame is None:
                blank = np.zeros((720, 960, 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting for compressed image topic...",
                            (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                cv2.imshow("Charuco Capture", blank)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                continue

            display = node.frame.copy()
            gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                node.aruco_dict,
                parameters=node.detector_params
            )

            marker_count = 0 if ids is None else len(ids)
            charuco_count = 0
            charuco_corners = None
            charuco_ids = None

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

                ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    markerCorners=corners,
                    markerIds=ids,
                    image=gray,
                    board=node.board
                )

                if ret is not None and charuco_ids is not None:
                    charuco_count = len(charuco_ids)
                    cv2.aruco.drawDetectedCornersCharuco(
                        display,
                        charuco_corners,
                        charuco_ids,
                        (0, 255, 255)
                    )

            qr_texts = detect_qr(display, node.qr_detector)

            valid = charuco_count >= MIN_CORNERS_TO_SAVE

            status_text = "VALID" if valid else "INVALID"
            status_color = (0, 255, 0) if valid else (0, 0, 255)

            cv2.putText(display, f"Topic: {TOPIC}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(display, f"ArUco markers: {marker_count}", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display, f"ChArUco corners: {charuco_count}", (20, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(display, f"Saved: {node.saved}", (20, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(display, f"Status: {status_text}", (20, 165),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 3)
            cv2.putText(display, f"Threshold: {MIN_CORNERS_TO_SAVE}+ corners", (20, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            cv2.putText(display, "SPACE=save valid | f=force save | q=quit",
                        (20, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if qr_texts:
                cv2.putText(display, f"QR detected: {len(qr_texts)}", (20, 270),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

            cv2.imshow("Charuco Capture", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                if valid:
                    filename = os.path.join(OUT_DIR, f"calib_{node.saved:03d}.jpg")
                    cv2.imwrite(filename, node.frame)
                    print(f"Saved valid image: {filename} ({charuco_count} corners)")
                    node.saved += 1
                else:
                    print(f"Rejected frame: only {charuco_count} ChArUco corners")

            elif key == ord('f'):
                filename = os.path.join(OUT_DIR, f"calib_{node.saved:03d}.jpg")
                cv2.imwrite(filename, node.frame)
                print(f"Force-saved image: {filename} ({charuco_count} corners)")
                node.saved += 1

            elif key == ord('q'):
                break

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

