#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray

import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import numpy as np
import time

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# ============================================================
# QoS COMPATIBLE CON micro-ROS (ESP32)
# BEST_EFFORT, KEEP_LAST, depth=1
# ============================================================
qos_micro_ros = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


class GesturePerception(Node):

    def __init__(self):
        super().__init__('gesture_perception')

        self.bridge = CvBridge()


        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True
        )
        self.mp_drawing = mp.solutions.drawing_utils

        self.create_subscription(
            Image,
            '/kinect/image_raw',
            self.rgb_callback,
            qos_micro_ros
        )

        self.create_subscription(
            Image,
            '/kinect/depth/image_raw',
            self.depth_callback,
            qos_micro_ros
        )

        self.pub_gesture = self.create_publisher(
            String,
            '/gesture_command',
            qos_micro_ros
        )

        self.pub_dist = self.create_publisher(
            Float32MultiArray,
            '/distance_zones',
            qos_micro_ros
        )

        # Control de frecuencia
        self.last_rgb_time = time.time()
        self.last_depth_time = time.time()

        # Ventanas de visualización
        cv2.namedWindow("Pose Detection", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth Heatmap", cv2.WINDOW_NORMAL)

        self.get_logger().info("Nodo de percepción + EXTRA iniciado correctamente.")


    def rgb_callback(self, msg: Image):

        # Limitar a ~10 Hz
        if time.time() - self.last_rgb_time < 0.1:
            return
        self.last_rgb_time = time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error en conversión RGB: {e}")
            return

        # MediaPipe requiere RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(frame_rgb)

        annotated_frame = frame.copy()

        if not results.pose_landmarks:
            # No se detecta pose, solo mostramos la imagen original
            self.get_logger().debug("No se detecta pose.")
            cv2.imshow("Pose Detection", annotated_frame)
            cv2.waitKey(1)
            return

        lm = results.pose_landmarks.landmark


        R_SHO = self.get_xyz(lm, self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value)
        R_ELB = self.get_xyz(lm, self.mp_pose.PoseLandmark.RIGHT_ELBOW.value)
        R_WRI = self.get_xyz(lm, self.mp_pose.PoseLandmark.RIGHT_WRIST.value)

        L_SHO = self.get_xyz(lm, self.mp_pose.PoseLandmark.LEFT_SHOULDER.value)
        L_ELB = self.get_xyz(lm, self.mp_pose.PoseLandmark.LEFT_ELBOW.value)
        L_WRI = self.get_xyz(lm, self.mp_pose.PoseLandmark.LEFT_WRIST.value)

        angle_right = self.angle(R_SHO, R_ELB, R_WRI)
        angle_left = self.angle(L_SHO, L_ELB, L_WRI)


        gesture = None

        # BACKWARD → ambos brazos abajo
        if angle_left < 110 and angle_right < 115:
            gesture = "backward"

        # FORWARD → ambos brazos levantados
        elif 145 < angle_left < 149 and angle_right > 150:
            gesture = "forward"

        # STOP → brazos extendidos horizontalmente
        elif 114 < angle_left <= 125 and 108 < angle_right < 120:
            gesture = "stop"

        # RIGHT → brazo derecho más levantado
        elif angle_right > angle_left and angle_right > 150:
            gesture = "right"

        # LEFT → brazo izquierdo más levantado
        elif angle_left > angle_right + 5 and angle_left > 140:
            gesture = "left"

        # Publicar gesto si se detecta
        if gesture:
            self.pub_gesture.publish(String(data=gesture))
            self.get_logger().info(
                f"Gesto detectado: {gesture} "
                f"(Ángulos → R={angle_right:.1f}°, L={angle_left:.1f}°)"
            )
        else:
            self.get_logger().debug(
                f"Ángulos → R={angle_right:.1f}°, L={angle_left:.1f}° — Sin gesto válido."
            )

        self.mp_drawing.draw_landmarks(
            annotated_frame,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_drawing.DrawingSpec(
                color=(0, 0, 255), thickness=2, circle_radius=3),
            connection_drawing_spec=self.mp_drawing.DrawingSpec(
                color=(0, 255, 0), thickness=2)
        )

        cv2.putText(
            annotated_frame, f"Angle L: {angle_left:.1f} deg",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
        )
        cv2.putText(
            annotated_frame, f"Angle R: {angle_right:.1f} deg",
            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2
        )
        if gesture:
            cv2.putText(
                annotated_frame, f"Gesture: {gesture}",
                (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

        cv2.imshow("Pose Detection", annotated_frame)
        cv2.waitKey(1)

     # ============================================================
    def depth_callback(self, msg: Image):

        # Limitar a ~10 Hz
        if time.time() - self.last_depth_time < 0.1:
            return
        self.last_depth_time = time.time()

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error al convertir depth: {e}")
            return

        if depth is None or depth.ndim != 2:
            return

        depth = depth.astype(np.float32)

        h, w = depth.shape
        third = w // 3

        left_region   = depth[:, :third]
        center_region = depth[:, third:2*third]
        right_region  = depth[:, 2*third:]

        # ------------------------------
        # Función para media segura [m]
        # ------------------------------
        def safe_mean_m(region: np.ndarray) -> float:
            arr = region.copy().astype(np.float32)
            # 0 o valores negativos los marcamos como NaN (sin medición)
            arr[arr <= 0] = np.nan
            m = np.nanmean(arr)
            if np.isnan(m):
                return 10.0  # valor "lejos" por defecto
            return float(m / 1000.0)  # mm → m

        dl = safe_mean_m(left_region)
        dc = safe_mean_m(center_region)
        dr = safe_mean_m(right_region)

        # Publicar en /distance_zones
        msg_out = Float32MultiArray()
        msg_out.data = [dl, dc, dr]
        self.pub_dist.publish(msg_out)

        self.get_logger().info(
            f"Distancias [m] → Izq={dl:.2f}, Centro={dc:.2f}, Der={dr:.2f}"
        )


        max_range_m = 5.0
        depth_m = depth / 1000.0
        depth_m[depth_m <= 0] = max_range_m  # sin medición → lejos

        depth_m = np.clip(depth_m, 0.0, max_range_m)
        # Normalizar [0, max_range] → [0, 255]
        norm = (depth_m / max_range_m * 255.0).astype(np.uint8)

        # Invertir para que distancias pequeñas se vean "calientes"
        norm_inv = 255 - norm

        heatmap = cv2.applyColorMap(norm_inv, cv2.COLORMAP_JET)

        # Dibujar líneas que separan las 3 zonas
        cv2.line(heatmap, (third, 0), (third, h), (255, 255, 255), 2)
        cv2.line(heatmap, (2 * third, 0), (2 * third, h), (255, 255, 255), 2)

        # Mostrar las distancias sobre la imagen
        cv2.putText(
            heatmap, f"L: {dl:.2f} m", (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )
        cv2.putText(
            heatmap, f"C: {dc:.2f} m", (third + 10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )
        cv2.putText(
            heatmap, f"R: {dr:.2f} m", (2 * third + 10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
        )

        cv2.imshow("Depth Heatmap", heatmap)
        cv2.waitKey(1)

    # ============================================================
    # UTILIDADES DE GEOMETRÍA
    # ============================================================
    def get_xyz(self, lm, idx: int) -> np.ndarray:
        """Retorna coordenadas normalizadas (x, y, z) de un landmark."""
        return np.array([lm[idx].x, lm[idx].y, lm[idx].z], dtype=np.float32)

    def angle(self, A: np.ndarray, B: np.ndarray, C: np.ndarray) -> float:
        """Calcula el ángulo ABC en grados."""
        AB = A - B
        CB = C - B

        dot = np.dot(AB, CB)
        norm = np.linalg.norm(AB) * np.linalg.norm(CB)
        if norm == 0.0:
            return 180.0

        value = np.clip(dot / norm, -1.0, 1.0)
        return float(np.degrees(np.arccos(value)))


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = GesturePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
