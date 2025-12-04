#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# QoS para sensores (cámaras)
qos_sensor = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# QoS para comandos
qos_gesture = QoSProfile(
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

        # Suscripción al topic de la rosbag o cámara
        self.create_subscription(Image, '/kinect/image_raw', self.rgb_callback, qos_sensor)

        # Publicador de gestos
        self.pub_cmd = self.create_publisher(String, '/gesture_command', qos_gesture)

        self.last_time = time.time()
        self.get_logger().info(" Nodo de percepción listo. Mostrando ventana de pose...")

    def rgb_callback(self, msg):
        # Limitar a ~10 Hz
        if time.time() - self.last_time < 0.1:
            return
        self.last_time = time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(frame_rgb)

        annotated_frame = frame.copy()

        if not results.pose_landmarks:
            self.get_logger().debug("No se detecta pose.")
            cv2.imshow("Pose Detection", annotated_frame)
            cv2.waitKey(1)
            return

        lm = results.pose_landmarks.landmark

        # --- Calcular ángulos y gesto ---
        R_SHO = self.get_xyz(lm, self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value)
        R_ELB = self.get_xyz(lm, self.mp_pose.PoseLandmark.RIGHT_ELBOW.value)
        R_WRI = self.get_xyz(lm, self.mp_pose.PoseLandmark.RIGHT_WRIST.value)

        L_SHO = self.get_xyz(lm, self.mp_pose.PoseLandmark.LEFT_SHOULDER.value)
        L_ELB = self.get_xyz(lm, self.mp_pose.PoseLandmark.LEFT_ELBOW.value)
        L_WRI = self.get_xyz(lm, self.mp_pose.PoseLandmark.LEFT_WRIST.value)

        angle_right = self.angle(R_SHO, R_ELB, R_WRI)
        angle_left = self.angle(L_SHO, L_ELB, L_WRI)

        # Lógica de gestos
        gesture = None

        # 1. BACKWARD: ambos brazos colgando → ángulos bajos
        if angle_left < 110 and angle_right < 115:
            gesture = "backward"

        # 2. FORWARD: ambos brazos levantados verticalmente → ángulos altos
        elif 145 < angle_left < 149 and angle_right > 150:
            gesture = "forward"

        # 3. STOP: brazos extendidos horizontalmente → ángulos medios
        elif 114 < angle_left <= 125 and 108 < angle_right < 120:
            gesture = "stop"

        # 4. RIGHT: brazo derecho más levantado que el izquierdo
        elif angle_right > angle_left  and angle_right > 150:
            gesture = "right"

        # 5. LEFT: brazo izquierdo más levantado que el derecho
        elif angle_left > angle_right + 5 and angle_left > 140:
            gesture = "left"

        # Si no coincide con ninguno → None


        # Si no coincide con ninguno → None

        # Publicar gesto si existe
        if gesture:
            self.pub_cmd.publish(String(data=gesture))
            self.get_logger().info(f"Gesto detectado: {gesture}")
        else:
            self.get_logger().debug(f"Ángulos → R: {angle_right:.1f}°, L: {angle_left:.1f}° — Ningún gesto coincide.")

        # Dibujar landmarks
        self.mp_drawing.draw_landmarks(
            annotated_frame,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=3),
            connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2)
        )

        # Mostrar ángulos y gesto en la imagen
        cv2.putText(annotated_frame, f"Angle L: {angle_left:.1f}°", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(annotated_frame, f"Angle R: {angle_right:.1f}°", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if gesture:
            cv2.putText(annotated_frame, f"Gesto: {gesture}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow("Pose Detection", annotated_frame)
        cv2.waitKey(1)

    def get_xyz(self, lm, idx):
        return np.array([lm[idx].x, lm[idx].y, lm[idx].z])

    def angle(self, A, B, C):
        AB = A - B
        CB = C - B
        dot = np.dot(AB, CB)
        norm = np.linalg.norm(AB) * np.linalg.norm(CB)
        if norm == 0:
            return 180.0
        value = np.clip(dot / norm, -1.0, 1.0)
        return np.degrees(np.arccos(value))


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


if __name__ == '__main__':
    main()
