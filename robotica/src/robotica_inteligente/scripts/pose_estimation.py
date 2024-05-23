#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import ackermann_msgs.msg
import numpy as np


# TODO Declarar el detector de pose de mediapipe a utilizar
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)

# Publicador de mensajes de control
ackermann_command_publisher = None

#Procesar la imagen del operador
def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convertir la imagen de ROS a una imagen de OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # TODO Procesar la imagen con MediaPipe
    image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)

    # TODO Reconocer el gesto mediante alguna clasificación a partir de los landmarks
    steering_angle = 0.0
    speed = 0.0

    if results.multi_hand_landmarks and results.multi_handedness:
        for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            hand_label = handedness.classification[0].label

            if hand_label == "Right":
                    # Calcular apertura de la mano de forma progresiva
                    fingers_open = 0
                    total_fingers = 4
                    for tip, mcp in zip(
                        ["INDEX_FINGER_TIP", "MIDDLE_FINGER_TIP", "RING_FINGER_TIP", "PINKY_TIP"],
                        ["INDEX_FINGER_MCP", "MIDDLE_FINGER_MCP", "RING_FINGER_MCP", "PINKY_MCP"]
                    ):
                        if hand_landmarks.landmark[mp_hands.HandLandmark[tip]].y < hand_landmarks.landmark[mp_hands.HandLandmark[mcp]].y:
                            fingers_open += 1

                    hand_open_ratio = fingers_open / total_fingers
                    
                    # Normalizar a [-1, 1]
                    speed = np.clip(hand_open_ratio * 2 - 1, -1.0, 1.0)

            elif hand_label == "Left":
                # Mano izquierda: determinar ángulo de giro progresivo
                thumb_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x
                index_finger_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x

                # Calcular la distancia horizontal entre el pulgar y el índice
                distance = thumb_tip_x - index_finger_tip_x

                # Normalizar la distancia al rango [-1, 1]
                steering_angle = np.clip(distance * 5, -1.0, 1.0)

            # Dibujar landmarks en la imagen
            for landmark in hand_landmarks.landmark:
                height, width, _ = cv_image.shape
                cx, cy = int(landmark.x * width), int(landmark.y * height)
                cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)

    # Crear y enviar mensaje de control Ackermann
    ackermann_cmd = ackermann_msgs.msg.AckermannDrive()
    ackermann_cmd.speed = speed
    ackermann_cmd.steering_angle = steering_angle

    # Publicar el mensaje de control Ackermann
    ackermann_command_publisher.publish(ackermann_cmd)
    
    # TODO Dibujar los landsmarks sobre la imagen
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp.solutions.drawing_utils.draw_landmarks(
                cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    
    # Mostrar la imagen con los landmarks/gestos detectados
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)

    


def main():
    global ackermann_command_publisher

    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
