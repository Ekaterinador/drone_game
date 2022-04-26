#!/usr/bin/env python3
from geometry_msgs import msg
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
import tf
import numpy as np
import random
from std_msgs.msg import String, Int8
import time
import serial

import cv2
import mediapipe as mp
import numpy as np

pub = rospy.Publisher('/shot', Int8, queue_size=10)

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

coord = tuple(np.multiply(
                np.array((0.5, 0.5)),
                [640, 480]).astype(int))

if __name__ == '__main__':
    print('start')
    cap = cv2.VideoCapture(4)
    rospy.init_node('camera', anonymous=True)

    # rate = rospy.Rate(10)
    # Initiate holistic model
    with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
        while cap.isOpened():
            shot = 0
            ret, frame = cap.read()

            # Recolor Feed
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Make Detections
            results = holistic.process(image)

            # Recolor image back to BGR for rendering
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Right hand
            mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)

            # Left Hand
            mp_drawing.draw_landmarks(image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)

            coord = coords = tuple(np.multiply(
                np.array((0.5, 0.5)),
                [640, 480]).astype(int))

            try:
                thumb = results.right_hand_landmarks.landmark[4]
                ind_finger = results.right_hand_landmarks.landmark[8]

                dist = ((thumb.x - ind_finger.x) ** 2 + (thumb.y - ind_finger.y) ** 2 + (
                            thumb.z - ind_finger.z) ** 2) ** 0.5

                if dist > 0.1:
                    print('SHOT')
                    cv2.putText(image, 'SHOT', coord, cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 2, cv2.LINE_AA)
                    shot = 1
            except:
                pass

            cv2.imshow('Raw Webcam Feed', image)
            pub.publish(shot)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()



