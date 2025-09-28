#!/usr/bin/env python3
# encoding: utf-8
import cv2
import sys
import mediapipe as mp
import sdk.fps as fps

mp_drawing = mp.solutions.drawing_utils
mp_objectron = mp.solutions.objectron

try:
     with mp_objectron.Objectron(static_image_mode=False,
                                max_num_objects=1,
                                min_detection_confidence=0.4,
                                min_tracking_confidence=0.5,
                                model_name='Cup') as objectron:
        # For webcam input:
        cap = cv2.VideoCapture("/dev/astra_camera")
        # cap = cv2.VideoCapture("/dev/gemini_camera")
        print('\n******Press any key to exit!******')
        fps = fps.FPS()
        while cap.isOpened():
            success, image = cap.read()
            if not success:
              print("Ignoring empty camera frame.")
              # If loading a video, use 'break' instead of 'continue'.
              continue

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = objectron.process(image)

            # Draw the box landmarks on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.detected_objects:
                for detected_object in results.detected_objects:
                    mp_drawing.draw_landmarks(
                      image, detected_object.landmarks_2d, mp_objectron.BOX_CONNECTIONS)
                    mp_drawing.draw_axis(image, detected_object.rotation,
                                         detected_object.translation)
            fps.update()
            result_image = fps.show_fps(cv2.flip(image, 1))
            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('MediaPipe Objectron', result_image)
            key = cv2.waitKey(1)
            if key != -1:
                break
except:
    print('***please run workon mediapipe and run this scripts***')
    sys.exit()
cap.release()
cv2.destroyAllWindows()
