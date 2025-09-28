#!/usr/bin/env python3
# encoding: utf-8
import cv2
import mediapipe as mp
#import sdk.fps as fps

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# For webcam input:
cap = cv2.VideoCapture("/dev/astra_camera")
# cap = cv2.VideoCapture("/dev/gemini_camera")
print('\n******Press any key to exit!******')
#fps = fps.FPS()
with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:
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
    results = pose.process(image)
    
    # Draw the pose annotation on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp_drawing.draw_landmarks(
        image,
        results.pose_landmarks,
        mp_pose.POSE_CONNECTIONS)
    #fps.update()
    #result_image = fps.show_fps(cv2.flip(image, 1))
    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
    key = cv2.waitKey(1)
    if key != -1:
        break

cap.release()
cv2.destroyAllWindows()
