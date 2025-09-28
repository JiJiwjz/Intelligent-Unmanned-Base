#!/usr/bin/env python3
# encoding: utf-8
import cv2
import mediapipe as mp
import sdk.fps as fps

mp_drawing = mp.solutions.drawing_utils
mp_face_mesh = mp.solutions.face_mesh

# For webcam input:
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
cap = cv2.VideoCapture("/dev/astra_camera")
# cap = cv2.VideoCapture("/dev/gemini_camera")
print('\n******Press any key to exit!******')
fps = fps.FPS()
with mp_face_mesh.FaceMesh(
    max_num_faces=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as face_mesh:
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
    results = face_mesh.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # Draw the face mesh annotations on the image.
    image.flags.writeable = True
    if results.multi_face_landmarks:
      for face_landmarks in results.multi_face_landmarks:
         mp_drawing.draw_landmarks(
            image=image,
            landmark_list=face_landmarks,
            landmark_drawing_spec=drawing_spec)
    fps.update()

    result_image = fps.show_fps(cv2.flip(image, 1))
    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Face Mesh', result_image)
    key = cv2.waitKey(1)
    if key != -1: 
        break

cap.release()
cv2.destroyAllWindows()
