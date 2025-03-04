import cv2

def detect_objects(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    objects = cv2.detectMultiScale(gray, 1.1, 4)
    return objects
