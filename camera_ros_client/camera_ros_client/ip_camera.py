import cv2
import numpy as np

cap = cv2.VideoCapture("rtsp://admin:123@rplcam1.cels.anl.gov:8554/profile0")

while True:
    ret, frame = cap.read()
    print(frame)
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
