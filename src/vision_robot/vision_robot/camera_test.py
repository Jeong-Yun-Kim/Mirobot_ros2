import cv2
cap = cv2.VideoCapture(4, cv2.CAP_V4L2)

if not cap.isOpened():
    print("camera open fail")
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("frame read fail")
        break

    cv2.imshow("camera test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
