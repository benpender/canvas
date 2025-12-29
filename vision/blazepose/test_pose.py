import cv2
import mediapipe as mp

cap = cv2.VideoCapture(0)
pose = mp.solutions.pose.Pose(
    enable_segmentation=True,
    model_complexity=1
)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb)

    if results.segmentation_mask is not None:
        mask = (results.segmentation_mask > 0.5).astype('uint8') * 255
        cv2.imshow("BlazePose Mask", mask)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()

