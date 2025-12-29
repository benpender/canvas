import cv2
import numpy as np
import time

# TO ACTIVATE AND RUN:
# source venv/bin/activate
# python test_pose.py

from mediapipe import Image, ImageFormat
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    PoseLandmarker,
    PoseLandmarkerOptions,
    RunningMode,
)

# ---------------------------------------------------------------------
# CONFIGURATION (EDIT ONLY THIS SECTION)
# ---------------------------------------------------------------------

# Camera
CAMERA_INDEX = 0

# Stereo camera options
STEREO_MODE = True        # True if camera outputs side-by-side
STEREO_VIEW = "left"     # "left", "right", or "full"

# Orientation fixes
FLIP_VERTICAL = True     # FIX upside-down camera (default ON)
FLIP_HORIZONTAL = False # Mirror image (optional, usually OFF)

# BlazePose
MODEL_PATH = "pose_landmarker_full.task"
SEGMENTATION_THRESHOLD = 0.5

# ---------------------------------------------------------------------
# CREATE BLAZEPOSE LANDMARKER
# ---------------------------------------------------------------------

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=RunningMode.VIDEO,
    output_segmentation_masks=True,
    num_poses=1,
)

pose = PoseLandmarker.create_from_options(options)

# ---------------------------------------------------------------------
# OPEN WEBCAM
# ---------------------------------------------------------------------

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

start_time = time.time()

# ---------------------------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------------------------

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # ---------------------------------------------------------------
    # Stereo cropping (side-by-side cameras)
    # ---------------------------------------------------------------

    h, w, _ = frame.shape

    if STEREO_MODE and STEREO_VIEW != "full":
        half_w = w // 2
        if STEREO_VIEW == "left":
            frame = frame[:, :half_w]
        elif STEREO_VIEW == "right":
            frame = frame[:, half_w:]

    # ---------------------------------------------------------------
    # Orientation fixes
    # ---------------------------------------------------------------

    if FLIP_VERTICAL:
        frame = cv2.flip(frame, 0)   # upside-down fix

    if FLIP_HORIZONTAL:
        frame = cv2.flip(frame, 1)   # mirror left/right

    # ---------------------------------------------------------------
    # Convert to MediaPipe image
    # ---------------------------------------------------------------

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    mp_image = Image(
        image_format=ImageFormat.SRGB,
        data=rgb
    )

    timestamp_ms = int((time.time() - start_time) * 1000)

    # ---------------------------------------------------------------
    # Run BlazePose
    # ---------------------------------------------------------------

    result = pose.detect_for_video(mp_image, timestamp_ms)

    # ---------------------------------------------------------------
    # Segmentation mask output (DEBUG WINDOW)
    # ---------------------------------------------------------------

    if result.segmentation_masks:
        mask = result.segmentation_masks[0].numpy_view()

        # Threshold → binary silhouette
        mask = (mask > SEGMENTATION_THRESHOLD).astype(np.uint8) * 255

        # Resize mask to match processed frame
        mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]))

        # Debug overlay
        cv2.putText(
            mask,
            f"Stereo: {STEREO_VIEW} | FlipV: {FLIP_VERTICAL} | FlipH: {FLIP_HORIZONTAL}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            255,
            2
        )

        cv2.imshow("BlazePose Segmentation Mask", mask)

    # ESC to quit
    if cv2.waitKey(1) & 0xFF == 27:
        break

# ---------------------------------------------------------------------
# CLEANUP
# ---------------------------------------------------------------------

cap.release()
cv2.destroyAllWindows()
pose.close()
