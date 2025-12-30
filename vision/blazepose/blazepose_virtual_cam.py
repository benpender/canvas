"""
BlazePose full-body segmentation → Virtual Webcam (OBS)

macOS-correct version.

This script:
• Handles stereo side-by-side cameras
• Fixes upside-down cameras
• Runs BlazePose segmentation locally (Python)
• Applies configurable mask smoothing
• Outputs a silhouette mask as OBS Virtual Camera

IMPORTANT:
• OBS must be running
• OBS Virtual Camera must be started
• OBS scene should be EMPTY
"""

import cv2
import numpy as np
import time
import pyvirtualcam

from mediapipe import Image, ImageFormat
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    PoseLandmarker,
    PoseLandmarkerOptions,
    RunningMode,
)

# ============================================================
# ======================= CONFIG =============================
# ============================================================

CAMERA_INDEX = 0

# Stereo camera support
STEREO_MODE = True
STEREO_VIEW = "left"   # "left", "right", or "full"

# Orientation fixes
FLIP_VERTICAL = True
FLIP_HORIZONTAL = False

# BlazePose model
MODEL_PATH = "pose_landmarker_full.task"

# Segmentation tuning
SEGMENTATION_THRESHOLD = 0.6

# Mask shaping
ENABLE_DILATION = True
DILATION_KERNEL_SIZE = 5
DILATION_ITERATIONS = 1

ENABLE_BLUR = True
BLUR_KERNEL_SIZE = 11   # must be odd

# Virtual camera
VIRTUAL_CAM_FPS = 30

# ============================================================
# ================== END CONFIG ==============================
# ============================================================

# ------------------------------------------------------------
# Create BlazePose landmarker
# ------------------------------------------------------------

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=RunningMode.VIDEO,
    output_segmentation_masks=True,
    num_poses=1,
)

pose = PoseLandmarker.create_from_options(options)

# ------------------------------------------------------------
# Open physical webcam
# ------------------------------------------------------------

cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

# Read first frame to determine output size
ret, frame = cap.read()
if not ret:
    raise RuntimeError("Could not read initial frame")

# Apply stereo crop once for sizing
if STEREO_MODE and STEREO_VIEW != "full":
    half_w = frame.shape[1] // 2
    if STEREO_VIEW == "left":
        frame = frame[:, :half_w]
    elif STEREO_VIEW == "right":
        frame = frame[:, half_w:]

# Orientation fixes
if FLIP_VERTICAL:
    frame = cv2.flip(frame, 0)
if FLIP_HORIZONTAL:
    frame = cv2.flip(frame, 1)

h, w, _ = frame.shape

# ------------------------------------------------------------
# Open virtual camera (macOS-correct: BGR)
# ------------------------------------------------------------

virtual_cam = pyvirtualcam.Camera(
    width=w,
    height=h,
    fps=VIRTUAL_CAM_FPS,
    fmt=pyvirtualcam.PixelFormat.BGR,
    print_fps=True,
)

print("[INFO] Virtual camera started:", virtual_cam.device)
print(f"[INFO] Output resolution: {w} x {h}")

start_time = time.time()

# ------------------------------------------------------------
# Main loop
# ------------------------------------------------------------

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Stereo crop
    if STEREO_MODE and STEREO_VIEW != "full":
        half_w = frame.shape[1] // 2
        if STEREO_VIEW == "left":
            frame = frame[:, :half_w]
        elif STEREO_VIEW == "right":
            frame = frame[:, half_w:]

    # Orientation fixes
    if FLIP_VERTICAL:
        frame = cv2.flip(frame, 0)
    if FLIP_HORIZONTAL:
        frame = cv2.flip(frame, 1)

    # BlazePose inference
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    mp_image = Image(
        image_format=ImageFormat.SRGB,
        data=rgb
    )

    timestamp_ms = int((time.time() - start_time) * 1000)
    result = pose.detect_for_video(mp_image, timestamp_ms)

    # Build segmentation mask
    if result.segmentation_masks:
        mask = result.segmentation_masks[0].numpy_view()
        mask = cv2.resize(mask, (w, h))
        mask = (mask > SEGMENTATION_THRESHOLD).astype(np.uint8) * 255

        if ENABLE_DILATION:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (DILATION_KERNEL_SIZE, DILATION_KERNEL_SIZE)
            )
            mask = cv2.dilate(mask, kernel, iterations=DILATION_ITERATIONS)

        if ENABLE_BLUR:
            mask = cv2.GaussianBlur(
                mask,
                (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE),
                0
            )
    else:
        mask = np.zeros((h, w), dtype=np.uint8)

    # --------------------------------------------------------
    # IMPORTANT: convert GRAY → BGR (macOS / OBS requirement)
    # --------------------------------------------------------

    frame_out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Send to virtual camera
    virtual_cam.send(frame_out)
    virtual_cam.sleep_until_next_frame()

# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------

virtual_cam.close()
cap.release()
pose.close()
