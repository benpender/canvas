"""
BlazePose WebSocket Streamer (Overlay Version 2.0)

This script mirrors the logic of 'blazepose_overlay.py':
• Captures video
• Runs BlazePose
• Blends Mask (50%) with Video
• Broadcasts via WebSocket
"""

import asyncio
import cv2
import numpy as np
import time
import websockets
import os

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

PORT = 8765
CAMERA_INDEX = 0

STEREO_MODE = True
STEREO_VIEW = "left"   # python overlay script defaults to this
FLIP_VERTICAL = True
FLIP_HORIZONTAL = False

# Resolve model path relative to this script
script_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(script_dir, "pose_landmarker_full.task")

SEGMENTATION_THRESHOLD = 0.6
ENABLE_DILATION = True
DILATION_KERNEL_SIZE = 5
DILATION_ITERATIONS = 1
ENABLE_BLUR = True
BLUR_KERNEL_SIZE = 11
JPEG_QUALITY = 85

# ============================================================

options = PoseLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=RunningMode.VIDEO,
    output_segmentation_masks=True,
    num_poses=1,
)
pose = PoseLandmarker.create_from_options(options)

connected_clients = set()

async def handler(websocket):
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.remove(websocket)

async def broadcast_loop():
    print(f"[INFO] Opening Camera {CAMERA_INDEX}...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[ERROR] Could not open webcam")
        return

    # Request decent resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    start_time = time.time()
    print(f"[INFO] Server starting on ws://localhost:{PORT}")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # 1. Pre-process (Stereo/Flip)
        if STEREO_MODE and STEREO_VIEW != "full":
            half_w = frame.shape[1] // 2
            if STEREO_VIEW == "left":
                frame = frame[:, :half_w]
            elif STEREO_VIEW == "right":
                frame = frame[:, half_w:]

        if FLIP_VERTICAL:
            frame = cv2.flip(frame, 0)
        if FLIP_HORIZONTAL:
            frame = cv2.flip(frame, 1)

        h, w, _ = frame.shape

        # 2. BlazePose
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = Image(image_format=ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((time.time() - start_time) * 1000)
        result = pose.detect_for_video(mp_image, timestamp_ms)

        # 3. Mask Generation
        if result.segmentation_masks:
            mask = result.segmentation_masks[0].numpy_view()
            mask = cv2.resize(mask, (w, h))
            mask = (mask > SEGMENTATION_THRESHOLD).astype(np.uint8) * 255

            if ENABLE_DILATION:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (DILATION_KERNEL_SIZE, DILATION_KERNEL_SIZE))
                mask = cv2.dilate(mask, kernel, iterations=DILATION_ITERATIONS)

            if ENABLE_BLUR:
                mask = cv2.GaussianBlur(mask, (BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE), 0)
        else:
            mask = np.ones((h, w), dtype=np.uint8) * 255

        # Invert Mask (Black=Human)
        mask = cv2.bitwise_not(mask)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # 4. Blend (50% Opacity)
        blended = cv2.addWeighted(frame, 0.5, mask_bgr, 0.5, 0)

        # 5. JPEG Encode
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        _, jpeg_data = cv2.imencode('.jpg', blended, encode_param)
        bytes_data = jpeg_data.tobytes()

        # 6. Broadcast
        if connected_clients:
            for ws in list(connected_clients):
                try:
                    await ws.send(bytes_data)
                except:
                    pass
        
        await asyncio.sleep(0.01)

    cap.release()
    pose.close()

async def main():
    async with websockets.serve(handler, "localhost", PORT):
        await broadcast_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
