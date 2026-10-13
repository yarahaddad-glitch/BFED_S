"""Laptop webcam colour selector for the Smart MealBot simulation.
Run this file separately first; integrate detect_tray_colour() later.
Requires: pip install opencv-python numpy
"""
import cv2
import numpy as np

# Each colour corresponds to a tray identifier, not a robot coordinate yet.
COLOUR_TO_TRAY = {"GREEN": "green_tray", "BLUE": "blue_tray", "YELLOW": "yellow_tray"}


def detect_tray_colour(camera_index=0):
    camera = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)  # Windows laptop webcam
    if not camera.isOpened():
        raise RuntimeError("Laptop camera unavailable. Close Teams/Zoom or try camera_index=1.")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    title = "MealBot - Colour Scanner"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(title, 400, 300)  # Smaller camera window; drag it to a screen corner.
    votes = []
    selected = None

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Webcam opened but no image was received.")

            height, width = frame.shape[:2]
            x1, x2 = width // 3, 2 * width // 3  # Scan only the centre region.
            y1, y2 = height // 3, 2 * height // 3
            roi = frame[y1:y2, x1:x2]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            masks = {
                "GREEN": cv2.inRange(hsv, (35, 65, 60), (85, 255, 255)),
                "BLUE": cv2.inRange(hsv, (90, 65, 60), (135, 255, 255)),
                "YELLOW": cv2.inRange(hsv, (18, 65, 70), (34, 255, 255)),
            }
            coverage = {name: cv2.countNonZero(mask) / mask.size for name, mask in masks.items()}
            best = max(coverage, key=coverage.get)
            current = best if coverage[best] >= 0.30 else None  # Require 30% of centre region.
            votes.append(current)
            votes = votes[-12:]  # Require a stable reading across 12 camera frames.

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
            cv2.putText(frame, f"Scanning: {current or 'SHOW COLOUR CARD'}", (15, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(frame, "Hold card in box | Q = cancel", (15, height - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.imshow(title, frame)

            if len(votes) == 12 and current is not None and all(v == current for v in votes):
                selected = current
                print(f"DETECTED: {selected} -> {COLOUR_TO_TRAY[selected]}")
                break
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()

    return selected


if __name__ == "__main__":
    colour = detect_tray_colour()
    print("Selected tray:", COLOUR_TO_TRAY[colour] if colour else "None")
