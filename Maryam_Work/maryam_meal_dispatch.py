"""Smart MealBot: colour card -> matching tray + matching food area.

STRUCTURE ONLY. No robot movement, no Swift code and NO conveyor belt logic.
This file only decides WHAT should happen; your conveyor code does the rest.

  GREEN  -> green_tray -> green food area
  BLUE   -> blue_tray  -> blue food area
  YELLOW -> no tray. Flashing "ISOLATION PRISONER LOOSE" warning instead.

Classes you call from main_testing.py:
  ColourScanner    opens the laptop webcam and reports the colour held up
  MealDispatcher   turns a detection into a TrayOrder (tray + food area) or an alert
  IsolationAlert   raised when YELLOW is shown

Run this file on its own first to test the camera and the logic:
    python maryam_meal_dispatch.py
Requires: pip install opencv-python numpy

Sketch of use in main_testing.py later:

    from Maryam_Work.maryam_meal_dispatch import ColourScanner, build_default_dispatcher

    dispatcher = build_default_dispatcher(
        tray_objects={"GREEN": green_tray, "BLUE": blue_tray})
    result = dispatcher.handle(ColourScanner().scan())

    if result.action == "SEND_TRAY":
        order = result.order
        order.tray.swift_object      # the tray to put on the conveyor belt
        order.food_area.place_xyz    # the matching coloured food area
        dispatcher.mark_on_conveyor(order)
        # ... your conveyor code runs here ...
        dispatcher.mark_delivered(order)
"""
from __future__ import annotations

import sys
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional, Sequence

import cv2

# ============================================================
# 1. CONFIGURATION  (edit these)
# ============================================================
# Colour -> tray identifier. YELLOW is deliberately NOT a tray colour.
COLOUR_TO_TRAY = {"GREEN": "green_tray", "BLUE": "blue_tray"}
TRAY_COLOURS = tuple(COLOUR_TO_TRAY)

# Colour -> food area identifier and its world XYZ (PLACEHOLDER: fill in later).
COLOUR_TO_FOOD_AREA = {"GREEN": "green_food_area", "BLUE": "blue_food_area"}
FOOD_AREA_XYZ = {"GREEN": None, "BLUE": None}

# Colour -> warning raised instead of a tray being picked.
ALERT_COLOURS = {"YELLOW": "ISOLATION PRISONER LOOSE"}

# HSV ranges (OpenCV hue is 0-179).
HSV_RANGES = {
    "GREEN": ((35, 65, 60), (85, 255, 255)),
    "BLUE": ((90, 65, 60), (135, 255, 255)),
    "YELLOW": ((18, 65, 70), (34, 255, 255)),
}


# ============================================================
# 2. CAMERA: COLOUR SCANNER
# ============================================================
@dataclass(frozen=True)
class DetectionResult:
    """What the scanner saw. colour is None if the person pressed Q."""
    colour: Optional[str]
    coverage: float = 0.0

    @property
    def cancelled(self) -> bool:
        return self.colour is None

    @property
    def is_tray_colour(self) -> bool:
        return self.colour in COLOUR_TO_TRAY

    @property
    def is_alert(self) -> bool:
        return self.colour in ALERT_COLOURS


class ColourScanner:
    """Laptop-webcam scanner. Hold a colour card inside the white box."""

    def __init__(self, camera_index: int = 0, stable_frames: int = 12,
                 min_coverage: float = 0.30,
                 window_title: str = "MealBot - Colour Scanner",
                 alert_display_seconds: float = 2.5):
        self.camera_index = camera_index
        self.stable_frames = stable_frames        # frames that must agree
        self.min_coverage = min_coverage          # share of the centre box
        self.window_title = window_title
        self.alert_display_seconds = alert_display_seconds

    # ---- pure image logic (testable without a camera) ----
    def classify(self, frame):
        """Return (colour or None, coverage of that colour, (x1, y1, x2, y2))."""
        height, width = frame.shape[:2]
        x1, x2 = width // 3, 2 * width // 3       # scan only the centre region
        y1, y2 = height // 3, 2 * height // 3
        hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        coverage = {}
        for name, (low, high) in HSV_RANGES.items():
            mask = cv2.inRange(hsv, low, high)
            coverage[name] = cv2.countNonZero(mask) / mask.size
        best = max(coverage, key=coverage.get)
        colour = best if coverage[best] >= self.min_coverage else None
        return colour, coverage[best], (x1, y1, x2, y2)

    # ---- camera loop ----
    def _open_camera(self):
        backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
        camera = cv2.VideoCapture(self.camera_index, backend)
        if not camera.isOpened():
            raise RuntimeError(
                "Laptop camera unavailable. Close Teams/Zoom or try camera_index=1.")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return camera

    def scan(self) -> DetectionResult:
        """Block until a colour is held steady (or Q is pressed)."""
        camera = self._open_camera()
        cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_title, 400, 300)
        votes = deque(maxlen=self.stable_frames)
        result = DetectionResult(None)
        try:
            while True:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("Webcam opened but no image was received.")
                colour, coverage, box = self.classify(frame)
                votes.append(colour)
                self._draw_overlay(frame, box, colour)
                cv2.imshow(self.window_title, frame)

                if (len(votes) == self.stable_frames and colour is not None
                        and all(v == colour for v in votes)):
                    result = DetectionResult(colour, coverage)
                    print(f"DETECTED: {colour}")
                    break
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if result.is_alert:
                self._show_alert(camera, ALERT_COLOURS[result.colour])
        finally:
            camera.release()
            cv2.destroyAllWindows()
        return result

    # ---- drawing ----
    def _draw_overlay(self, frame, box, colour):
        x1, y1, x2, y2 = box
        height = frame.shape[0]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(frame, f"Scanning: {colour or 'SHOW COLOUR CARD'}", (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(frame, "Hold card in box | Q = cancel", (15, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    @staticmethod
    def _put_centered(frame, text, y, scale, colour, thickness):
        width = frame.shape[1]
        (text_w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX,
                                         scale, thickness)
        cv2.putText(frame, text, (max(5, (width - text_w) // 2), y),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thickness)

    def _show_alert(self, camera, message: str):
        """Flashing red warning on the live camera feed."""
        end = time.monotonic() + self.alert_display_seconds
        while time.monotonic() < end:
            ok, frame = camera.read()
            if not ok:
                break
            height, width = frame.shape[:2]
            if int(time.monotonic() * 4) % 2 == 0:       # flash ~2 Hz
                cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (0, 0, 255), 14)
                cv2.rectangle(frame, (0, height // 2 - 55), (width, height // 2 + 45),
                              (0, 0, 255), -1)
                self._put_centered(frame, "WARNING", height // 2 - 15, 1.0,
                                   (255, 255, 255), 3)
                self._put_centered(frame, message, height // 2 + 28, 0.7,
                                   (255, 255, 255), 2)
            cv2.imshow(self.window_title, frame)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break


def detect_tray_colour(camera_index: int = 0):
    """Backwards-compatible helper: returns 'GREEN', 'BLUE', 'YELLOW' or None."""
    return ColourScanner(camera_index=camera_index).scan().colour


# ============================================================
# 3. ISOLATION ALERT  (yellow card)
# ============================================================
class IsolationAlert:
    """Raised when YELLOW is shown. Add listeners to react in the simulation
    (siren, lock doors, flash something in Swift ...)."""

    MESSAGE = "WARNING: ISOLATION PRISONER LOOSE"

    def __init__(self):
        self.active = False
        self.triggered_at: Optional[float] = None
        self.listeners: list[Callable[["IsolationAlert"], None]] = []

    def trigger(self):
        self.active = True
        self.triggered_at = time.time()
        print(f"\n*** {self.MESSAGE} ***\n", flush=True)
        for listener in self.listeners:
            listener(self)

    def clear(self):
        self.active = False
        self.triggered_at = None


# ============================================================
# 4. TRAYS, FOOD AREAS, ORDERS
# ============================================================
class TrayStatus(Enum):
    WAITING = "waiting"
    ON_CONVEYOR = "on conveyor"
    DELIVERED = "delivered"


@dataclass
class Tray:
    tray_id: str                         # e.g. "green_tray"
    colour: str                          # "GREEN" / "BLUE"
    swift_object: object = None          # the mesh from create_environment()
    status: TrayStatus = TrayStatus.WAITING


@dataclass
class FoodArea:
    area_id: str                         # e.g. "green_food_area"
    colour: str
    place_xyz: Optional[Sequence[float]] = None   # world XYZ of the area


@dataclass
class TrayOrder:
    """What your conveyor code needs: which tray, and where it goes."""
    tray: Tray
    food_area: FoodArea


@dataclass
class DispatchResult:
    action: str                 # SEND_TRAY / ALERT / REJECTED / CANCELLED
    message: str
    order: Optional[TrayOrder] = None


# ============================================================
# 5. DISPATCHER  (the class main_testing.py calls)
# ============================================================
class MealDispatcher:
    """Colour detection in -> tray order or alert out."""

    def __init__(self, trays: dict, food_areas: dict,
                 alert: Optional[IsolationAlert] = None):
        self.trays = trays              # colour -> Tray
        self.food_areas = food_areas    # colour -> FoodArea
        self.alert = alert or IsolationAlert()

    def handle(self, detection: DetectionResult) -> DispatchResult:
        if detection.cancelled:
            return DispatchResult("CANCELLED", "Camera cancelled - no tray selected")

        if detection.is_alert:
            self.alert.trigger()
            return DispatchResult("ALERT", IsolationAlert.MESSAGE)

        if not detection.is_tray_colour:
            return DispatchResult("REJECTED", f"No tray for colour {detection.colour}")

        tray = self.trays.get(detection.colour)
        area = self.food_areas.get(detection.colour)
        if tray is None or area is None:
            return DispatchResult(
                "REJECTED", f"{detection.colour} tray or food area is not configured")
        if tray.status != TrayStatus.WAITING:
            return DispatchResult("REJECTED", f"{tray.tray_id} is already {tray.status.value}")

        message = f"Selected {tray.tray_id} for {area.area_id}"
        print(message, flush=True)
        return DispatchResult("SEND_TRAY", message, TrayOrder(tray, area))

    # Status helpers: call these from your conveyor code.
    def mark_on_conveyor(self, order: TrayOrder):
        order.tray.status = TrayStatus.ON_CONVEYOR

    def mark_delivered(self, order: TrayOrder):
        order.tray.status = TrayStatus.DELIVERED

    def reset(self, colour: str):
        """Make a delivered tray selectable again."""
        self.trays[colour].status = TrayStatus.WAITING

    def status_lines(self) -> list[str]:
        lines = [f"alert: {'ACTIVE' if self.alert.active else 'clear'}"]
        for colour, tray in self.trays.items():
            lines.append(f"{tray.tray_id}: {tray.status.value} -> "
                         f"{self.food_areas[colour].area_id}")
        return lines


def build_default_dispatcher(tray_objects: Optional[dict] = None) -> MealDispatcher:
    """Build trays + food areas from the CONFIGURATION block.

    tray_objects: optional {"GREEN": green_tray_mesh, "BLUE": blue_tray_mesh}
    so each order carries your Swift object.
    """
    tray_objects = tray_objects or {}
    trays = {c: Tray(COLOUR_TO_TRAY[c], c, swift_object=tray_objects.get(c))
             for c in TRAY_COLOURS}
    areas = {c: FoodArea(COLOUR_TO_FOOD_AREA[c], c, FOOD_AREA_XYZ.get(c))
             for c in TRAY_COLOURS}
    return MealDispatcher(trays, areas)


# ============================================================
# 6. STANDALONE TEST (camera + logic only)
# ============================================================
if __name__ == "__main__":
    dispatcher = build_default_dispatcher()
    result = dispatcher.handle(ColourScanner().scan())
    print(f"[{result.action}] {result.message}")
    print("\n".join(dispatcher.status_lines()))