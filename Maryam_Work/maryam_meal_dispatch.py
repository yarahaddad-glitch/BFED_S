# MealBot: classify a colour card, then select a matching meal order or an alert.
from __future__ import annotations

import sys
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional, Sequence

import cv2


# Green/blue select meals; yellow is an alert, not a tray order.
COLOUR_TO_TRAY = {"GREEN": "green_tray", "BLUE": "blue_tray"}
TRAY_COLOURS = tuple(COLOUR_TO_TRAY)

# Logical food-area identifiers; XYZ remains unset here. The integrated task uses its own station targets.
COLOUR_TO_FOOD_AREA = {"GREEN": "green_food_area", "BLUE": "blue_food_area"}
FOOD_AREA_XYZ = {"GREEN": None, "BLUE": None}

# Colour -> warning raised instead of a tray being picked.
ALERT_COLOURS = {"YELLOW": "ISOLATION PRISONER LOOSE"}

# SHOW_HSV - Threshold hue, saturation and brightness to build a mask for each colour.
# OpenCV hue uses 0-179. Lighting can still affect detection; these are not learned AI classes.
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
    # Store one completed scan's colour/coverage; None represents cancellation.
    colour: Optional[str]
    coverage: float = 0.0

    # Check whether the scan ended without an order (Q in standalone mode or GUI cancellation).
    @property
    def cancelled(self) -> bool:
        return self.colour is None

    # Check whether the detected colour is configured for a meal tray.
    @property
    def is_tray_colour(self) -> bool:
        return self.colour in COLOUR_TO_TRAY

    # Check whether the colour represents a warning instead of a meal.
    @property
    def is_alert(self) -> bool:
        return self.colour in ALERT_COLOURS


class ColourScanner:
    # Reusable webcam classification and standalone scanning helpers.

    # Require 30% ROI coverage and configure 12 stable frames by default.
    # The final Qt scanner also reads stable_frames when collecting consecutive classifications.
    def __init__(self, camera_index: int = 0, stable_frames: int = 12,
                 min_coverage: float = 0.30,
                 window_title: str = "MealBot - Colour Scanner",
                 alert_display_seconds: float = 2.5):
        self.camera_index = camera_index
        self.stable_frames = max(1, int(stable_frames))
        self.min_coverage = min_coverage
        self.window_title = window_title
        self.alert_display_seconds = alert_display_seconds
        self._camera = None
        self._window_open = False
        self._alert_card_held = None  # avoid retriggering YELLOW continuously

    # ---- pure image logic (testable without a camera) ----
    # SHOW_CLASSIFY - Classify one image; return colour, coverage and detection-box coordinates.
    # Used directly by the final GUI. No webcam access or robot movement happens in this function.
    def classify(self, frame):
        height, width = frame.shape[:2]
        # Inspect the central third in X and Y to reduce background interference.
        x1, x2 = width // 3, 2 * width // 3
        y1, y2 = height // 3, 2 * height // 3
        # Convert OpenCV's BGR image region to hue/saturation/value before thresholding.
        hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        coverage = {}
        for name, (low, high) in HSV_RANGES.items():
            # White pixels match this colour range; coverage is their fraction of the ROI.
            mask = cv2.inRange(hsv, low, high)
            coverage[name] = cv2.countNonZero(mask) / mask.size
        # Accept the strongest colour only if enough of the region matches it.
        best = max(coverage, key=coverage.get)
        colour = best if coverage[best] >= self.min_coverage else None
        return colour, coverage[best], (x1, y1, x2, y2)

    # ---- camera lifetime: open once per scan session ----
    # SHOW_CAMERA - Open/reuse the laptop webcam; report an error if it is unavailable.
    def _open_camera(self):
        if self._camera is not None and self._camera.isOpened():
            return self._camera

        # DirectShow is commonly reliable with OpenCV on Windows.
        backend = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
        camera = cv2.VideoCapture(self.camera_index, backend)
        if not camera.isOpened():
            camera.release()
            raise RuntimeError(
                "Laptop camera unavailable. Close Teams/Zoom or try camera_index=1.")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._camera = camera
        return camera

    # Open the standalone OpenCV display; the integrated GUI uses a Qt camera dialog instead.
    def _open_window(self):
        if not self._window_open:
            cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_title, 400, 300)
            self._window_open = True

    # Release this scanner's capture/window; repeated cleanup calls are allowed.
    def close(self):
        camera, self._camera = self._camera, None
        if camera is not None:
            camera.release()
        if self._window_open:
            try:
                cv2.destroyWindow(self.window_title)
                cv2.waitKey(1)  # let OpenCV process the window-close event
            except cv2.error:
                pass
            self._window_open = False
        self._alert_card_held = None

    # Independent OpenCV loop; the final GUI uses CameraScanner.tick().
    # Read frames, require stable colour, handle Q and prevent repeated held-yellow alerts.
    def scan(self) -> DetectionResult:
        keep_camera_for_next_scan = False
        try:
            camera = self._open_camera()
            self._open_window()
            votes = deque(maxlen=self.stable_frames)
            card_removed_frames = 0
            read_failures = 0

            while True:
                ok, frame = camera.read()
                if not ok:
                    read_failures += 1
                    if read_failures >= 5:
                        raise RuntimeError(
                            "Webcam opened but stopped providing frames. "
                            "Close other camera apps and retry.")
                    cv2.waitKey(30)
                    continue
                read_failures = 0
                colour, coverage, box = self.classify(frame)

                # The main loop calls scan() again immediately on an ALERT.
                # Don't trigger the same YELLOW card repeatedly; require it
                # to leave the white box for a few frames before rearming.
                if self._alert_card_held is not None:
                    votes.clear()
                    if colour == self._alert_card_held:
                        card_removed_frames = 0
                        prompt = "REMOVE YELLOW CARD TO RESET"
                    else:
                        card_removed_frames += 1
                        prompt = "READYING NEXT SCAN..."
                        if card_removed_frames >= 6:
                            self._alert_card_held = None
                            card_removed_frames = 0
                    self._draw_overlay(frame, box, colour, prompt=prompt)
                    cv2.imshow(self.window_title, frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        return DetectionResult(None)
                    continue

                votes.append(colour)
                self._draw_overlay(frame, box, colour)
                cv2.imshow(self.window_title, frame)

                # Poll keys even when the detection becomes stable this frame.
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    return DetectionResult(None)

                # Accept only consecutive identical, non-empty classifications.
                # The final Qt scanner implements its corresponding voting check in camera_scanner.py.
                if (len(votes) == self.stable_frames and colour is not None
                        and all(v == colour for v in votes)):
                    result = DetectionResult(colour, coverage)
                    print(f"DETECTED: {colour}", flush=True)
                    if result.is_alert:
                        self._alert_card_held = colour
                        if self._show_alert(camera, ALERT_COLOURS[colour]):
                            return DetectionResult(None)
                        # The caller will immediately call scan() again on
                        # an ALERT. Keep the existing window/capture alive.
                        keep_camera_for_next_scan = True
                    return result
        finally:
            if not keep_camera_for_next_scan:
                self.close()

    # ---- drawing ----
    # Draw the ROI border and detected-colour prompt on the preview image; reused by the Qt scanner.
    def _draw_overlay(self, frame, box, colour, prompt=None):
        x1, y1, x2, y2 = box
        height = frame.shape[0]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.putText(frame, f"Scanning: {colour or 'SHOW COLOUR CARD'}", (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(frame, prompt or "Hold card in box | Q = cancel",
                    (15, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2)

    # Centre warning text horizontally on an OpenCV frame.
    @staticmethod
    def _put_centered(frame, text, y, scale, colour, thickness):
        width = frame.shape[1]
        (text_w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX,
                                         scale, thickness)
        cv2.putText(frame, text, (max(5, (width - text_w) // 2), y),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thickness)

    # Show the flashing warning in standalone mode; return True if the user presses Q.
    def _show_alert(self, camera, message: str) -> bool:
        end = time.monotonic() + self.alert_display_seconds
        while time.monotonic() < end:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Webcam stopped providing frames during alert.")
            height, width = frame.shape[:2]
            if int(time.monotonic() * 4) % 2 == 0:
                cv2.rectangle(frame, (0, 0), (width - 1, height - 1),
                              (0, 0, 255), 14)
                cv2.rectangle(frame,
                              (0, height // 2 - 55), (width, height // 2 + 45),
                              (0, 0, 255), -1)
                self._put_centered(frame, "WARNING", height // 2 - 15, 1.0,
                                   (255, 255, 255), 3)
                self._put_centered(frame, message, height // 2 + 28, 0.7,
                                   (255, 255, 255), 2)
            cv2.imshow(self.window_title, frame)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                return True
        return False


# Compatibility helper for standalone callers that need only the colour string.
def detect_tray_colour(camera_index: int = 0):
    scanner = ColourScanner(camera_index=camera_index)
    try:
        return scanner.scan().colour
    finally:
        scanner.close()


# ============================================================
# 3. ISOLATION ALERT  (yellow card)
# ============================================================
class IsolationAlert:
    # Logical yellow-card warning. It does not by itself command a physical stop or lock doors.

    MESSAGE = "WARNING: ISOLATION PRISONER LOOSE"

    # Initialise warning state, timestamp and optional functions to notify on an alert.
    def __init__(self):
        self.active = False
        self.triggered_at: Optional[float] = None
        self.listeners: list[Callable[["IsolationAlert"], None]] = []

    # SHOW_ALERT - Record/print the warning and call any registered listeners; no tray is sent.
    def trigger(self):
        self.active = True
        self.triggered_at = time.time()
        print(f"\n*** {self.MESSAGE} ***\n", flush=True)
        for listener in self.listeners:
            listener(self)

    # Clear the logical warning state and its timestamp.
    def clear(self):
        self.active = False
        self.triggered_at = None



# 4. TRAYS, FOOD AREAS, ORDERS
class TrayStatus(Enum):
    # Named logical order states, separate from the remaining physical meshes in TrayStacks.
    WAITING = "waiting"
    ON_CONVEYOR = "on conveyor"
    DELIVERED = "delivered"


@dataclass
class Tray:
    # Logical tray selection/status; an actual Swift mesh reference can optionally be supplied.
    tray_id: str                         # e.g. "green_tray"
    colour: str                          # "GREEN" / "BLUE"
    swift_object: object = None          # the mesh from create_environment()
    status: TrayStatus = TrayStatus.WAITING


@dataclass
class FoodArea:
    # Logical matching food area; this record does not move food or robots.
    area_id: str                         # e.g. "green_food_area"
    colour: str
    place_xyz: Optional[Sequence[float]] = None   # world XYZ of the area


@dataclass
class TrayOrder:
    # Bundle the selected logical tray and its matching food area.
    tray: Tray
    food_area: FoodArea


@dataclass
class DispatchResult:
    # Return the decision, message and optional order to main_all.run_orders().
    action: str                 # SEND_TRAY / ALERT / REJECTED / CANCELLED
    message: str
    order: Optional[TrayOrder] = None


class MealDispatcher:
    # Convert a completed detection into a task decision, without moving anything.

    # Store the available colour mappings and the alert object.
    def __init__(self, trays: dict, food_areas: dict,
                 alert: Optional[IsolationAlert] = None):
        self.trays = trays              # colour -> Tray
        self.food_areas = food_areas    # colour -> FoodArea
        self.alert = alert or IsolationAlert()

    # SHOW_DISPATCH - Decide CANCELLED, ALERT, REJECTED or SEND_TRAY from a completed scan.
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
        # Do not issue another order for a colour whose logical tray is already busy.
        if tray.status != TrayStatus.WAITING:
            return DispatchResult("REJECTED", f"{tray.tray_id} is already {tray.status.value}")

        message = f"Selected {tray.tray_id} for {area.area_id}"
        print(message, flush=True)
        # Main receives this order and calls IntegratedMeal.serve() with its colour.
        # IntegratedMeal selects the real next tray mesh from TrayStacks.
        return DispatchResult("SEND_TRAY", message, TrayOrder(tray, area))

    # Status helpers: call these from your conveyor code.
    # The movement code calls this after a successful deposit onto the conveyor.
    def mark_on_conveyor(self, order: TrayOrder):
        order.tray.status = TrayStatus.ON_CONVEYOR

    # Mark the logical order complete after the serving task succeeds.
    def mark_delivered(self, order: TrayOrder):
        order.tray.status = TrayStatus.DELIVERED

    # Allow another order of this colour; this does not create or replenish physical trays.
    # This order reset is unrelated to the GUI emergency-stop reset.
    def reset(self, colour: str):
        self.trays[colour].status = TrayStatus.WAITING

    # Produce readable order/alert status lines for debugging or standalone output.
    def status_lines(self) -> list[str]:
        lines = [f"alert: {'ACTIVE' if self.alert.active else 'clear'}"]
        for colour, tray in self.trays.items():
            lines.append(f"{tray.tray_id}: {tray.status.value} -> "
                         f"{self.food_areas[colour].area_id}")
        return lines


# Build the logical green/blue tray and food-area mappings used by main_all.
def build_default_dispatcher(tray_objects: Optional[dict] = None) -> MealDispatcher:
    tray_objects = tray_objects or {}
    trays = {c: Tray(COLOUR_TO_TRAY[c], c, swift_object=tray_objects.get(c))
             for c in TRAY_COLOURS}
    areas = {c: FoodArea(COLOUR_TO_FOOD_AREA[c], c, FOOD_AREA_XYZ.get(c))
             for c in TRAY_COLOURS}
    return MealDispatcher(trays, areas)



# This test runs only when this file is launched directly, not when main_all imports it.
if __name__ == "__main__":
    dispatcher = build_default_dispatcher()
    scanner = ColourScanner()
    try:
        result = dispatcher.handle(scanner.scan())
    finally:
        scanner.close()
    print(f"[{result.action}] {result.message}")
    print("\n".join(dispatcher.status_lines()))
