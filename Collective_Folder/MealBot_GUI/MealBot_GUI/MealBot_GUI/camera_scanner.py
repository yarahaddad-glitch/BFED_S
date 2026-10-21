"""Qt camera popup, using the existing team's HSV colour classifier."""
from collections import deque
from time import monotonic

import cv2
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QDialog, QLabel, QPushButton, QVBoxLayout

from Maryam_Work.maryam_meal_dispatch import ColourScanner, DetectionResult


class CameraScanner(QDialog):
    def __init__(self, runtime, parent):
        super().__init__(parent)
        self.runtime = runtime
        self.setWindowTitle('MealBot colour scanner')
        self.classifier = ColourScanner()
        self.capture = None
        self.votes = deque(maxlen=self.classifier.stable_frames)
        self.yellow_held = False
        self.removed = 0
        self.failures = 0
        self.pending_result = None
        self.result_ready_at = 0.0
        layout = QVBoxLayout(self)
        self.identified = QLabel('Show a colour card')
        self.identified.setStyleSheet('font-size: 16px; font-weight: bold;')
        layout.addWidget(self.identified)
        self.preview = QLabel('Waiting for Start')
        self.preview.setMinimumSize(400, 300)
        self.preview.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.preview)
        self.prompt = QLabel('Hold a green or blue card inside the white box.')
        layout.addWidget(self.prompt)
        cancel = QPushButton('Finish scanning')
        cancel.clicked.connect(self.cancel)
        layout.addWidget(cancel)
        self.timer = QTimer(self)
        self.timer.setInterval(40)
        self.timer.timeout.connect(self.tick)
        self.timer.start()

    def release(self):
        self.pending_result = None
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self.hide()

    def cancel(self):
        self.runtime.finish_scan(DetectionResult(None))
        self.release()

    def closeEvent(self, event):
        self.cancel()
        event.accept()

    def reject(self):
        self.cancel()

    def show_identified(self, result):
        self.identified.setText(f'Colour identified - {result.colour.capitalize()}')
        self.prompt.setText('Identified')
        self.pending_result = result
        self.result_ready_at = monotonic() + 1.0

    def tick(self):
        if self.runtime.closed:
            self.release()
            self.timer.stop()
            return
        if not self.runtime.scan_requested.is_set():
            self.release()
            return
        if self.runtime.state != 'RUNNING':
            self.prompt.setText('Paused')
            self.votes.clear()
            return
        if self.pending_result is not None:
            if monotonic() >= self.result_ready_at:
                result = self.pending_result
                self.runtime.finish_scan(result)
                self.release()
            return
        try:
            if self.capture is None:
                self.capture = self.classifier._open_camera()
                self.votes.clear()
                self.failures = 0
                self.identified.setText('Show a colour card')
                self.show()
            ok, frame = self.capture.read()
            if not ok:
                self.failures += 1
                if self.failures >= 5:
                    raise RuntimeError('Camera stopped providing frames')
                return
            self.failures = 0
            colour, coverage, box = self.classifier.classify(frame)
            self.classifier._draw_overlay(frame, box, colour)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width, _ = rgb.shape
            image = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888).copy()
            self.preview.setPixmap(QPixmap.fromImage(image).scaled(
                480, 360, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            self.prompt.setText('Hold GREEN or BLUE in the box. YELLOW raises an alert.')
            if self.yellow_held:
                self.removed = self.removed + 1 if colour != 'YELLOW' else 0
                self.prompt.setText('YELLOW alert: remove the yellow card before continuing.')
                if self.removed >= 6:
                    self.yellow_held = False
                return
            self.votes.append(colour)
            if (colour is not None and len(self.votes) == self.votes.maxlen
                    and all(v == colour for v in self.votes)):
                self.yellow_held = colour == 'YELLOW'
                self.removed = 0
                self.show_identified(DetectionResult(colour, coverage))
                self.votes.clear()
        except Exception as exc:
            self.runtime.finish_scan(error=str(exc))
            self.release()
