"""Scale the complete interactive panel, including text and widget geometry."""
from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter
from PySide6.QtWidgets import QFrame, QGraphicsScene, QGraphicsView


class ScaledPanel(QGraphicsView):
    def __init__(self, panel, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.NoFrame)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self.panel = panel
        panel.setFixedSize(490, 560)
        scene = QGraphicsScene(self)
        scene.addWidget(panel)
        scene.setSceneRect(0, 0, 490, 560)
        self.setScene(scene)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
