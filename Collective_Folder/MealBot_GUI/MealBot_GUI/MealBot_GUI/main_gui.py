"""Smart MealBot -- compact simulation-only GUI with robot camera focus.

Standalone mode: run this file beside robot_bridge.py, system_manager.py,
manual_mode.py and theme.py. Joint jogging remains in the existing ManualMode.

When integrated into the SAME Python process as a running Swift scene, attach
that scene and its real robot objects to enable camera following:

    window = MainWindow()
    window.attach_swift_scene(env, {
        "Maryam": maryam_robot,
        "Yara": yara_robot,
        "Hannah": hannah_robot,
    })

This camera feature only moves the Swift VIEWER; it does not read camera
pixels, create another Swift instance, or control the robots or real hardware.
"""

import sys

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QInputDialog, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QStackedWidget,
    QVBoxLayout, QWidget,
)

from robot_bridge import RobotBridge
from system_manager import SystemManager
from manual_mode import ManualMode
from theme import apply_theme


# Builds on your existing purple theme without changing ManualMode.
COMPACT_STYLE = """
QPushButton#navButton {
    font-size: 10px;
    padding: 1px 4px;
    min-height: 19px;
    max-height: 19px;
    text-align: left;
}
QPushButton#stop {
    font-size: 10px;
    font-weight: bold;
    padding: 2px;
    min-height: 26px;
    max-height: 26px;
}
QLabel#brand { font-size: 11px; font-weight: 800; }
QLabel#stateLabel { font-size: 10px; padding: 2px 3px; }
QLabel#heading { font-size: 13px; font-weight: bold; }
QLabel#dashboardSubtitle { color: #81669b; font-size: 9px; }
QFrame#overviewRow {
    background: #f2ebff;
    border: 1px solid #dac6f9;
    border-radius: 6px;
}
QLabel#metricCaption { color: #684c85; font-size: 10px; font-weight: 600; }
QLabel#metricValue { color: #48216f; font-size: 10px; font-weight: bold; }
QPushButton#cameraRobotButton {
    background: #f4eaff;
    color: #48216f;
    border: 1px solid #c5a6eb;
    border-radius: 5px;
    font-size: 10px;
    font-weight: bold;
    padding: 2px 3px;
    min-height: 25px;
}
QPushButton#cameraRobotButton[active="true"] {
    background: #8551c2;
    color: #ffffff;
    border-color: #8551c2;
}
QLabel#cameraStatus {
    color: #654881;
    background: #f6f0ff;
    border: 1px solid #d7c3f3;
    border-radius: 5px;
    font-size: 9px;
    padding: 3px;
}
QPushButton#cameraAction {
    font-size: 10px;
    padding: 2px;
    min-height: 22px;
}
"""


# The focus distance is relative to each robot's actual world coordinates,
# not a hard-coded workcell centre. Feel free to adjust these per robot.
CAMERA_OFFSETS = {
    "Maryam": (1.7, -1.8, 1.2),
    "Yara": (1.5, -1.7, 1.1),
    "Hannah": (1.5, -1.7, 1.1),
}


def _xyz(pose):
    """Return x/y/z from an SE3 pose, numpy vector, or 3-sequence."""
    if hasattr(pose, "t"):
        pose = pose.t
    elif hasattr(pose, "A"):
        pose = pose.A
        return [float(pose[i, 3]) for i in range(3)]
    return [float(pose[i]) for i in range(3)]


def _camera_pose_for_robot(robot, name):
    """Compute a viewpoint centred between the robot base and end effector.

    Accepts standard Robotics Toolbox DH/ERobot models. Also accepts a Swift
    handle with a .robot or .model reference and a .q joint-state vector.
    """
    model = robot
    if not callable(getattr(model, "fkine", None)):
        model = getattr(robot, "robot", None) or getattr(robot, "model", None)
    if model is None or not callable(getattr(model, "fkine", None)):
        raise ValueError("Robot must provide fkine(q), or a model/robot with fkine(q).")

    base_pose = getattr(model, "base", None)
    if base_pose is None:
        raise ValueError("Robot model has no world base pose for camera focus.")
    base = _xyz(base_pose)

    q = getattr(robot, "q", None)
    if q is None:
        q = getattr(model, "q", None)
    if q is None:
        raise ValueError("Robot has no live joint configuration q.")
    tip = _xyz(model.fkine(q))

    # Tracking the arm centre produces less camera jitter than locking to the
    # gripper alone while still following the movement of the full arm.
    look_at = [0.45 * base[i] + 0.55 * tip[i] for i in range(3)]
    look_at[2] = max(look_at[2], base[2] + 0.25)
    offset = CAMERA_OFFSETS[name]
    position = [look_at[i] + offset[i] for i in range(3)]
    return position, look_at


class SimplePage(QWidget):
    """Small information panel for modules that are not integrated yet."""

    def __init__(self, title, description):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(7)
        heading = QLabel(title)
        heading.setObjectName("heading")
        layout.addWidget(heading)
        detail = QLabel(description)
        detail.setWordWrap(True)
        layout.addWidget(detail)
        layout.addStretch()


def compact_scroll(page):
    """Keep the existing ManualMode accessible in the small GUI footprint."""
    wrapper = QScrollArea()
    wrapper.setFrameShape(QFrame.Shape.NoFrame)
    wrapper.setWidgetResizable(True)
    wrapper.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
    wrapper.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
    wrapper.setWidget(page)
    wrapper.setMinimumSize(0, 0)
    return wrapper


def status_row(name, value, tooltip=""):
    frame = QFrame()
    frame.setObjectName("overviewRow")
    frame.setFixedHeight(23)
    frame.setToolTip(tooltip)
    line = QHBoxLayout(frame)
    line.setContentsMargins(7, 2, 7, 2)
    line.setSpacing(3)
    caption = QLabel(name)
    caption.setObjectName("metricCaption")
    line.addWidget(caption)
    line.addStretch(1)
    value_label = QLabel(value)
    value_label.setObjectName("metricValue")
    value_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    line.addWidget(value_label)
    return frame, value_label


class MainWindow(QMainWindow):
    """Small top-left overlay for a single shared Swift scene."""

    def __init__(self, env=None, robots=None):
        super().__init__()
        self.setWindowTitle("Smart MealBot | Compact Controls")
        self.resize(365, 270)
        self.setMinimumSize(345, 255)

        self.bridge = RobotBridge()        # No changes to working joint code.
        self.system = SystemManager()     # Original simulation latch retained.

        # Camera binding is explicit: there is no fake video/camera feed.
        self.swift_env = None
        self.swift_robots = {}
        self.camera_selected = None
        self.camera_following = False
        self.camera_timer = QTimer(self)
        self.camera_timer.setInterval(180)  # ~5.5 view adjustments/second.
        self.camera_timer.timeout.connect(self.update_camera)

        root = QWidget()
        self.setCentralWidget(root)
        overall = QHBoxLayout(root)
        overall.setContentsMargins(4, 4, 4, 4)
        overall.setSpacing(5)

        sidebar = QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(101)
        overall.addWidget(sidebar)
        left = QVBoxLayout(sidebar)
        left.setContentsMargins(5, 6, 5, 6)
        left.setSpacing(1)

        brand = QLabel("🤖 MEALBOT")
        brand.setObjectName("brand")
        left.addWidget(brand)
        left.addSpacing(3)

        surface = QWidget()
        surface.setObjectName("pageSurface")
        overall.addWidget(surface, 1)
        content = QVBoxLayout(surface)
        content.setContentsMargins(5, 5, 5, 5)
        content.setSpacing(0)
        self.pages = QStackedWidget()
        self.pages.setMinimumSize(0, 0)
        content.addWidget(self.pages)

        self.pages.addWidget(self.make_dashboard())
        self.pages.addWidget(compact_scroll(self.make_automatic_page()))
        self.pages.addWidget(compact_scroll(ManualMode(self.bridge, self.system)))
        self.pages.addWidget(compact_scroll(self.make_collision_page()))
        self.pages.addWidget(compact_scroll(self.make_recovery_page()))
        self.pages.addWidget(compact_scroll(self.make_plc_page()))
        self.pages.addWidget(self.make_camera_page())  # REPLACES Event Log.

        self.nav_buttons = []
        for index, name in enumerate((
            "⌂  Dashboard", "▶  Automatic", "🎮  Manual", "🛡  Collision",
            "🔒  Emergency", "🔌  PLC", "📷  Camera Mode",
        )):
            button = QPushButton(name)
            button.setObjectName("navButton")
            button.setFixedHeight(19)
            button.clicked.connect(
                lambda checked=False, page_index=index: self.navigate(page_index)
            )
            left.addWidget(button)
            self.nav_buttons.append(button)

        left.addStretch(1)
        self.state = QLabel("● READY")
        self.state.setObjectName("stateLabel")
        left.addWidget(self.state)

        emergency_stop = QPushButton("⛔ STOP")
        emergency_stop.setObjectName("stop")
        emergency_stop.setToolTip("EMERGENCY STOP — latch the simulated system")
        emergency_stop.setFixedHeight(26)
        emergency_stop.clicked.connect(
            lambda checked=False: self.system.e_stop("GUI simulation stop")
        )
        left.addWidget(emergency_stop)

        self.system.changed.connect(self.on_state)
        self.navigate(0)
        self.on_state(self.system.state)

        if env is not None:
            self.attach_swift_scene(env, robots or {})

        screen = QApplication.primaryScreen()
        if screen is not None:
            area = screen.availableGeometry()
            self.move(area.left() + 8, area.top() + 8)

    def make_dashboard(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(1, 1, 1, 1)
        layout.setSpacing(3)

        heading = QLabel("Workcell status")
        heading.setObjectName("heading")
        layout.addWidget(heading)

        subtitle = QLabel("Simulation  •  Group integration pending")
        subtitle.setObjectName("dashboardSubtitle")
        layout.addWidget(subtitle)

        row, self.dashboard_state = status_row("System", "READY")
        layout.addWidget(row)
        row, _ = status_row("Manual joints", "READY", "Existing jog controls preserved")
        layout.addWidget(row)
        row, _ = status_row("Auto delivery", "PENDING", "Waiting for group workflow")
        layout.addWidget(row)
        row, _ = status_row("Collision / PLC", "PENDING", "Awaiting the safety and PLC modules")
        layout.addWidget(row)
        row, self.dashboard_camera = status_row("Camera", "NOT LINKED")
        layout.addWidget(row)

        open_camera = QPushButton("📷 Select robot camera →")
        open_camera.setObjectName("cameraAction")
        open_camera.clicked.connect(lambda checked=False: self.navigate(6))
        layout.addWidget(open_camera)
        return page

    @staticmethod
    def disabled_action(name):
        button = QPushButton(name)
        button.setEnabled(False)
        button.setToolTip("Not connected. Waiting for team integration.")
        return button

    def make_automatic_page(self):
        page = SimplePage(
            "Automatic Mode",
            "Colour scanning, tray pickup, filling and delivery are waiting for "
            "the group's working sequence. These buttons are disabled.",
        )
        controls = QVBoxLayout()
        for label in ("START", "PAUSE", "RESUME"):
            controls.addWidget(self.disabled_action(label))
        page.layout().insertLayout(2, controls)
        return page

    def make_collision_page(self):
        page = SimplePage(
            "Collision Mode",
            "Waiting for your teammate's safety/collision module. "
            "No mock detection is shown as a working sensor.",
        )
        page.layout().insertWidget(2, self.disabled_action("Run collision test"))
        return page

    def make_recovery_page(self):
        page = SimplePage(
            "Emergency / Recovery",
            "STOP latches the simulation. Reset below only unlocks this prototype's "
            "SIMULATED latch, not a real robot or safety controller.",
        )
        reset = QPushButton("Unlock simulated stop")
        reset.clicked.connect(lambda checked=False: self.reset_stop())
        page.layout().insertWidget(2, reset)
        return page

    def make_plc_page(self):
        page = SimplePage(
            "PLC Monitor — OFFLINE",
            "Future read-only Modbus RTU monitoring. X14 / input 12 needs "
            "verification; this GUI performs NO PLC reads or writes.",
        )
        page.layout().insertWidget(2, self.disabled_action("Refresh PLC inputs"))
        return page

    def make_camera_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(5)

        heading = QLabel("Camera Mode")
        heading.setObjectName("heading")
        layout.addWidget(heading)
        subtitle = QLabel("Focus the Swift viewer on:")
        subtitle.setObjectName("dashboardSubtitle")
        layout.addWidget(subtitle)

        self.camera_buttons = {}
        for key, label in (
            ("Maryam", "📷 MaryamBot"),
            ("Yara", "📷 YaraBot"),
            ("Hannah", "📷 HannahBot"),
        ):
            button = QPushButton(label)
            button.setObjectName("cameraRobotButton")
            button.setFixedHeight(26)
            button.clicked.connect(
                lambda checked=False, robot_key=key: self.select_camera_robot(robot_key)
            )
            layout.addWidget(button)
            self.camera_buttons[key] = button

        self.camera_status = QLabel("Select a robot. Swift scene not linked yet.")
        self.camera_status.setObjectName("cameraStatus")
        self.camera_status.setWordWrap(True)
        self.camera_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.camera_status, 1)

        stop_follow = QPushButton("Stop following / free view")
        stop_follow.setObjectName("cameraAction")
        stop_follow.clicked.connect(lambda checked=False: self.stop_camera_follow())
        layout.addWidget(stop_follow)
        return page

    def attach_swift_scene(self, env, robots):
        """Bind a REAL, existing Swift environment; never launch another one.

        Use after your team shares the running scene. The dict values must be
        the live robotics models (or handles referring to those models).
        You may supply only the robots currently available.
        """
        if not callable(getattr(env, "set_camera_pose", None)):
            raise TypeError("Expected a running Swift environment with set_camera_pose().")
        if not isinstance(robots, dict):
            raise TypeError("robots must be a dict with Maryam, Yara, Hannah keys")
        self.stop_camera_follow(show_message=False)
        self.swift_env = env
        self.swift_robots = {key: robots[key] for key in CAMERA_OFFSETS if key in robots}
        self.camera_status.setText(
            "Swift linked. Select Maryam, Yara or Hannah to follow its motion."
        )
        self.dashboard_camera.setText("LINKED")
        # If a robot was selected before connection, select it again in the UI.

    def select_camera_robot(self, name):
        """Focus on the chosen robot; keep tracking while its joints move."""
        if name not in CAMERA_OFFSETS:
            return
        self.camera_selected = name
        for key, button in self.camera_buttons.items():
            button.setProperty("active", "true" if key == name else "false")
            button.style().unpolish(button)
            button.style().polish(button)
            button.update()

        if self.swift_env is None:
            self.camera_following = False
            self.camera_timer.stop()
            self.camera_status.setText(
                f"{name} selected. Camera unavailable until this GUI shares "
                "the running Swift scene."
            )
            self.dashboard_camera.setText("NOT LINKED")
            return
        if name not in self.swift_robots:
            self.camera_following = False
            self.camera_timer.stop()
            self.camera_status.setText(
                f"{name}Bot not found in the connected scene."
            )
            self.dashboard_camera.setText("MISSING ROBOT")
            return
        self.camera_following = True
        if self.update_camera():  # Instantly move to the selected arm.
            self.camera_timer.start()
            self.camera_status.setText(
                f"Following {name}Bot in Swift. Its movement stays centred."
            )
            self.dashboard_camera.setText(name.upper())

    def update_camera(self):
        """Change only the Swift VIEW camera, not any robot joint or pose."""
        if not self.camera_following or self.swift_env is None:
            return False
        try:
            robot = self.swift_robots[self.camera_selected]
            position, look_at = _camera_pose_for_robot(robot, self.camera_selected)
            self.swift_env.set_camera_pose(position, look_at)
            return True
        except Exception as exc:
            self.stop_camera_follow(show_message=False)
            self.camera_status.setText(f"Camera focus failed: {exc}")
            self.dashboard_camera.setText("ERROR")
            return False

    def stop_camera_follow(self, show_message=True):
        """Leave the last Swift viewpoint in place for free mouse orbiting."""
        self.camera_timer.stop()
        self.camera_following = False
        if show_message:
            self.camera_status.setText("Following OFF. Rotate and zoom in Swift freely.")
            self.dashboard_camera.setText(
                "LINKED" if self.swift_env is not None else "NOT LINKED"
            )

    def navigate(self, index):
        self.pages.setCurrentIndex(index)
        for i, button in enumerate(self.nav_buttons):
            button.setProperty("active", "true" if i == index else "false")
            button.style().unpolish(button)
            button.style().polish(button)
            button.update()

    def on_state(self, state):
        state = str(state)
        self.state.setText("● " + state)
        self.dashboard_state.setText(state)
        if state == "ESTOP":
            self.navigate(4)

    def reset_stop(self):
        if self.system.state != "ESTOP":
            return
        password, ok = QInputDialog.getText(
            self, "Prototype recovery", "Enter DEMO password:",
            QLineEdit.EchoMode.Password,
        )
        if ok and password == "demo123":
            self.system.reset()
        elif ok:
            QMessageBox.warning(self, "Denied", "Incorrect demo password")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    apply_theme(app)
    app.setStyleSheet(app.styleSheet() + "\n" + COMPACT_STYLE)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
