"""GUI for main_all's shared workcell; standalone execution uses that entry point.

Qt owns desktop widgets and the colour scanner. Scene commands are queued to
the simulation thread. Physical safety and recovery belong to workcell_actions.
"""

import sys
from pathlib import Path

PROJECT_ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'Maryam_Work').is_dir())
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QInputDialog, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QStackedWidget,
    QVBoxLayout, QWidget, QPlainTextEdit,
)

from robot_bridge import RobotBridge
from system_manager import SystemManager
from manual_mode import ManualMode
from theme import apply_theme
from scaled_panel import ScaledPanel


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
    "Maryam": (-1.1, -1.75, 0.8),
    # +Y keeps the viewpoint inside the meal area, in front of the wall at Y=1.23.
    "Yara": (0.9, 1.1, 1.25),
    "Hannah": (0.15, -0.35, 2.8),
    "ReBot": (0.9, -1.1, 0.8),
}


def _xyz(pose):
    """Return x/y/z from an SE3 pose, numpy vector, or 3-sequence."""
    if hasattr(pose, "t"):
        pose = pose.t
    elif hasattr(pose, "A"):
        pose = pose.A
        return [float(pose[i, 3]) for i in range(3)]
    return [float(pose[i]) for i in range(3)]


def _camera_pose_for_robot(robot, name, near_robot=None):
    """Compute a viewpoint centred between the robot base and end effector.

    Accepts standard Robotics Toolbox DH/ERobot models. Also accepts a Swift
    handle with a .robot or .model reference and a .q joint-state vector.
    """
    model = robot
    if not callable(getattr(model, "fkine", None)) and name != 'ReBot':
        model = getattr(robot, "robot", None) or getattr(robot, "model", None)
    if model is None or (not callable(getattr(model, "fkine", None)) and name != 'ReBot'):
        raise ValueError("Robot must provide fkine(q), or a model/robot with fkine(q).")

    base_pose = getattr(model, "base", None)
    if base_pose is None:
        raise ValueError("Robot model has no world base pose for camera focus.")
    base = _xyz(base_pose)

    if name == 'Hannah':
        # Follow above the mobile base, independent of arm extension or base yaw.
        look_at = [base[0], base[1], base[2] + 0.45]
        return [base[i] + CAMERA_OFFSETS[name][i] for i in range(3)], look_at

    q = getattr(robot, "q", None)
    if q is None:
        q = getattr(model, "q", None)
    if q is None:
        raise ValueError("Robot has no live joint configuration q.")
    if name == 'ReBot':
        links = model.get_link_transforms(q)
        tip = _xyz(model.base * model.get_gripper_transforms(links, model.gripper_opening)[0])
    else:
        tip = _xyz(model.fkine(q))

    # Tracking the arm centre produces less camera jitter than locking to the
    # gripper alone while still following the movement of the full arm.
    look_at = [0.45 * base[i] + 0.55 * tip[i] for i in range(3)]
    look_at[2] = max(look_at[2], base[2] + 0.25)
    offset = CAMERA_OFFSETS[name]
    position = [look_at[i] + offset[i] for i in range(3)]
    if name == 'Maryam' and near_robot is not None:
        anchor = _xyz(near_robot.base)
        position = [anchor[0], max(anchor[1] + 0.5, 1.9),
                    max(anchor[2] + 1.5, look_at[2] + 0.35)]
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

    def __init__(self, env=None, robots=None, runtime=None):
        super().__init__()
        self.setWindowTitle("Smart MealBot | Compact Controls")
        self.resize(350, 400)
        self.setMinimumSize(280, 320)
        self.setWindowFlag(Qt.WindowStaysOnTopHint, True)

        self.bridge = RobotBridge()
        self.system = SystemManager()
        self.runtime = runtime
        self.system.runtime = runtime
        self.last_activity = 0
        self.last_safety_alert = 0
        self.safety_warning = None

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
        self.manual_page = ManualMode(self.bridge, self.system, host=self)
        self.pages.addWidget(compact_scroll(self.manual_page))
        self.pages.addWidget(compact_scroll(self.make_collision_page()))
        self.pages.addWidget(compact_scroll(self.make_recovery_page()))
        self.pages.addWidget(compact_scroll(self.make_plc_page()))
        self.pages.addWidget(self.make_camera_page())  # REPLACES Event Log.
        self.pages.addWidget(compact_scroll(self.make_safety_page()))

        self.nav_buttons = []
        self.nav_page_indices = []
        for name, index in (('Dashboard', 0), ('Automatic', 1), ('Manual', 2),
                            ('Camera', 6), ('Collision tests', 3), ('Safety tests', 7),
                            ('PLC', 5), ('Emergency stop', 4)):
            button = QPushButton(name)
            button.setObjectName("navButton")
            button.setStyleSheet('font-size: 10px; padding: 1px 3px;')
            button.setFixedHeight(19)
            button.clicked.connect(
                lambda checked=False, page_index=index: self.navigate(page_index)
            )
            left.addWidget(button)
            self.nav_buttons.append(button)
            self.nav_page_indices.append(index)

        left.addStretch(1)
        self.state = QLabel("● READY")
        self.state.setObjectName("stateLabel")
        left.addWidget(self.state)

        emergency_stop = QPushButton("⛔ STOP")
        emergency_stop.setObjectName("stop")
        emergency_stop.setToolTip("EMERGENCY STOP — latch the simulated system")
        emergency_stop.setFixedHeight(26)
        emergency_stop.clicked.connect(lambda checked=False: self.stop_all())
        left.addWidget(emergency_stop)

        from PySide6.QtGui import QShortcut, QKeySequence
        for key, action in (("E", self.stop_all), ("H", self.home_all), ("R", self.resume_task)):
            QShortcut(QKeySequence(key), self, activated=action)

        self.system.changed.connect(self.on_state)
        self.navigate(0)
        self.on_state(self.system.state)
        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(100)
        self.poll_timer.timeout.connect(self.poll_runtime)
        self.poll_timer.start()

        # One transform scales fonts, spacing, buttons, sliders and their hit areas.
        # Resizing the outer window preserves the panel's proportions.
        self.takeCentralWidget()
        root.setParent(None)
        self.scaled_panel = ScaledPanel(root, self)
        self.setCentralWidget(self.scaled_panel)

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

        subtitle = QLabel("One shared Swift workcell")
        subtitle.setObjectName("dashboardSubtitle")
        layout.addWidget(subtitle)

        row, self.dashboard_state = status_row("System", "READY")
        layout.addWidget(row)
        row, _ = status_row("Manual joints", "READY", "Existing jog controls preserved")
        layout.addWidget(row)
        row, self.dashboard_loaded = status_row("Swift scene", "LOADING")
        layout.addWidget(row)
        row, _ = status_row("Collision / PLC", "PENDING", "Awaiting the safety and PLC modules")
        layout.addWidget(row)
        row, self.dashboard_safety = status_row('Zone / curtain', 'LOADING')
        layout.addWidget(row)
        row, self.dashboard_camera = status_row("Camera", "NOT LINKED")
        layout.addWidget(row)

        open_camera = QPushButton("📷 Select robot camera →")
        open_camera.setObjectName("cameraAction")
        open_camera.clicked.connect(lambda checked=False: self.navigate(6))
        layout.addWidget(open_camera)
        self.start_button = QPushButton('Start / Resume')
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_sequence)
        layout.addWidget(self.start_button)
        self.activity_terminal = QPlainTextEdit()
        self.activity_terminal.setReadOnly(True)
        self.activity_terminal.setMaximumBlockCount(80)
        self.activity_terminal.setMinimumHeight(110)
        self.activity_terminal.setMaximumHeight(150)
        self.activity_terminal.setPlaceholderText('Loading workcell')
        layout.addWidget(self.activity_terminal)
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
            "Start opens the colour scanner and starts prisoner animation. "
            "Pause before manual control. Resume restores saved manual poses first.",
        )
        controls = QVBoxLayout()
        for label, callback in (("Start / Resume", self.start_sequence),
                                ("Pause for manual control", self.pause_sequence),
                                ("Restart (team hook)", self.restart_sequence)):
            button=QPushButton(label)
            button.clicked.connect(callback)
            controls.addWidget(button)
        page.layout().insertLayout(2, controls)
        return page

    def make_collision_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        heading = QLabel('MaryamBot collision test')
        heading.setObjectName('heading')
        layout.addWidget(heading)
        self.collision_status = QLabel('Loading')
        self.collision_status.setWordWrap(True)
        layout.addWidget(self.collision_status)
        for label, callback in (('Insert obstacle on next tray move', self.arm_collision_test),
                                ('1. Remove obstacle', self.remove_collision_obstacle),
                                ('2. Reset', self.reset_stop),
                                ('3. Start / Resume', self.start_sequence)):
            button = QPushButton(label)
            button.clicked.connect(callback)
            layout.addWidget(button)
        layout.addStretch()
        return page

    def arm_collision_test(self):
        if self.runtime and self.runtime.collision:
            self.runtime.request_pause()
            self.runtime.submit(self.runtime.collision.arm_test)
            self.select_camera_robot('Maryam')

    def remove_collision_obstacle(self):
        if self.runtime and self.runtime.collision:
            self.runtime.request_pause()
            self.runtime.submit(self.runtime.collision.remove_obstacle)

    def make_recovery_page(self):
        page = SimplePage(
            "Emergency / Recovery",
            "STOP freezes every robot. Home sends them to their home pose (still holding items). "
            "Reset resumes the task from where it stopped. "
            "Yara's hardware and recovery hooks: Collective_Folder/workcell_actions.py.",
        )
        home = QPushButton("Home (robots go to home pose)")
        home.clicked.connect(lambda checked=False: self.home_all())
        page.layout().insertWidget(2, home)
        reset = QPushButton("Reset (resume the task)")
        reset.clicked.connect(lambda checked=False: self.resume_task())
        page.layout().insertWidget(3, reset)
        clear = QPushButton('Clear safety test actor')
        clear.clicked.connect(self.clear_safety_actor)
        page.layout().insertWidget(4, clear)
        return page

    def make_safety_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        heading = QLabel('Safety tests')
        heading.setObjectName('heading')
        layout.addWidget(heading)
        self.safety_status = QLabel('Loading')
        layout.addWidget(self.safety_status)
        for text, kind in (('Test light curtain', 'curtain'),
                                            ('Test kitchen entry', 'kitchen'),
                                            ('Test isolation entry', 'isolation')):
            button = QPushButton(text)
            button.clicked.connect(lambda checked=False, k=kind:self.run_safety_test(k))
            layout.addWidget(button)
        clear = QPushButton('1. Remove prisoner')
        clear.clicked.connect(self.clear_safety_actor)
        layout.addWidget(clear)
        reset = QPushButton('2. Reset')
        reset.clicked.connect(self.reset_stop)
        layout.addWidget(reset)
        resume = QPushButton('3. Start / Resume')
        resume.clicked.connect(self.start_sequence)
        layout.addWidget(resume)
        layout.addStretch()
        return page

    def run_safety_test(self, kind):
        if self.runtime and self.runtime.safety:
            self.runtime.request_pause()
            self.runtime.submit(lambda:self.runtime.safety.run(kind))

    def clear_safety_actor(self):
        if self.runtime and self.runtime.safety:
            self.runtime.request_pause()
            self.runtime.submit(self.runtime.safety.clear_actor)

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
            ("ReBot", "📷 ReBot"),
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
            "Swift linked. Select Maryam, Yara, Hannah or ReBot to follow its motion."
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
            position, look_at = _camera_pose_for_robot(
                robot, self.camera_selected, near_robot=self.swift_robots.get('Yara'))
            if self.runtime:
                env = self.swift_env
                self.runtime.submit(lambda: env.set_camera_pose(position, look_at))
            else:
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
        if index == 2:
            self.manual_page.activate()
        for page_index, button in zip(self.nav_page_indices, self.nav_buttons):
            button.setProperty("active", "true" if page_index == index else "false")
            button.style().unpolish(button)
            button.style().polish(button)
            button.update()

    def on_state(self, state):
        state = str(state)
        self.state.setText("● " + state)
        self.dashboard_state.setText(state)
        if state == "ESTOP" and self.pages.currentIndex() not in (3, 7):
            self.navigate(4)

    def stop_all(self):
        """STOP: freeze every robot in place (task paused, not cancelled)."""
        if self.runtime and hasattr(self.runtime, "estop_stop"):
            self.runtime.estop_stop("GUI stop")
        self.system.e_stop("GUI simulation stop")

    def home_all(self):
        """HOME: (after STOP) robots glide to their home pose, still carrying anything they hold."""
        if self.runtime and hasattr(self.runtime, "estop_home"):
            self.runtime.estop_home()

    def resume_task(self):
        """RESET: release the stop and carry on with the task that was running."""
        if self.system.state == "ESTOP":
            self.system.reset()                      # clear the GUI/runtime stop latch
        if self.runtime and hasattr(self.runtime, "estop_resume"):
            self.runtime.estop_resume()              # robots glide back to where they stopped
        if self.runtime and self.runtime.status()[0] in ("READY", "PAUSED"):
            self.runtime.submit(self.runtime.start)  # carry on automatically

    def reset_stop(self):
        if self.system.state != "ESTOP":
            return
        self.system.reset()

    def start_sequence(self):
        if self.runtime and self.runtime.state in ('READY','PAUSED'):
            self.runtime.submit(self.runtime.start)

    def pause_sequence(self):
        if self.runtime:
            self.runtime.request_pause()

    def restart_sequence(self):
        if self.runtime:
            self.runtime.submit(self.runtime.restart)

    def poll_runtime(self):
        if not self.runtime:
            return
        state, message = self.runtime.status()
        for number, activity in self.runtime.activity_since(self.last_activity):
            self.activity_terminal.appendPlainText(activity)
            self.last_activity = number
        if self.runtime.collision:
            self.collision_status.setText(self.runtime.collision.status)
        if self.runtime.safety:
            safety = self.runtime.safety
            hazards = safety.active_hazards()
            self.dashboard_safety.setText('TRIPPED' if hazards else 'CLEAR')
            self.safety_status.setText('Prisoner in area' if hazards else 'Area clear')
            if safety.alert_number > self.last_safety_alert:
                self.last_safety_alert = safety.alert_number
                if self.safety_warning:
                    self.safety_warning.close()
                self.safety_warning = QMessageBox(QMessageBox.Warning, 'Warning',
                    'Prisoner in area', QMessageBox.Ok, self)
                self.safety_warning.setWindowModality(Qt.NonModal)
                self.safety_warning.show()
        self.start_button.setEnabled(state in ('READY','PAUSED'))
        self.dashboard_loaded.setText('LOADED' if self.runtime.robots else 'LOADING')
        if state == 'FAULT':
            self.dashboard_loaded.setText('FAULT')
        if self.runtime.robots and self.swift_env is None:
            cameras = {name.replace('Bot','') if name != 'ReBot' else 'ReBot': robot
                       for name, robot in self.runtime.robots.items()}
            self.attach_swift_scene(self.runtime.env, cameras)
        if state != self.system.state:
            self.system.state = state
            self.system.changed.emit(state)

    def closeEvent(self, event):
        if self.safety_warning:
            self.safety_warning.close()
        if self.runtime:
            self.runtime.close()
        event.accept()


if __name__ == "__main__":
    from Collective_Folder.EnviroParts_S.main_all import main
    main()
