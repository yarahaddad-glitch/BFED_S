"""Thread-safe GUI requests; scene commands run at simulation checkpoints."""
from collections import deque
import threading

from Collective_Folder import workcell_actions as actions


class WorkcellClosed(Exception):
    pass


class WorkcellRuntime:
    def __init__(self):
        self.condition = threading.Condition()
        self.commands = deque()
        self.activity = deque([(1, 'Loading workcell')], maxlen=80)
        self.activity_sequence = 1
        self.state = "LOADING"
        self.message = "Loading the shared Swift workcell..."
        self.env = None
        self.robots = {}
        self.objects = {}
        self.bridge = None
        self.started = False
        self.closed = False
        self.terminal_fault = False
        self.stop_generation = 0
        self.manual_message = ''
        self.manual_pending = 0
        self.safety = None
        self.collision = None
        self.collision_checks = {}
        self.pending_collision = None
        self.scan_requested = threading.Event()
        self.scan_finished = threading.Event()
        self.scan_result = None
        self.scan_error = None

    def status(self):
        with self.condition:
            return self.state, self.message

    def register_collision_check(self, robot_name, check):
        """check(context, proposed_q, payload_pose) -> True or False/reason string."""
        if not callable(check):
            raise TypeError('Collision check must be callable')
        self.collision_checks.setdefault(robot_name, []).append(check)

    def configuration_clear(self, robot_name, q, payload_pose=None):
        for check in self.collision_checks.get(robot_name, []):
            result = check(self, q, payload_pose)
            if isinstance(result, str) or not result:
                return False
        return True

    def guard_configuration(self, robot_name, q, payload_pose=None):
        """Automatic controllers call this BEFORE assigning a proposed joint state."""
        while not self.configuration_clear(robot_name, q, payload_pose):
            self.pending_collision = (robot_name, q.copy(), payload_pose)
            if self.state != 'ESTOP':
                self.request_stop(f'{robot_name}: teammate collision check blocked motion')
            self.checkpoint()
        self.pending_collision = None

    def registered_path_blocked(self):
        return self.pending_collision is not None and not self.configuration_clear(*self.pending_collision)

    def log(self, message):
        with self.condition:
            self.activity_sequence += 1
            self.activity.append((self.activity_sequence, str(message)))

    def activity_since(self, sequence):
        with self.condition:
            return [(number, message) for number, message in self.activity if number > sequence]

    def set_state(self, state, message):
        with self.condition:
            if state != self.state:
                short = {'READY':'Ready', 'RUNNING':'Running', 'PAUSED':'Paused',
                         'COMPLETE':'Finished', 'FAULT':'Fault'}
                if state in short:
                    self.log(short[state])
            self.state, self.message = state, message
            self.condition.notify_all()

    def submit(self, callback):
        with self.condition:
            self.commands.append(callback)
            self.condition.notify_all()

    def request_stop(self, reason="GUI emergency stop"):
        with self.condition:
            if self.state != 'ESTOP':
                self.log('Stopped')
            self.stop_generation += 1
            self.state, self.message = "ESTOP", reason
            self.commands.append(self._stop_hook)
            self.condition.notify_all()

    def _stop_hook(self):
        note = actions.on_stop(self)
        if note:
            self.message = f'{self.message}. {note}'

    def request_pause(self):
        with self.condition:
            if self.state == "RUNNING":
                self.log('Paused')
                self.state, self.message = "PAUSED", "Paused. Select a robot to jog."
                self.condition.notify_all()

    def start(self):
        # Executed by checkpoint, after any in-flight frame has finished.
        if self.terminal_fault:
            raise ValueError('Task runner failed; correct the fault and relaunch the workcell')
        if self.state not in ("READY", "PAUSED"):
            raise ValueError("Reset the stop first, then press Start / Resume.")
        if self.safety and self.safety.check():
            raise ValueError('Clear the unsafe zone / light curtain before starting')
        if self.collision and self.collision.active_hazards():
            raise ValueError('Remove the collision-test obstacle before starting')
        if self.registered_path_blocked():
            raise ValueError('Clear the collision reported by the team checker first')
        if self.bridge:
            self.bridge.restore_all()
        with self.condition:
            if self.state not in ('READY', 'PAUSED'):
                raise ValueError('Start interrupted by a stop or fault')
            self.started = True
            self.set_state("RUNNING", "Automatic meal sequence running")

    def reset(self):
        if self.terminal_fault:
            raise ValueError('Task runner failed; reset cannot resume a failed sequence')
        if self.state != "ESTOP":
            raise ValueError("No emergency-stop latch to acknowledge")
        generation = self.stop_generation
        if self.registered_path_blocked():
            raise ValueError('Clear the collision reported by the team checker first')
        if self.collision and self.collision.active_hazards():
            raise ValueError('Remove the collision-test obstacle before acknowledging the stop')
        if self.safety and self.safety.check():
            raise ValueError('Clear the unsafe zone / light curtain before acknowledging the stop')
        if not actions.on_reset(self):
            raise ValueError("Reset rejected: a stop input is still active")
        with self.condition:
            if generation != self.stop_generation:
                raise ValueError('A new stop arrived during acknowledgement')
            self.set_state("PAUSED" if self.started else "READY",
                           "Acknowledged. Press Start / Resume separately.")

    def home(self, name):
        if self.state not in ("READY", "PAUSED", "COMPLETE"):
            raise ValueError("Pause or acknowledge the stop before requesting Home")
        if not actions.on_home(self, name):
            raise ValueError("Home trajectory pending in Collective_Folder/workcell_actions.py")

    def motion_step(self, dt=0.025):
        """Team recovery hooks can animate while paused without resuming automation."""
        if self.closed or self.state not in ('READY', 'PAUSED', 'COMPLETE'):
            raise ValueError('Recovery motion is not permitted in the current state')
        if self.bridge is None:
            raise ValueError('Shared workcell is not loaded')
        self.bridge.draw(dt)

    def restart(self):
        if self.state not in ("READY", "PAUSED", "COMPLETE"):
            raise ValueError("Pause or acknowledge the stop before Restart")
        if not actions.on_restart(self):
            raise ValueError("Restart pending in Collective_Folder/workcell_actions.py")
        # A full sequence restart also needs the task runner to reset its local
        # execution state. Keep this explicit until the team's hook supports it.
        self.set_state("PAUSED", "Restart hook completed. Press Start / Resume.")

    def checkpoint(self):
        """Wait without spinning; service queued GUI requests while paused."""
        while True:
            with self.condition:
                if self.closed:
                    raise WorkcellClosed()
                callback = self.commands.popleft() if self.commands else None
                if callback is None:
                    if self.state == "RUNNING":
                        return
                    self.condition.wait(timeout=0.1)
                    continue
            try:
                callback()
            except Exception as exc:
                with self.condition:
                    self.message = str(exc)
                    self.log('Remove prisoner before reset' if 'Clear the unsafe zone' in str(exc)
                             else 'Action not ready' if 'pending' in str(exc) else 'Command rejected')

    def fail(self, reason):
        self.terminal_fault = True
        self.set_state('FAULT', reason)

    def scan(self):
        """Worker side of the Qt camera dialog; no OpenCV UI on this thread."""
        self.scan_error = None
        self.scan_finished.clear()
        self.scan_requested.set()
        while not self.scan_finished.wait(0.1):
            if self.closed:
                raise WorkcellClosed()
        if self.closed:
            raise WorkcellClosed()
        self.scan_requested.clear()
        if self.scan_error:
            raise RuntimeError(self.scan_error)
        return self.scan_result

    def finish_scan(self, result=None, error=None):
        self.scan_result, self.scan_error = result, error
        self.scan_requested.clear()
        self.scan_finished.set()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.scan_finished.set()
