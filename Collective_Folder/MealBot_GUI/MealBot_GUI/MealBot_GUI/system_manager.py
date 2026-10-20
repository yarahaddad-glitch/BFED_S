"""Simulation-only shared motion permission / stop latch."""
from PySide6.QtCore import QObject, Signal
class SystemManager(QObject):
    changed=Signal(str)
    event=Signal(str)
    def __init__(self):
        super().__init__()
        self.state='READY'
        self.stop_reason=''
    def e_stop(self,reason='GUI emergency stop'):
        self.state='ESTOP'; self.stop_reason=reason
        self.changed.emit(self.state); self.event.emit('STOP: '+reason)
    def reset(self):
        # Prototype reset only, not a real safety reset or hardware rearm.
        if self.state=='ESTOP':
            self.state='READY'; self.stop_reason=''
            self.changed.emit(self.state); self.event.emit('Simulation stop latch reset')
    def allow_manual(self):
        return self.state in ('READY','MANUAL')
    def enter_manual(self):
        if self.allow_manual():
            self.state='MANUAL'; self.changed.emit(self.state)
