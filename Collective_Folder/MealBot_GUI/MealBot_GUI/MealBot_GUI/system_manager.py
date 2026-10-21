"""Simulation-only shared motion permission / stop latch."""
from PySide6.QtCore import QObject, Signal
class SystemManager(QObject):
    changed=Signal(str)
    event=Signal(str)
    def __init__(self):
        super().__init__()
        self.runtime=None
        self.state='LOADING'
        self.stop_reason=''
    def e_stop(self,reason='GUI emergency stop'):
        if self.runtime:
            self.runtime.request_stop(reason)
        self.state='ESTOP'; self.stop_reason=reason
        self.changed.emit(self.state); self.event.emit('STOP: '+reason)
    def reset(self):
        if self.runtime:
            self.runtime.submit(self.runtime.reset)
            return
        # Prototype reset only, not a real safety reset or hardware rearm.
        if self.state=='ESTOP':
            self.state='READY'; self.stop_reason=''
            self.changed.emit(self.state); self.event.emit('Simulation stop latch reset')
    def allow_manual(self):
        return self.state in ('READY','PAUSED','COMPLETE','MANUAL')
    def enter_manual(self):
        if self.runtime:
            return
        if self.allow_manual():
            self.state='MANUAL'; self.changed.emit(self.state)
