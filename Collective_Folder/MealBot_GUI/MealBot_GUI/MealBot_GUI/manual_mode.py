"""Individual joint jog page, using actual robot classes through RobotBridge."""
from PySide6.QtWidgets import (QWidget,QVBoxLayout,QHBoxLayout,QLabel,QComboBox,QPushButton,
                               QSlider,QGroupBox,QGridLayout,QMessageBox,QLineEdit)
from PySide6.QtCore import Qt, QTimer
import numpy as np

class ManualMode(QWidget):
    def __init__(self,bridge,system,host=None):
        super().__init__()
        self.bridge,self.system=bridge,system
        self.host=host
        self.name=None
        self.pending_jogs={}
        self.sliders=[]; self.values=[]
        layout=QVBoxLayout(self)
        layout.setContentsMargins(3,3,3,3)
        layout.setSpacing(5)
        title=QLabel('Manual control'); title.setObjectName('heading'); layout.addWidget(title)
        line=QHBoxLayout(); line.setSpacing(5); layout.addLayout(line)
        line.addWidget(QLabel('Robot:'))
        self.selector=QComboBox(); self.selector.addItems(['MaryamBot','HannahBot','YaraBot','ReBot']); line.addWidget(self.selector)
        self.load_btn=QPushButton('Select arm'); line.addWidget(self.load_btn)
        self.group=QGroupBox('🤖 Joint angles (degrees)'); self.grid=QGridLayout(self.group);self.grid.setContentsMargins(6,9,6,5);self.grid.setHorizontalSpacing(5);self.grid.setVerticalSpacing(3);layout.addWidget(self.group)
        for i in range(6):
            label=QLabel(f'Joint {i+1}')
            minus=QPushButton('−'); minus.setFixedWidth(25)
            slider=QSlider(Qt.Horizontal); slider.setEnabled(False)
            plus=QPushButton('+'); plus.setFixedWidth(25)
            value=QLabel('—'); value.setMinimumWidth(43)
            self.grid.addWidget(label,i,0);self.grid.addWidget(minus,i,1)
            self.grid.addWidget(slider,i,2);self.grid.addWidget(plus,i,3);self.grid.addWidget(value,i,4)
            slider.valueChanged.connect(lambda v,k=i:self.jog(k,v))
            minus.clicked.connect(lambda _,k=i:self.increment(k,-1))
            plus.clicked.connect(lambda _,k=i:self.increment(k,+1))
            self.sliders.append(slider);self.values.append(value)
        self.limit_note=QLabel('Load a robot to read its joint limits.'); self.limit_note.setWordWrap(True);layout.addWidget(self.limit_note)
        self.restore_btn=QPushButton('Reset robot to saved pose');layout.addWidget(self.restore_btn)
        self.restore_btn.clicked.connect(self.restore_pose)
        cart=QGroupBox('World translation (10 mm)')
        cart_grid=QGridLayout(cart)
        for axis, label in enumerate(('X','Y','Z')):
            for column, sign in enumerate((-1,1)):
                button=QPushButton(f'{label} {"-" if sign < 0 else "+"}')
                button.clicked.connect(lambda _,a=axis,s=sign:self.translate(a,s*.01))
                cart_grid.addWidget(button,axis,column)
        layout.addWidget(cart)
        self.joint_target=QLineEdit()
        self.joint_target.setPlaceholderText('Six joint angles (degrees)')
        layout.addWidget(self.joint_target)
        self.target_btn=QPushButton('Move to supplied joint pose')
        self.target_btn.clicked.connect(self.move_target)
        layout.addWidget(self.target_btn)
        self.home_btn=QPushButton('Home (team recovery hook)');layout.addWidget(self.home_btn)
        self.status=QLabel('Swift not launched.');self.status.setWordWrap(True);layout.addWidget(self.status)
        layout.addStretch(1)
        self.load_btn.clicked.connect(self.load_selected)
        self.selector.currentTextChanged.connect(self.selected_changed)
        self.home_btn.clicked.connect(self.go_home)
        self.system.changed.connect(self.refresh_permissions)
        self.refresh_permissions()
        self.timer=QTimer(self);self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh_angles);self.timer.start()
        self.jog_timer=QTimer(self);self.jog_timer.setInterval(60)
        self.jog_timer.timeout.connect(self.flush_jogs);self.jog_timer.start()

    def activate(self):
        runtime=self.system.runtime
        if runtime:
            runtime.request_pause()
            state,_=runtime.status()
            if self.system.state!=state:
                self.system.state=state
                self.system.changed.emit(state)
        self.load_selected()
    def selected_changed(self,name):
        self.name=None
        for s in self.sliders:s.setEnabled(False)
        window=self.host or self.window()
        if hasattr(window,'select_camera_robot'):
            window.select_camera_robot(name.replace('Bot','') if name!='ReBot' else 'ReBot')
        if self.system.allow_manual():
            self.load_selected()
        else:
            self.status.setText('Pause automatic motion, then select / save pose.')
    def load_selected(self):
        if not self.system.allow_manual(): return
        name=self.selector.currentText()
        try:
            robot=self.bridge.load(name)
            limits,source=self.bridge.limits_deg(name)
            angles=self.bridge.angles_deg(name)
            self.name=name
            self.queue(lambda:self.bridge.begin_manual(name),f'{name}: saved pose; ready to jog')
            window=self.host or self.window()
            if hasattr(window,'select_camera_robot'):
                window.select_camera_robot(name.replace('Bot','') if name!='ReBot' else 'ReBot')
            for i,slider in enumerate(self.sliders):
                slider.blockSignals(True)
                slider.setRange(int(np.ceil(limits[0,i])),int(np.floor(limits[1,i])))
                slider.setValue(int(round(angles[i])))
                self.values[i].setText(f'{angles[i]:.1f}°')
                slider.blockSignals(False)
            self.limit_note.setText(f'{source}. Existing guards plus team manual-safety hook; whole-cell coverage pending Hannah.')
            self.system.enter_manual()
            self.status.setText(f'{name} loaded ({robot.n} joints).')
            self.refresh_permissions()
        except Exception as e:
            self.status.setText('Load failed: '+str(e))
            QMessageBox.warning(self,'Robot load failed',str(e))
    def refresh_permissions(self,*_):
        allowed=self.system.allow_manual()
        self.load_btn.setEnabled(allowed)
        self.home_btn.setEnabled(allowed and self.name is not None)
        self.restore_btn.setEnabled(allowed and self.name is not None)
        self.target_btn.setEnabled(allowed and self.name is not None)
        for s in self.sliders:s.setEnabled(allowed and self.name is not None)
        if not allowed:self.status.setText('Motion locked: '+self.system.state)
    def increment(self,index,direction):
        if self.name and self.system.allow_manual():
            s=self.sliders[index];s.setValue(max(s.minimum(),min(s.maximum(),s.value()+direction)))
    def jog(self,index,value):
        if not self.name or not self.system.allow_manual():return
        try:
            name=self.name
            self.pending_jogs[(name,index)]=float(value)
            self.values[index].setText(f'{value:.1f}°')
            self.status.setText(f'{self.name}: J{index+1} = {value}°')
        except Exception as e:
            self.status.setText(f'Jog rejected: {e}')
    def go_home(self):
        if not self.name or not self.system.allow_manual():return
        name=self.name
        self.queue(lambda:self.bridge.home(name))

    def queue(self, action, success='Robot command completed'):
        runtime=self.system.runtime
        if not runtime:return
        with runtime.condition:
            runtime.manual_pending+=1
        def execute():
            try:
                action()
                runtime.manual_message=success
            except Exception as exc:
                runtime.manual_message=f'Command rejected: {exc}'
                raise
            finally:
                with runtime.condition:
                    runtime.manual_pending-=1
        runtime.submit(execute)

    def flush_jogs(self):
        pending,self.pending_jogs=self.pending_jogs,{}
        for (name,index),value in pending.items():
            if self.system.allow_manual():
                self.queue(lambda n=name,j=index,v=value:self.bridge.set_joint(n,j,v),
                           f'{name}: J{index+1} = {value:.1f} degrees')

    def restore_pose(self):
        if self.name and self.system.allow_manual():
            name=self.name
            self.queue(lambda:self.bridge.restore(name))

    def translate(self,axis,distance):
        if self.name and self.system.allow_manual():
            name=self.name
            self.queue(lambda:self.bridge.translate(name,axis,distance))

    def move_target(self):
        if not self.name or not self.system.allow_manual():return
        try:
            values=np.array([float(v) for v in self.joint_target.text().replace(',',' ').split()])
            if values.shape!=(6,) or not np.all(np.isfinite(values)):
                raise ValueError('Enter six finite angles in degrees')
            name=self.name
            self.queue(lambda:self.bridge.move_to(name,np.deg2rad(values)))
        except ValueError as exc:
            self.status.setText(str(exc))

    def refresh_angles(self):
        if not self.name:return
        try:
            for slider,label,value in zip(self.sliders,self.values,self.bridge.angles_deg(self.name)):
                pending=self.pending_jogs or (self.system.runtime and self.system.runtime.manual_pending)
                if not slider.isSliderDown() and not pending:
                    slider.blockSignals(True);slider.setValue(int(round(value)));slider.blockSignals(False)
                label.setText(f'{value:.1f}°')
            if self.system.runtime and self.system.runtime.manual_message:
                self.status.setText(self.system.runtime.manual_message)
        except Exception as exc:
            self.status.setText(str(exc))
