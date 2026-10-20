"""Individual joint jog page, using actual robot classes through RobotBridge."""
from PySide6.QtWidgets import (QWidget,QVBoxLayout,QHBoxLayout,QLabel,QComboBox,QPushButton,
                               QSlider,QGroupBox,QGridLayout,QMessageBox)
from PySide6.QtCore import Qt
import numpy as np

class ManualMode(QWidget):
    def __init__(self,bridge,system):
        super().__init__()
        self.bridge,self.system=bridge,system
        self.name=None
        self.sliders=[]; self.values=[]
        layout=QVBoxLayout(self)
        title=QLabel('🎮 MANUAL ROBOT CONTROL'); title.setObjectName('heading'); layout.addWidget(title)
        line=QHBoxLayout(); layout.addLayout(line)
        line.addWidget(QLabel('Select robot:'))
        self.selector=QComboBox(); self.selector.addItems(['MaryamBot','HannahBot','YaraBot']); line.addWidget(self.selector)
        self.load_btn=QPushButton('Load selected robot in Swift'); line.addWidget(self.load_btn)
        self.group=QGroupBox('🤖 J1–J6  •  Joint angles (degrees)'); self.grid=QGridLayout(self.group);layout.addWidget(self.group)
        for i in range(6):
            label=QLabel(f'Joint {i+1}')
            minus=QPushButton('−'); minus.setFixedWidth(36)
            slider=QSlider(Qt.Horizontal); slider.setEnabled(False)
            plus=QPushButton('+'); plus.setFixedWidth(36)
            value=QLabel('—'); value.setMinimumWidth(62)
            self.grid.addWidget(label,i,0);self.grid.addWidget(minus,i,1)
            self.grid.addWidget(slider,i,2);self.grid.addWidget(plus,i,3);self.grid.addWidget(value,i,4)
            slider.valueChanged.connect(lambda v,k=i:self.jog(k,v))
            minus.clicked.connect(lambda _,k=i:self.increment(k,-1))
            plus.clicked.connect(lambda _,k=i:self.increment(k,+1))
            self.sliders.append(slider);self.values.append(value)
        self.limit_note=QLabel('Load a robot to read its joint limits.'); self.limit_note.setWordWrap(True);layout.addWidget(self.limit_note)
        self.home_btn=QPushButton('Reset selected robot pose to home (simulation only)');layout.addWidget(self.home_btn)
        self.status=QLabel('Swift not launched.');self.status.setWordWrap(True);layout.addWidget(self.status)
        layout.addStretch()
        self.load_btn.clicked.connect(self.load_selected)
        self.selector.currentTextChanged.connect(self.selected_changed)
        self.home_btn.clicked.connect(self.go_home)
        self.system.changed.connect(self.refresh_permissions)
        self.refresh_permissions()
    def selected_changed(self,name):
        self.name=None
        for s in self.sliders:s.setEnabled(False)
        self.status.setText(f'Press Load to select {name}.')
    def load_selected(self):
        if not self.system.allow_manual(): return
        name=self.selector.currentText()
        try:
            robot=self.bridge.load(name)
            limits,source=self.bridge.limits_deg(name)
            angles=self.bridge.angles_deg(name)
            self.name=name
            for i,slider in enumerate(self.sliders):
                slider.blockSignals(True)
                slider.setRange(int(np.ceil(limits[0,i])),int(np.floor(limits[1,i])))
                slider.setValue(int(round(angles[i])))
                self.values[i].setText(f'{angles[i]:.1f}°')
                slider.blockSignals(False)
            self.limit_note.setText(f'{source}. Manual setpoint preview only: no path or obstacle clearance checking.')
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
        for s in self.sliders:s.setEnabled(allowed and self.name is not None)
        if not allowed:self.status.setText('Motion locked: '+self.system.state)
    def increment(self,index,direction):
        if self.name and self.system.allow_manual():
            s=self.sliders[index];s.setValue(max(s.minimum(),min(s.maximum(),s.value()+direction)))
    def jog(self,index,value):
        if not self.name or not self.system.allow_manual():return
        try:
            self.bridge.set_joint(self.name,index,float(value))
            self.values[index].setText(f'{value:.1f}°')
            self.status.setText(f'{self.name}: J{index+1} = {value}°')
        except Exception as e:
            self.status.setText(f'Jog rejected: {e}')
    def go_home(self):
        if not self.name or not self.system.allow_manual():return
        answer=QMessageBox.question(self,'Simulation-only reset','Instantly reset this robot to its saved home pose? This is NOT a planned trajectory.')
        if answer!=QMessageBox.Yes:return
        try:
            self.bridge.home(self.name)
            self.load_selected()
        except Exception as e:QMessageBox.warning(self,'Home rejected',str(e))
