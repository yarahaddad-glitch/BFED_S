"""Smart MealBot isolated GUI prototype. Run from its own folder or Assignment 2."""
import sys
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QHBoxLayout,QVBoxLayout,
                               QPushButton,QStackedWidget,QLabel,QMessageBox,QInputDialog,QTextEdit,QLineEdit)
from PySide6.QtCore import Qt
from robot_bridge import RobotBridge
from system_manager import SystemManager
from manual_mode import ManualMode
from theme import apply_theme

class SimplePage(QWidget):
    def __init__(self,title,desc):
        super().__init__();lay=QVBoxLayout(self)
        a=QLabel(title);a.setObjectName('heading');lay.addWidget(a)
        b=QLabel(desc);b.setWordWrap(True);lay.addWidget(b);lay.addStretch()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__();self.setWindowTitle('Smart MealBot | Isolated GUI Prototype');self.resize(660, 480); self.setMinimumSize(620, 450)
        self.bridge=RobotBridge();self.system=SystemManager()
        root=QWidget();self.setCentralWidget(root);overall=QHBoxLayout(root)
        overall.setContentsMargins(7,7,7,7);overall.setSpacing(8)
        sidebar=QWidget();sidebar.setObjectName('sidebar');sidebar.setFixedWidth(164);overall.addWidget(sidebar)
        left=QVBoxLayout(sidebar);left.setContentsMargins(8,10,8,9);left.setSpacing(5)
        brand=QLabel('🤖 SMART MEALBOT');brand.setObjectName('brand');left.addWidget(brand)
        blurb=QLabel('💜 simulation control centre');left.addWidget(blurb)
        left.addSpacing(4)
        surface=QWidget();surface.setObjectName('pageSurface');overall.addWidget(surface,1)
        content=QVBoxLayout(surface);content.setContentsMargins(10,10,10,10)
        self.pages=QStackedWidget();content.addWidget(self.pages)
        descriptions=[
            ('Dashboard','Overview of the simulated workcell. Robot status and colour camera integration will be added after joint testing.'),
            ('Automatic Mode','START / PAUSE / RESUME will connect here once the complete coordinated sequence is ready.'),
            ('Manual Mode',''),
            ('Collision Mode','Teammate collision scenarios will be called from here when their safety module is provided. No mock collision is passed off as sensing.'),
            ('Emergency / Recovery','Use the always-visible emergency stop. Reset below is a simulation-only latch reset.'),
            ('PLC Monitor','Offline. Tomorrow: read-only Modbus RTU from the teacher-programmed PLC (X14 at input 12); do not issue writes.'),
            ('Event Log','')]
        self.nav_buttons=[]
        for i,(title,desc) in enumerate(descriptions):
            if title=='Manual Mode':page=ManualMode(self.bridge,self.system)
            elif title=='Emergency / Recovery':
                page=SimplePage(title,desc)
                reset=QPushButton('Unlock simulated stop latch');page.layout().insertWidget(2,reset)
                reset.clicked.connect(self.reset_stop)
            elif title=='Event Log':
                page=QWidget();v=QVBoxLayout(page);v.addWidget(QLabel('EVENT LOG'));self.log=QTextEdit();self.log.setReadOnly(True);v.addWidget(self.log)
            else:page=SimplePage(title,desc)
            self.pages.addWidget(page)
            icons=['🏠','▶','🎮','🛡','🔒','🔌','📋']
            btn=QPushButton(f'{icons[i]}  {title}');btn.setObjectName('navButton');btn.setMinimumHeight(28)
            left.addWidget(btn);self.nav_buttons.append(btn)
            btn.clicked.connect(lambda _,k=i:self.navigate(k))
        left.addStretch()
        self.state=QLabel('●  State: READY');self.state.setObjectName('stateLabel');left.addWidget(self.state)
        stop=QPushButton('⛔ EMERGENCY STOP');stop.setObjectName('stop');stop.setMinimumHeight(39);left.addWidget(stop)
        stop.clicked.connect(lambda:self.system.e_stop('GUI simulation stop'))
        self.system.changed.connect(self.on_state)
        self.system.event.connect(self.log.append)
        self.log.append('GUI launched in SIMULATION mode; no physical robot is controlled.')
        self.navigate(0)
        # Small floating control-panel position; can still be dragged/resized.
        screen = QApplication.primaryScreen()
        if screen:
            rect = screen.availableGeometry()
            self.move(rect.right() - self.width() - 12, rect.top() + 12)
    def navigate(self,index):
        self.pages.setCurrentIndex(index)
        for i,btn in enumerate(self.nav_buttons):
            btn.setProperty('active', 'true' if i==index else 'false')
            btn.style().unpolish(btn)
            btn.style().polish(btn)
            btn.update()
    def on_state(self,state):
        self.state.setText('●  State: '+state)
        if state=='ESTOP':self.navigate(4)
    def reset_stop(self):
        if self.system.state!='ESTOP':return
        password,ok=QInputDialog.getText(self,'Prototype recovery','Enter DEMO password:',QLineEdit.EchoMode.Password)
        if ok and password=='demo123': self.system.reset()
        elif ok: QMessageBox.warning(self,'Denied','Incorrect demo password')

if __name__=='__main__':
    app=QApplication(sys.argv);apply_theme(app);window=MainWindow();window.show();sys.exit(app.exec())
