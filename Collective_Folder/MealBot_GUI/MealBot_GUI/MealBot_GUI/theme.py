"""Compact purple cartoon styling. No robot-control logic is changed."""

PURPLE_THEME = """
QWidget { background: #F7F1FF; color: #38204F; font: 8pt 'Trebuchet MS'; }
QMainWindow { background: #F7F1FF; }
QWidget#sidebar { background: #5A338B; border-radius: 12px; }
QWidget#sidebar QLabel { background: transparent; color: white; }
QWidget#pageSurface { background: #FFFCFF; border: 2px solid #DCC2FA; border-radius: 12px; }
QLabel#heading { font: bold 12pt 'Trebuchet MS'; color: #58278E; background: transparent; }
QLabel#brand { font: bold 10pt 'Trebuchet MS'; color: white; background: transparent; }
QLabel#subtitle { font: 8pt 'Trebuchet MS'; color: #74558D; background: transparent; }
QLabel#stateLabel { background: #EEDCFF; color: #512581; border: 1px solid #C39AEA;
    border-radius: 7px; padding: 5px; font-weight: bold; }
QWidget#sidebar QLabel#stateLabel { background: #EEDCFF; color: #512581; }
QPushButton { background: #B580E6; color: #FFFFFF; border: 1px solid #8850BC;
    border-radius: 7px; padding: 4px 6px; font-weight: bold; min-height: 16px; }
QPushButton:hover { background: #CB9DF2; border-color: #7C42AE; }
QPushButton:pressed { background: #884EBB; }
QPushButton:disabled { background: #E2D5EC; color: #8E809D; border-color: #CDBCD9; }
QPushButton#navButton { background: #7045A0; border: 1px solid #966FC1; text-align: left;
    border-radius: 8px; padding: 5px 7px; }
QPushButton#navButton:hover { background: #8659B9; }
QPushButton#navButton[active='true'] { background: #F3E5FF; color: #542C7B; border-color: #F8F1FF; }
QPushButton#stop { background: #E63E72; border: 2px solid #A71F50; border-radius: 10px;
    font: bold 9pt 'Trebuchet MS'; min-height: 25px; }
QPushButton#stop:hover { background: #F35485; }
QGroupBox { background: #FFFCFF; color: #57298B; border: 1px solid #D7B6F4;
    border-radius: 10px; margin-top: 9px; padding: 8px 5px 5px 5px;
    font: bold 9pt 'Trebuchet MS'; }
QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 4px; background: #FFFCFF; }
QComboBox, QLineEdit, QTextEdit { background: white; color: #44255C;
    border: 1px solid #CBA8EC; border-radius: 7px; padding: 4px; selection-background-color: #A776DC; }
QComboBox::drop-down { border: none; width: 14px; }
QComboBox QAbstractItemView { background: white; color: #44255C; selection-background-color: #EDD9FF; }
QSlider::groove:horizontal { height: 6px; border-radius: 3px; background: #E3C9FB; }
QSlider::sub-page:horizontal { background: #B178E6; border-radius: 3px; }
QSlider::handle:horizontal { background: #8643CA; border: 1px solid #61339A;
    width: 13px; margin: -5px 0; border-radius: 7px; }
QSlider:disabled::handle:horizontal { background: #C8B8D5; border-color: #BDB0C8; }
QMessageBox, QInputDialog { background: #F7F1FF; }
"""

def apply_theme(app):
    app.setStyleSheet(PURPLE_THEME)
