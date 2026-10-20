"""Reusable purple cartoon theme for the simulated Smart MealBot GUI."""

PURPLE_THEME = """
QWidget { background: #F7F1FF; color: #38204F; font: 10pt 'Trebuchet MS'; }
QMainWindow { background: #F7F1FF; }
QWidget#sidebar { background: #5A338B; border-radius: 22px; }
QWidget#sidebar QLabel { background: transparent; color: #FFFFFF; }
QWidget#pageSurface { background: #FFFCFF; border: 2px solid #DCC2FA; border-radius: 20px; }
QLabel#heading { font: bold 20pt 'Trebuchet MS'; color: #58278E; background: transparent; }
QLabel#brand { font: bold 17pt 'Trebuchet MS'; color: white; background: transparent; }
QLabel#subtitle { font: 10pt 'Trebuchet MS'; color: #74558D; background: transparent; }
QLabel#stateLabel { background: #EEDCFF; color: #512581; border: 2px solid #C39AEA;
    border-radius: 12px; padding: 10px; font-weight: bold; }
QWidget#sidebar QLabel#stateLabel { background: #EEDCFF; color: #512581; }
QPushButton { background: #B580E6; color: #FFFFFF; border: 2px solid #8850BC;
    border-radius: 13px; padding: 9px 12px; font-weight: bold; min-height: 24px; }
QPushButton:hover { background: #CB9DF2; border-color: #7C42AE; }
QPushButton:pressed { background: #884EBB; }
QPushButton:disabled { background: #E2D5EC; color: #8E809D; border-color: #CDBCD9; }
QPushButton#navButton { background: #7045A0; border: 2px solid #966FC1; text-align: left;
    border-radius: 13px; padding: 11px 12px; }
QPushButton#navButton:hover { background: #8659B9; }
QPushButton#navButton[active='true'] { background: #F3E5FF; color: #542C7B; border-color: #F8F1FF; }
QPushButton#stop { background: #E63E72; border: 3px solid #A71F50; border-radius: 17px;
    font: bold 12pt 'Trebuchet MS'; min-height: 46px; }
QPushButton#stop:hover { background: #F35485; }
QGroupBox { background: #FFFCFF; color: #57298B; border: 2px solid #D7B6F4;
    border-radius: 16px; margin-top: 16px; padding: 15px 10px 10px 10px;
    font: bold 11pt 'Trebuchet MS'; }
QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 7px;
    background: #FFFCFF; }
QComboBox, QLineEdit, QTextEdit { background: #FFFFFF; color: #44255C;
    border: 2px solid #CBA8EC; border-radius: 12px; padding: 9px; selection-background-color: #A776DC; }
QComboBox::drop-down { border: none; width: 24px; }
QComboBox QAbstractItemView { background: white; color: #44255C; selection-background-color: #EDD9FF; }
QSlider::groove:horizontal { height: 10px; border-radius: 5px; background: #E3C9FB; }
QSlider::sub-page:horizontal { background: #B178E6; border-radius: 5px; }
QSlider::handle:horizontal { background: #8643CA; border: 2px solid #61339A;
    width: 21px; margin: -7px 0; border-radius: 11px; }
QSlider:disabled::handle:horizontal { background: #C8B8D5; border-color: #BDB0C8; }
QMessageBox, QInputDialog { background: #F7F1FF; }
"""


def apply_theme(app):
    app.setStyleSheet(PURPLE_THEME)
