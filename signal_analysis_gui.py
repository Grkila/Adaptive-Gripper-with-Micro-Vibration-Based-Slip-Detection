import sys
import json
import csv
import time
import random
import numpy as np
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QComboBox, QCheckBox, QFileDialog, 
                             QGroupBox, QSplitter, QFrame, QScrollArea, QRadioButton, QButtonGroup,
                             QTabWidget, QTextEdit, QSpinBox, QDoubleSpinBox, QSlider,
                             QDialog, QFormLayout, QDialogButtonBox, QColorDialog, QGridLayout,
                             QMenuBar, QMenu, QToolBar)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QThread, QObject
from PyQt6.QtGui import QColor, QPalette, QFont, QAction
import pyqtgraph as pg

# ==========================================
# Constants & Configuration
# ==========================================
DEFAULT_BAUD_RATE = 2000000
BAUD_RATES = [9600, 115200, 500000, 921600, 1000000, 2000000]

# Themes
THEMES = {
    "Dark": {
        "bg": "#161719", "panel": "#1f1f20", "text": "#c7d0d9", "border": "#333",
        "accent1": "#00bcd4", "accent2": "#e91e63", "accent3": "#8bc34a",
        "accent4": "#ff9800", "accent5": "#9c27b0",
        "chart_bg": "#161719", "grid": "#333333", "hover": "#2a2a2b"
    },
    "Light": {
        "bg": "#f0f2f5", "panel": "#ffffff", "text": "#1c1e21", "border": "#ccd0d5",
        "accent1": "#0078d7", "accent2": "#d32f2f", "accent3": "#2e7d32",
        "accent4": "#ed6c02", "accent5": "#9c27b0",
        "chart_bg": "#ffffff", "grid": "#e4e6eb", "hover": "#e4e6eb"
    }
}

# Defaults (Dark)
COLOR_BG = THEMES["Dark"]["bg"]
COLOR_PANEL = THEMES["Dark"]["panel"]
COLOR_TEXT = THEMES["Dark"]["text"]
COLOR_ACCENT_1 = THEMES["Dark"]["accent1"]
COLOR_ACCENT_2 = THEMES["Dark"]["accent2"]
COLOR_ACCENT_3 = THEMES["Dark"]["accent3"]
COLOR_ACCENT_4 = THEMES["Dark"]["accent4"]
COLOR_ACCENT_5 = THEMES["Dark"]["accent5"]

def get_stylesheet(theme):
    return f"""
    QMainWindow {{
        background-color: {theme['bg']};
        color: {theme['text']};
    }}
    QWidget {{
        background-color: {theme['bg']};
        color: {theme['text']};
        font-family: 'Segoe UI', sans-serif;
        font-size: 10pt;
    }}
    QGroupBox {{
        border: 1px solid {theme['border']};
        border-radius: 5px;
        margin-top: 10px;
        font-weight: bold;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 3px;
    }}
    QPushButton {{
        background-color: {theme['panel']};
        border: 1px solid {theme['border']};
        border-radius: 4px;
        padding: 5px 10px;
        color: {theme['text']};
    }}
    QPushButton:hover {{
        background-color: {theme['hover']};
        border-color: {theme['accent1']};
    }}
    QPushButton:pressed {{
        background-color: {theme['accent1']};
        color: #000;
    }}
    QComboBox {{
        background-color: {theme['panel']};
        border: 1px solid {theme['border']};
        border-radius: 4px;
        padding: 5px;
        color: {theme['text']};
    }}
    QComboBox QAbstractItemView {{
        background-color: {theme['panel']};
        color: {theme['text']};
        selection-background-color: {theme['accent1']};
        selection-color: #000;
    }}
    QCheckBox {{
        spacing: 5px;
    }}
    QCheckBox::indicator {{
        width: 15px;
        height: 15px;
        border-radius: 3px;
        border: 1px solid #555;
        background-color: {theme['panel']};
    }}
    QCheckBox::indicator:checked {{
        background-color: {theme['accent3']};
        border-color: {theme['accent3']};
    }}
    QLabel {{
        color: {theme['text']};
    }}
    QMenu {{
        background-color: {theme['panel']};
        border: 1px solid {theme['border']};
    }}
    QMenu::item {{
        padding: 5px 20px;
        color: {theme['text']};
        background-color: transparent;
    }}
    QMenu::item:selected {{
        background-color: {theme['accent1']};
        color: #000;
    }}
    QTabWidget::pane {{ border: 1px solid {theme['border']}; }}
    QTabBar::tab {{
        background: {theme['panel']};
        color: {theme['text']};
        padding: 8px 20px;
        border-top-left-radius: 4px;
        border-top-right-radius: 4px;
        border: 1px solid {theme['border']};
        margin-right: 2px;
    }}
    QTabBar::tab:selected {{
        background: {theme['hover']};
        border-bottom: 2px solid {theme['accent1']};
    }}
    QSpinBox#accent_border {{
        border: 1px solid {theme['accent1']};
    }}
    """

# ==========================================
# Serial Worker
# ==========================================
class SerialWorker(QThread):
    data_received = pyqtSignal(list)
    raw_received = pyqtSignal(list)
    error_occurred = pyqtSignal(str)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.ser = None
        self.pending_commands = []
        self.read_buffer = ""

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, dsrdtr=False)
            
            while self.running and self.ser.is_open:
                while self.pending_commands:
                    cmd = self.pending_commands.pop(0)
                    self.ser.write((cmd + '\n').encode())

                if self.ser.in_waiting:
                    try:
                        raw_data = self.ser.read(self.ser.in_waiting)
                        text_data = raw_data.decode('utf-8', errors='ignore')
                        
                        self.read_buffer += text_data
                        
                        if '\n' in self.read_buffer:
                            lines = self.read_buffer.split('\n')
                            self.read_buffer = lines.pop()
                            
                            batch_data = []
                            raw_lines_to_emit = lines[-20:] if len(lines) > 20 else lines
                            
                            for line in lines:
                                line = line.strip()
                                if not line:
                                    continue
                                
                                payload_str = line
                                valid_payload = False
                                
                                if '|' in line:
                                    parts = line.rsplit('|', 1)
                                    if len(parts) == 2:
                                        content, chk_hex = parts
                                        try:
                                            chk_hex = chk_hex.strip()
                                            recv_chk = int(chk_hex, 16)
                                            calc_chk = 0
                                            for char in content:
                                                calc_chk ^= ord(char)
                                            if calc_chk == recv_chk:
                                                payload_str = content
                                                valid_payload = True
                                        except ValueError:
                                            pass
                                else:
                                    valid_payload = False

                                if valid_payload and payload_str.startswith('{') and payload_str.endswith('}'):
                                    try:
                                        data = json.loads(payload_str)
                                        batch_data.append(data)
                                    except json.JSONDecodeError:
                                        pass
                            
                            if batch_data:
                                self.data_received.emit(batch_data)
                            
                            if raw_lines_to_emit:
                                self.raw_received.emit(raw_lines_to_emit)
                                
                    except Exception:
                        pass
                else:
                    self.msleep(1)
                    
        except Exception as e:
            if self.running:
                self.error_occurred.emit(str(e))
        finally:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

    def send_command(self, cmd):
        self.pending_commands.append(cmd)

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()
        self.wait()

# ==========================================
# Signal Generator (Simulation)
# ==========================================
class SignalGenerator:
    def __init__(self):
        self.t = 0
    
    def next_sample(self):
        self.t += 0.01
        mx = 10 * np.sin(2 * np.pi * 1.0 * self.t) + np.random.normal(0, 0.5)
        my = 10 * np.sin(2 * np.pi * 2.0 * self.t + np.pi/4) + np.random.normal(0, 0.5)
        mz = 10 * np.sin(2 * np.pi * 0.5 * self.t + np.pi/2) + np.random.normal(0, 0.5)
        mag = np.sqrt(mx**2 + my**2 + mz**2)
        cur = 5 + 2 * np.sin(2 * np.pi * 5.0 * self.t) + np.random.normal(0, 0.2)
        
        return {
            "mlx": mx, "mly": my, "mlz": mz,
            "mhx": mx * 0.1, "mhy": my * 0.1, "mhz": mz * 0.1,
            "rmx": mx + np.random.normal(0, 2), "rmy": my + np.random.normal(0, 2), "rmz": mz + np.random.normal(0, 2),
            "mag": mag, "cur": cur, "slip": 0,
            "t": self.t * 1000 
        }

# ==========================================
# Main Application
# ==========================================
class StyleEditorDialog(QDialog):
    def __init__(self, current_style, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Plot Style")
        self.result_style = current_style.copy()
        
        layout = QVBoxLayout(self)
        form = QFormLayout()
        
        # Color Picker
        self.btn_color = QPushButton()
        self.btn_color.setFixedSize(50, 25)
        self.btn_color.setStyleSheet(f"background-color: {self.result_style['color']}; border: 1px solid #555;")
        self.btn_color.clicked.connect(self.pick_color)
        form.addRow("Color:", self.btn_color)
        
        # Width
        self.spin_width = QSpinBox()
        self.spin_width.setRange(1, 10)
        self.spin_width.setValue(self.result_style.get('width', 1))
        self.spin_width.valueChanged.connect(self.set_width)
        form.addRow("Line Width:", self.spin_width)
        
        # Style
        self.combo_style = QComboBox()
        self.styles = {
            "Solid": Qt.PenStyle.SolidLine,
            "Dash": Qt.PenStyle.DashLine,
            "Dot": Qt.PenStyle.DotLine,
            "DashDot": Qt.PenStyle.DashDotLine,
            "DashDotDot": Qt.PenStyle.DashDotDotLine
        }
        for name, val in self.styles.items():
            self.combo_style.addItem(name, val)
            if val == self.result_style.get('style'):
                self.combo_style.setCurrentText(name)
        
        self.combo_style.currentIndexChanged.connect(self.set_line_style)
        form.addRow("Line Type:", self.combo_style)
        
        layout.addLayout(form)
        
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        
    def pick_color(self):
        c = QColorDialog.getColor(QColor(self.result_style['color']), self, "Select Color")
        if c.isValid():
            self.result_style['color'] = c.name()
            self.btn_color.setStyleSheet(f"background-color: {c.name()}; border: 1px solid #555;")

    def set_width(self, val):
        self.result_style['width'] = val

    def set_line_style(self):
        self.result_style['style'] = self.combo_style.currentData()

class PlotSettingsWidget(QGroupBox):
    def __init__(self, title, plot_widget, parent=None):
        super().__init__(title, parent)
        self.plot = plot_widget
        self.layout = QVBoxLayout(self)
        
        # X-Axis Window (Samples)
        h_x = QHBoxLayout()
        h_x.addWidget(QLabel("X-Window (Samples):"))
        self.spin_window = QSpinBox()
        self.spin_window.setRange(10, 50000)
        self.spin_window.setValue(1000)
        self.spin_window.setSingleStep(100)
        self.spin_window.setObjectName("accent_border")
        h_x.addWidget(self.spin_window)
        self.layout.addLayout(h_x)

        # Auto Scale
        self.chk_auto = QCheckBox("Auto Scale Y-Axis")
        self.chk_auto.setChecked(True)
        self.chk_auto.toggled.connect(self.toggle_auto)
        self.layout.addWidget(self.chk_auto)
        
        # Center DC
        self.chk_center = QCheckBox("Center at DC")
        self.chk_center.toggled.connect(self.toggle_center)
        self.layout.addWidget(self.chk_center)
        
        # Range for DC
        h_dc = QHBoxLayout()
        h_dc.addWidget(QLabel("Range (+/-):"))
        self.spin_dc_range = QDoubleSpinBox()
        self.spin_dc_range.setRange(0.1, 10000)
        self.spin_dc_range.setValue(20)
        self.spin_dc_range.setEnabled(False)
        h_dc.addWidget(self.spin_dc_range)
        self.layout.addLayout(h_dc)
        
        # Manual Range
        h_man = QHBoxLayout()
        self.spin_min = QSpinBox()
        self.spin_min.setRange(-10000, 10000)
        self.spin_min.setValue(-10)
        self.spin_min.setEnabled(False)
        self.spin_min.valueChanged.connect(self.update_manual)
        
        self.spin_max = QSpinBox()
        self.spin_max.setRange(-10000, 10000)
        self.spin_max.setValue(10)
        self.spin_max.setEnabled(False)
        self.spin_max.valueChanged.connect(self.update_manual)
        
        h_man.addWidget(self.spin_min)
        h_man.addWidget(self.spin_max)
        self.layout.addLayout(h_man)
        
    def toggle_auto(self, checked):
        if checked:
            self.chk_center.setChecked(False)
            self.plot.enableAutoRange(axis='y')
            self.spin_min.setEnabled(False)
            self.spin_max.setEnabled(False)
        else:
            if not self.chk_center.isChecked():
                self.spin_min.setEnabled(True)
                self.spin_max.setEnabled(True)
                self.update_manual()
                
    def toggle_center(self, checked):
        if checked:
            self.chk_auto.setChecked(False)
            self.spin_dc_range.setEnabled(True)
            self.spin_min.setEnabled(False)
            self.spin_max.setEnabled(False)
            self.plot.disableAutoRange(axis='y')
        else:
            self.spin_dc_range.setEnabled(False)
            if not self.chk_auto.isChecked():
                self.spin_min.setEnabled(True)
                self.spin_max.setEnabled(True)
                self.update_manual()
                
    def update_manual(self):
        if not self.chk_auto.isChecked() and not self.chk_center.isChecked():
            mn = self.spin_min.value()
            mx = self.spin_max.value()
            if mn < mx:
                self.plot.setYRange(mn, mx)

    def apply_dc_center(self, values):
        if self.chk_center.isChecked() and values:
            avg = np.mean(values)
            rng = self.spin_dc_range.value()
            self.plot.setYRange(avg - rng, avg + rng, padding=0)

class AdaptiveGripperGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Adaptive Gripper Analysis")
        self.resize(1200, 800)
        
        # Data Storage
        self.buffer_size = 50000
        self.data = {
            'mlx': [], 'mly': [], 'mlz': [], 'mag': [], 
            'mhx': [], 'mhy': [], 'mhz': [],
            'rmx': [], 'rmy': [], 'rmz': [],
            'cur': [], 'slip': [], 's_ind': [],
            'srv': [], 'grp': [],
            'timestamp': [], 'recv_ts': []
        }
        self.spike_buffer = {k: [] for k in self.data.keys() if k != 'timestamp'}
        
        self.fft_data = {'freqs': [], 'mags': []}
        self.recorded_events = []
        self.current_segment_start = None
        
        # Recording & Replay
        self.is_recording = False
        self.recording_data = [] 
        self.recording_fft = [] 
        self.replay_data = []    
        self.replay_fft_data = [] 
        self.replay_index = 0
        
        self.serial_thread = None
        self.sim_generator = SignalGenerator()
        self.is_simulating = False
        self.is_connected = False
        
        self.curve_styles = {
            'mlx': {'color': COLOR_ACCENT_2, 'style': Qt.PenStyle.SolidLine, 'width': 2},
            'mly': {'color': COLOR_ACCENT_3, 'style': Qt.PenStyle.SolidLine, 'width': 2},
            'mlz': {'color': COLOR_ACCENT_1, 'style': Qt.PenStyle.SolidLine, 'width': 2},
            'mag': {'color': "#ffffff", 'style': Qt.PenStyle.SolidLine, 'width': 2},
            'mhx': {'color': COLOR_ACCENT_2, 'style': Qt.PenStyle.DotLine, 'width': 1},
            'mhy': {'color': COLOR_ACCENT_3, 'style': Qt.PenStyle.DotLine, 'width': 1},
            'mhz': {'color': COLOR_ACCENT_1, 'style': Qt.PenStyle.DotLine, 'width': 1},
            'rmx': {'color': "#777777", 'style': Qt.PenStyle.DashLine, 'width': 1},
            'rmy': {'color': "#777777", 'style': Qt.PenStyle.DashLine, 'width': 1},
            'rmz': {'color': "#777777", 'style': Qt.PenStyle.DashLine, 'width': 1},
            'cur': {'color': COLOR_ACCENT_4, 'style': Qt.PenStyle.SolidLine, 'width': 2},
            'slip': {'color': "#ff0000", 'style': Qt.PenStyle.SolidLine, 'width': 2},
            's_ind': {'color': "#ff0000", 'style': Qt.PenStyle.DotLine, 'width': 1},
            'srv': {'color': COLOR_ACCENT_5, 'style': Qt.PenStyle.SolidLine, 'width': 2},
            'grp': {'color': COLOR_ACCENT_5, 'style': Qt.PenStyle.DashLine, 'width': 1},
        }
        
        # Theme Init
        self.current_theme = "Dark"

        # Setup UI
        self.setup_ui()
        self.setup_plotting()
        self.apply_theme("Dark")
        
        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(33)

        # Timer for replay
        self.replay_timer = QTimer()
        self.replay_timer.timeout.connect(self.update_replay_loop)

    def apply_theme(self, theme_name):
        if theme_name not in THEMES: return
        self.current_theme = theme_name
        theme = THEMES[theme_name]
        
        self.setStyleSheet(get_stylesheet(theme))
        
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor(theme['bg']))
        palette.setColor(QPalette.ColorRole.WindowText, QColor(theme['text']))
        palette.setColor(QPalette.ColorRole.Base, QColor(theme['panel']))
        palette.setColor(QPalette.ColorRole.AlternateBase, QColor(theme['bg']))
        palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(theme['text']))
        palette.setColor(QPalette.ColorRole.ToolTipText, QColor(theme['text']))
        palette.setColor(QPalette.ColorRole.Text, QColor(theme['text']))
        palette.setColor(QPalette.ColorRole.Button, QColor(theme['panel']))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor(theme['text']))
        palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
        palette.setColor(QPalette.ColorRole.Link, QColor(theme['accent1']))
        palette.setColor(QPalette.ColorRole.Highlight, QColor(theme['accent1']))
        palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
        QApplication.instance().setPalette(palette)
        
        pg.setConfigOption('background', theme['chart_bg'])
        pg.setConfigOption('foreground', theme['text'])
        
        plots = [self.plot_time_1, self.plot_time_2, self.plot_fft]
        if hasattr(self, 'plot_replay_1'): plots.extend([self.plot_replay_1, self.plot_replay_2, self.plot_replay_fft])
        
        for p in plots:
            if hasattr(p, 'setBackground'):
                p.setBackground(theme['chart_bg'])
                p.getAxis('bottom').setPen(theme['text'])
                p.getAxis('left').setPen(theme['text'])
                if p.plotItem.titleLabel:
                    p.plotItem.titleLabel.setAttr('color', theme['text'])

    def setup_menu(self):
        menubar = self.menuBar()
        menubar.clear()
        
        file_menu = menubar.addMenu('&File')
        load_action = QAction('Load Replay', self)
        load_action.triggered.connect(self.load_replay_file)
        file_menu.addAction(load_action)
        
        import_action = QAction('Import Data', self)
        import_action.triggered.connect(self.import_data)
        file_menu.addAction(import_action)
        
        file_menu.addSeparator()
        exit_action = QAction('Exit', self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        view_menu = menubar.addMenu('&View')
        theme_menu = view_menu.addMenu('Theme')
        dark_action = QAction('Dark', self)
        dark_action.triggered.connect(lambda: self.apply_theme("Dark"))
        theme_menu.addAction(dark_action)
        light_action = QAction('Light', self)
        light_action.triggered.connect(lambda: self.apply_theme("Light"))
        theme_menu.addAction(light_action)
        
        view_menu.addSeparator()
        self.action_show_fft = QAction('Show FFT', self, checkable=True)
        self.action_show_fft.toggled.connect(self.toggle_fft_view_menu)
        view_menu.addAction(self.action_show_fft)
        
        toolbar = QToolBar("Connection")
        self.addToolBar(toolbar)
        
        toolbar.addWidget(QLabel(" Port: "))
        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(120)
        self.refresh_ports()
        toolbar.addWidget(self.combo_ports)
        
        toolbar.addWidget(QLabel(" Baud: "))
        self.combo_baud = QComboBox()
        for b in BAUD_RATES: self.combo_baud.addItem(str(b))
        self.combo_baud.setCurrentText(str(DEFAULT_BAUD_RATE))
        toolbar.addWidget(self.combo_baud)
        
        toolbar.addSeparator()
        self.action_connect = QAction("Connect", self)
        self.action_connect.triggered.connect(self.toggle_connection)
        toolbar.addAction(self.action_connect)
        
        btn_refresh = QAction("Refresh", self)
        btn_refresh.triggered.connect(self.refresh_ports)
        toolbar.addAction(btn_refresh)
        
        toolbar.addSeparator()
        self.chk_sim = QCheckBox("Simulation")
        self.chk_sim.toggled.connect(self.toggle_simulation)
        toolbar.addWidget(self.chk_sim)
        
        toolbar.addSeparator()
        self.btn_record = QPushButton(" Record ")
        self.btn_record.setCheckable(True)
        self.btn_record.clicked.connect(self.toggle_recording)
        toolbar.addWidget(self.btn_record)

    def toggle_fft_view_menu(self, checked):
        self.update_layout_visibility()

    def setup_ui(self):
        self.setup_menu()
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        scroll_sidebar = QScrollArea()
        scroll_sidebar.setWidgetResizable(True)
        scroll_sidebar.setFixedWidth(380)
        scroll_sidebar.setFrameShape(QFrame.Shape.NoFrame)
        
        self.sidebar = QWidget()
        sidebar_layout = QVBoxLayout(self.sidebar)
        sidebar_layout.setContentsMargins(10, 15, 10, 15)
        sidebar_layout.setSpacing(15)
        scroll_sidebar.setWidget(self.sidebar)

        self.setup_telemetry_ui(sidebar_layout)

        grp_fft_cfg = QGroupBox("FFT Config")
        v_fft_cfg = QVBoxLayout()
        
        h_samp = QHBoxLayout()
        h_samp.addWidget(QLabel("Samples:"))
        self.spin_fft_samples = QSpinBox()
        self.spin_fft_samples.setRange(32, 2048)
        self.spin_fft_samples.setValue(128)
        h_samp.addWidget(self.spin_fft_samples)
        v_fft_cfg.addLayout(h_samp)
        
        h_rate = QHBoxLayout()
        h_rate.addWidget(QLabel("Rate (Hz):"))
        self.spin_fft_rate = QSpinBox()
        self.spin_fft_rate.setRange(1, 10000)
        self.spin_fft_rate.setValue(2000) 
        h_rate.addWidget(self.spin_fft_rate)
        v_fft_cfg.addLayout(h_rate)
        
        h_fft_scale = QHBoxLayout()
        self.chk_fft_auto = QCheckBox("Auto Y-Scale")
        self.chk_fft_auto.setChecked(True)
        self.chk_fft_auto.toggled.connect(self.toggle_fft_scale)
        h_fft_scale.addWidget(self.chk_fft_auto)
        
        self.spin_fft_max = QSpinBox()
        self.spin_fft_max.setRange(1, 100000)
        self.spin_fft_max.setValue(100)
        self.spin_fft_max.setEnabled(False)
        self.spin_fft_max.setSuffix(" max")
        self.spin_fft_max.valueChanged.connect(self.update_fft_range)
        h_fft_scale.addWidget(self.spin_fft_max)
        
        v_fft_cfg.addLayout(h_fft_scale)
        grp_fft_cfg.setLayout(v_fft_cfg)
        sidebar_layout.addWidget(grp_fft_cfg)

        self.plot_time_1 = pg.PlotWidget(title="Time Series 1")
        self.plot_time_1.showGrid(x=True, y=True, alpha=0.3)
        
        self.settings_p1 = PlotSettingsWidget("Plot 1 Settings", self.plot_time_1)
        sidebar_layout.addWidget(self.settings_p1)

        self.plot_time_2 = pg.PlotWidget(title="Time Series 2")
        self.plot_time_2.showGrid(x=True, y=True, alpha=0.3)
        
        self.chk_show_p2 = QCheckBox("Show Plot 2")
        self.chk_show_p2.setChecked(False)
        self.chk_show_p2.toggled.connect(self.update_layout_visibility)
        
        self.settings_p2 = PlotSettingsWidget("Plot 2 Settings", self.plot_time_2)
        self.settings_p2.layout.insertWidget(0, self.chk_show_p2)
        sidebar_layout.addWidget(self.settings_p2)

        grp_ana = QGroupBox("Analysis")
        v_ana = QVBoxLayout()
        self.chk_despike = QCheckBox("Despike (Median Filter)")
        self.chk_despike.setChecked(True)
        v_ana.addWidget(self.chk_despike)
        self.lbl_freq = QLabel("Dominant Freq: 0.0 Hz")
        v_ana.addWidget(self.lbl_freq)
        grp_ana.setLayout(v_ana)
        sidebar_layout.addWidget(grp_ana)
        
        sidebar_layout.addStretch()
        main_layout.addWidget(scroll_sidebar)

        self.tabs = QTabWidget()
        
        tab_viz = QWidget()
        layout_viz = QVBoxLayout(tab_viz)
        
        self.viz_splitter = QSplitter(Qt.Orientation.Vertical)
        self.viz_splitter.addWidget(self.plot_time_1)
        self.viz_splitter.addWidget(self.plot_time_2)
        
        self.plot_fft = pg.PlotWidget(title="Real-Time FFT")
        self.plot_fft.showGrid(x=True, y=True, alpha=0.3)
        self.plot_fft.setLabel('bottom', "Frequency", units='Hz')
        self.plot_fft.setLabel('left', "Magnitude")
        self.viz_splitter.addWidget(self.plot_fft)
        
        layout_viz.addWidget(self.viz_splitter)
        
        self.tab_replay = QWidget()
        self.setup_replay_ui(self.tab_replay)
        
        self.tab_raw = QWidget()
        layout_raw = QVBoxLayout(self.tab_raw)
        self.text_raw = QTextEdit()
        self.text_raw.setReadOnly(True)
        self.text_raw.setStyleSheet(f"background-color: #000; color: #0f0; font-family: Consolas, monospace;")
        layout_raw.addWidget(self.text_raw)
        
        self.tabs.addTab(tab_viz, "Visualizer")
        self.tabs.addTab(self.tab_replay, "Replay")
        self.tabs.addTab(self.tab_raw, "Raw Serial")
        
        main_layout.addWidget(self.tabs, stretch=1)
        self.update_layout_visibility()

    def update_layout_visibility(self):
        show_fft = self.action_show_fft.isChecked()
        show_p2 = self.chk_show_p2.isChecked()
        
        if show_fft:
            self.plot_fft.setVisible(True)
            self.plot_time_1.setVisible(False)
            self.plot_time_2.setVisible(False)
        else:
            self.plot_fft.setVisible(False)
            self.plot_time_1.setVisible(True)
            self.plot_time_2.setVisible(show_p2)
            
            # Ensure equal splitting when P2 is shown
            if show_p2:
                self.viz_splitter.setSizes([1000, 1000, 0])
            else:
                self.viz_splitter.setSizes([1000, 0, 0])
    
    def setup_replay_ui(self, parent):
        layout = QVBoxLayout(parent)
        
        h_controls = QHBoxLayout()
        self.btn_load_replay = QPushButton("Load File")
        self.btn_load_replay.clicked.connect(self.load_replay_file)
        h_controls.addWidget(self.btn_load_replay)
        
        self.btn_play_replay = QPushButton("Play")
        self.btn_play_replay.clicked.connect(self.toggle_replay)
        h_controls.addWidget(self.btn_play_replay)

        btn_step_back = QPushButton("<<")
        btn_step_back.clicked.connect(lambda: self.step_replay(-1))
        h_controls.addWidget(btn_step_back)
        
        btn_step_fwd = QPushButton(">>")
        btn_step_fwd.clicked.connect(lambda: self.step_replay(1))
        h_controls.addWidget(btn_step_fwd)
        
        h_controls.addWidget(QLabel("Speed (ms):"))
        self.spin_replay_speed = QSpinBox()
        self.spin_replay_speed.setRange(1, 1000)
        self.spin_replay_speed.setValue(33)
        self.spin_replay_speed.valueChanged.connect(self.update_replay_timer)
        h_controls.addWidget(self.spin_replay_speed)

        self.slider_replay = QSlider(Qt.Orientation.Horizontal)
        self.slider_replay.setRange(0, 100)
        self.slider_replay.sliderMoved.connect(self.on_replay_slider_move)
        h_controls.addWidget(self.slider_replay)
        
        self.lbl_replay_time = QLabel("0.0s")
        h_controls.addWidget(self.lbl_replay_time)
        
        layout.addLayout(h_controls)
        
        h_view = QHBoxLayout()
        self.chk_replay_fft = QCheckBox("Show FFT")
        self.chk_replay_fft.setChecked(True)
        self.chk_replay_fft.toggled.connect(self.update_replay_layout)
        h_view.addWidget(self.chk_replay_fft)
        
        self.chk_replay_p2 = QCheckBox("Show 2nd Graph")
        self.chk_replay_p2.toggled.connect(self.update_replay_layout)
        h_view.addWidget(self.chk_replay_p2)
        
        layout.addLayout(h_view)
        
        self.replay_splitter = QSplitter(Qt.Orientation.Vertical)
        
        self.plot_replay_1 = pg.PlotWidget(title="Replay Time Series 1")
        self.replay_splitter.addWidget(self.plot_replay_1)
        
        self.plot_replay_2 = pg.PlotWidget(title="Replay Time Series 2")
        self.plot_replay_2.setVisible(False)
        self.replay_splitter.addWidget(self.plot_replay_2)
        
        self.plot_replay_fft = pg.PlotWidget(title="Replay FFT")
        self.replay_splitter.addWidget(self.plot_replay_fft)
        
        layout.addWidget(self.replay_splitter)

        h_settings = QHBoxLayout()
        self.settings_replay_1 = PlotSettingsWidget("Graph 1 Settings", self.plot_replay_1)
        h_settings.addWidget(self.settings_replay_1)
        
        self.settings_replay_2 = PlotSettingsWidget("Graph 2 Settings", self.plot_replay_2)
        self.settings_replay_2.setVisible(False)
        h_settings.addWidget(self.settings_replay_2)
        
        layout.addLayout(h_settings)
        
        self.replay_curves_p1 = {}
        self.replay_curves_p2 = {}
        
        def create_replay_curve(plot_widget, key, name):
            s = self.curve_styles[key]
            pen = pg.mkPen(s['color'], width=s['width'], style=s['style'])
            return plot_widget.plot(pen=pen, name=name)
        
        keys = ['mlx', 'mly', 'mlz', 'mag', 'mhx', 'mhy', 'mhz', 'rmx', 'rmy', 'rmz', 'cur', 'slip', 's_ind', 'srv', 'grp']
        names = {
            'mlx': 'Mag X', 'mly': 'Mag Y', 'mlz': 'Mag Z', 'mag': 'Magnitude',
            'mhx': 'HP X', 'mhy': 'HP Y', 'mhz': 'HP Z',
            'rmx': 'Raw X', 'rmy': 'Raw Y', 'rmz': 'Raw Z',
            'cur': 'Current', 'slip': 'Slip State', 's_ind': 'Slip Ind',
            'srv': 'Servo', 'grp': 'Grip'
        }

        for key in keys:
            self.replay_curves_p1[key] = create_replay_curve(self.plot_replay_1, key, names[key])
            self.replay_curves_p2[key] = create_replay_curve(self.plot_replay_2, key, names[key])
            
            self.replay_curves_p1[key].setVisible(False)
            self.replay_curves_p2[key].setVisible(False)

        # Replay Curve (FFT)
        self.curve_replay_fft = self.plot_replay_fft.plot(pen=pg.mkPen(COLOR_ACCENT_1, width=2, fillLevel=0, brush=(0, 188, 212, 50)))
        
        # For backward compatibility with existing logic that uses self.replay_curves
        self.replay_curves = self.replay_curves_p1

    def step_replay(self, direction):
        if not self.replay_data: return
        self.replay_index += direction
        if self.replay_index < 0: self.replay_index = 0
        if self.replay_index >= len(self.replay_data): self.replay_index = len(self.replay_data) - 1
        self.slider_replay.setValue(self.replay_index)
        self.update_replay_plot_snapshot()

    def update_replay_timer(self, val):
        if self.replay_timer.isActive():
            self.replay_timer.setInterval(val)

    def update_replay_layout(self):
        show_fft = self.chk_replay_fft.isChecked()
        show_p2 = self.chk_replay_p2.isChecked()
        
        self.plot_replay_fft.setVisible(show_fft)
        self.plot_replay_2.setVisible(show_p2)
        self.settings_replay_2.setVisible(show_p2)
        
        # Distribute space in replay splitter
        sizes = []
        
        # 1. Plot 1 (Always visible)
        sizes.append(1000)
        
        # 2. Plot 2
        if show_p2:
            sizes.append(1000)
        else:
            sizes.append(0)
            
        # 3. FFT
        if show_fft:
            sizes.append(1000)
        else:
            sizes.append(0)
            
        self.replay_splitter.setSizes(sizes)

    def setup_telemetry_ui(self, layout):
        grp = QGroupBox("Telemetry & Plotting")
        vbox = QVBoxLayout()
        vbox.setSpacing(10)
        
        def create_stream_group(label, command, plot_keys):
            chk_cmd = QCheckBox(label)
            chk_cmd.toggled.connect(lambda c: self.send_stream_command(command, c))
            
            sub_widget = QWidget()
            sub_layout = QGridLayout(sub_widget)
            sub_layout.setContentsMargins(15, 0, 0, 0)
            sub_layout.setHorizontalSpacing(10)
            sub_layout.setVerticalSpacing(2)
            
            sub_layout.addWidget(QLabel("Signal"), 0, 0)
            sub_layout.addWidget(QLabel("P1"), 0, 1)
            sub_layout.addWidget(QLabel("P2"), 0, 2)
            sub_layout.addWidget(QLabel(""), 0, 3) 
            
            sub_checks = {}
            row_idx = 1
            for key, name in plot_keys.items():
                lbl = QLabel(name)
                
                chk_p1 = QCheckBox()
                chk_p1.toggled.connect(lambda c, k=key: self.toggle_plot_visibility(k, 1, c))
                
                chk_p2 = QCheckBox()
                chk_p2.toggled.connect(lambda c, k=key: self.toggle_plot_visibility(k, 2, c))
                
                btn_style = QPushButton("🎨")
                btn_style.setToolTip("Edit Style")
                btn_style.setFixedSize(25, 20)
                btn_style.clicked.connect(lambda _, k=key: self.open_style_picker(k))
                
                lbl.setVisible(False)
                chk_p1.setVisible(False)
                chk_p2.setVisible(False)
                btn_style.setVisible(False)
                
                sub_layout.addWidget(lbl, row_idx, 0)
                sub_layout.addWidget(chk_p1, row_idx, 1)
                sub_layout.addWidget(chk_p2, row_idx, 2)
                sub_layout.addWidget(btn_style, row_idx, 3)
                
                sub_checks[key] = (lbl, chk_p1, chk_p2, btn_style)
                row_idx += 1
                
            def on_main_toggle(checked):
                for lbl, cp1, cp2, btn in sub_checks.values():
                    lbl.setVisible(checked)
                    cp1.setVisible(checked)
                    cp2.setVisible(checked)
                    btn.setVisible(checked)
                    if not checked:
                        cp1.setChecked(False)
                        cp2.setChecked(False)
            
            chk_cmd.toggled.connect(on_main_toggle)
            
            vbox.addWidget(chk_cmd)
            vbox.addWidget(sub_widget)
            
            return chk_cmd, sub_checks

        self.grp_mag, self.chk_mag = create_stream_group("Mag Low Pass", "mag_lowpass", {
            'mlx': "Mag X", 'mly': "Mag Y", 'mlz': "Mag Z", 'mag': "Magnitude"
        })
        
        self.grp_mag_hp, self.chk_mag_hp = create_stream_group("Mag High Pass", "mag_highpass", {
            'mhx': "HP X", 'mhy': "HP Y", 'mhz': "HP Z"
        })
        
        self.grp_raw, self.chk_raw = create_stream_group("Mag Raw", "mag_raw", {
            'rmx': "Raw X", 'rmy': "Raw Y", 'rmz': "Raw Z"
        })

        self.grp_cur, self.chk_cur = create_stream_group("Current", "current", {
            'cur': "Current (mA)"
        })

        self.grp_slip, self.chk_slip = create_stream_group("Slip Detection", "slip", {
            'slip': "Slip State", 's_ind': "Slip Indicator"
        })

        self.grp_srv, self.chk_srv = create_stream_group("Servo & Mode", "servo", {
            'srv': "Servo Pos", 'grp': "Grip State"
        })
        
        self.chk_cmd_fft = QCheckBox("FFT Mode (Exclusive)")
        self.chk_cmd_fft.toggled.connect(lambda c: self.send_stream_command("fft", c))
        vbox.addWidget(self.chk_cmd_fft)

        grp.setLayout(vbox)
        layout.addWidget(grp)

    def toggle_plot_visibility(self, key, plot_idx, visible):
        if plot_idx == 1:
            if key in self.curves_p1:
                self.curves_p1[key].setVisible(visible)
            # Replay P1
            if hasattr(self, 'replay_curves_p1') and key in self.replay_curves_p1:
                self.replay_curves_p1[key].setVisible(visible)
        elif plot_idx == 2:
            if key in self.curves_p2:
                self.curves_p2[key].setVisible(visible)
            # Replay P2
            if hasattr(self, 'replay_curves_p2') and key in self.replay_curves_p2:
                self.replay_curves_p2[key].setVisible(visible)

    def send_stream_command(self, key, enabled):
        if self.is_connected and self.serial_thread:
            cmd = json.dumps({key: enabled})
            self.serial_thread.send_command(cmd)
            self.text_raw.append(f">> SENT: {cmd}")

    def setup_plotting(self):
        self.curves_p1 = {}
        self.curves_p2 = {}
        
        def create_curve(plot_widget, key, name):
            s = self.curve_styles[key]
            pen = pg.mkPen(s['color'], width=s['width'], style=s['style'])
            c = plot_widget.plot(pen=pen, name=name)
            c.setVisible(False)
            return c

        keys = ['mlx', 'mly', 'mlz', 'mag',
                'mhx', 'mhy', 'mhz',
                'rmx', 'rmy', 'rmz',
                'cur', 'slip', 's_ind',
                'srv', 'grp']
        
        names = {
            'mlx': 'Mag X', 'mly': 'Mag Y', 'mlz': 'Mag Z', 'mag': 'Magnitude',
            'mhx': 'HP X', 'mhy': 'HP Y', 'mhz': 'HP Z',
            'rmx': 'Raw X', 'rmy': 'Raw Y', 'rmz': 'Raw Z',
            'cur': 'Current', 'slip': 'Slip State', 's_ind': 'Slip Ind',
            'srv': 'Servo', 'grp': 'Grip'
        }

        for key in keys:
            self.curves_p1[key] = create_curve(self.plot_time_1, key, names[key])
            self.curves_p2[key] = create_curve(self.plot_time_2, key, names[key])

        self.curve_fft = self.plot_fft.plot(pen=pg.mkPen(COLOR_ACCENT_1, width=2, fillLevel=0, brush=(0, 188, 212, 50)))

    def update_curve_style(self, key):
        if key not in self.curve_styles:
            return
        
        s = self.curve_styles[key]
        pen = pg.mkPen(s['color'], width=s['width'], style=s['style'])
        
        if key in self.curves_p1:
            self.curves_p1[key].setPen(pen)
        if key in self.curves_p2:
            self.curves_p2[key].setPen(pen)
        # Replay Curves
        if hasattr(self, 'replay_curves_p1') and key in self.replay_curves_p1:
            self.replay_curves_p1[key].setPen(pen)
        if hasattr(self, 'replay_curves_p2') and key in self.replay_curves_p2:
            self.replay_curves_p2[key].setPen(pen)

    def open_style_picker(self, key):
        if key not in self.curve_styles:
            return
            
        dlg = StyleEditorDialog(self.curve_styles[key], self)
        if dlg.exec():
            self.curve_styles[key] = dlg.result_style
            self.update_curve_style(key)

    def toggle_fft_scale(self, checked):
        self.spin_fft_max.setEnabled(not checked)
        if checked:
            self.plot_fft.enableAutoRange(axis='y')
            if hasattr(self, 'plot_replay_fft'): self.plot_replay_fft.enableAutoRange(axis='y')
        else:
            self.update_fft_range()

    def update_fft_range(self):
        if not self.chk_fft_auto.isChecked():
            v = self.spin_fft_max.value()
            self.plot_fft.setYRange(0, v)
            if hasattr(self, 'plot_replay_fft'): self.plot_replay_fft.setYRange(0, v)

    def apply_styles(self):
        # Handled by apply_theme
        pass

    def refresh_ports(self):
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(p.device)

    def toggle_simulation(self, checked):
        self.is_simulating = checked
        if checked:
            if self.is_connected:
                self.toggle_connection() 
            self.action_connect.setEnabled(False)
            self.combo_ports.setEnabled(False)
        else:
            self.action_connect.setEnabled(True)
            self.combo_ports.setEnabled(True)

    def toggle_connection(self):
        if not self.is_connected:
            self.grp_mag.setChecked(False)
            self.grp_mag_hp.setChecked(False)
            self.grp_raw.setChecked(False)
            self.grp_cur.setChecked(False)
            self.grp_slip.setChecked(False)
            self.grp_srv.setChecked(False)
            self.chk_cmd_fft.setChecked(False)

            port = self.combo_ports.currentText()
            try:
                baud = int(self.combo_baud.currentText())
            except ValueError:
                baud = DEFAULT_BAUD_RATE
                
            if not port:
                return

            self.serial_thread = SerialWorker(port, baud)
            self.serial_thread.data_received.connect(self.handle_data_batch)
            self.serial_thread.raw_received.connect(self.handle_raw_batch)
            self.serial_thread.error_occurred.connect(self.handle_error)
            self.serial_thread.start()
            
            reset_cmds = [
                '{"fft": false}',
                '{"mag_raw": false}',
                '{"mag_filtered": false}',
                '{"mag_highpass": false}',
                '{"current": false}',
                '{"slip": false}',
                '{"servo": false}',
                '{"system": false}'
            ]
            for cmd in reset_cmds:
                self.serial_thread.send_command(cmd)
            
            self.is_connected = True
            self.action_connect.setText("Disconnect")
            self.chk_sim.setEnabled(False)
        else:
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread = None
            
            self.is_connected = False
            self.action_connect.setText("Connect")
            self.chk_sim.setEnabled(True)
            
            self.grp_mag.setChecked(False)
            self.grp_mag_hp.setChecked(False)
            self.grp_raw.setChecked(False)
            self.grp_cur.setChecked(False)
            self.grp_slip.setChecked(False)
            self.grp_srv.setChecked(False)
            self.chk_cmd_fft.setChecked(False)

    def handle_error(self, msg):
        self.text_raw.append(f"!! ERROR: {msg}")
        if self.is_connected:
            self.toggle_connection() 

    def handle_raw_batch(self, lines):
        if self.tabs.currentWidget() != self.tab_raw:
            return

        for line in lines[-10:]:
            self.text_raw.append(line)
        
        sb = self.text_raw.verticalScrollBar()
        sb.setValue(sb.maximum())

    def handle_data_batch(self, batch):
        for data in batch:
            self.process_data_point(data)

    def process_data_point(self, data):
        if self.is_recording and hasattr(self, 'recording_file_handle'):
             try:
                 self.recording_file_handle.write(json.dumps(data) + '\n')
             except Exception as e:
                 print(f"Rec write failed: {e}")
        
        current_time_ms = time.time() * 1000.0
        data['recv_ts'] = current_time_ms
        
        if 't' not in data or data['t'] == 0:
            data['t'] = current_time_ms

        if data.get('type') == 'fft':
            fft_vals = data.get('data', [])
            if fft_vals:
                self.process_external_fft(fft_vals)
            return

        self.append_data(data)

    def process_external_fft(self, fft_vals):
        sample_rate = self.spin_fft_rate.value()
        num_bins = len(fft_vals)
        if num_bins < 2: 
            return
            
        freqs = np.linspace(0, sample_rate / 2, num_bins)
        
        self.fft_data['freqs'] = freqs
        self.fft_data['mags'] = fft_vals
        
        try:
            idx_peak = np.argmax(fft_vals[1:]) + 1
            dom_freq = freqs[idx_peak]
            self.lbl_freq.setText(f"Dominant Freq: {dom_freq:.1f} Hz")
        except:
            pass

    def append_data(self, data):
        ts = data.get('t', 0)
        
        keys = ['mlx', 'mly', 'mlz', 'mag', 
                'mhx', 'mhy', 'mhz',
                'rmx', 'rmy', 'rmz', 
                'cur', 'slip', 's_ind', 
                'srv', 'grp']
                
        filtered_data = {}
        
        for key in keys:
            raw_val = data.get(key, 0.0)
            
            if key in self.spike_buffer:
                self.spike_buffer[key].append(raw_val)
                if len(self.spike_buffer[key]) > 3:
                    self.spike_buffer[key].pop(0)
            
            val_to_store = raw_val
            
            if self.chk_despike.isChecked() and key in self.spike_buffer:
                buf = self.spike_buffer[key]
                if len(buf) == 3:
                    val_to_store = sorted(buf)[1]
            
            filtered_data[key] = val_to_store

        for key in keys:
            self.data[key].append(filtered_data[key])
            
        self.data['timestamp'].append(ts)
        self.data['recv_ts'].append(data.get('recv_ts', 0))
        
        if len(self.data['timestamp']) > self.buffer_size + 1000:
            cutoff = len(self.data['timestamp']) - self.buffer_size
            for k in self.data:
                self.data[k] = self.data[k][cutoff:]

        if self.is_recording:
            record_packet = data.copy()
            for k, v in filtered_data.items():
                record_packet[k] = v
            self.recording_data.append(record_packet)

    def toggle_recording(self):
        if not self.is_recording:
            parent_dir = QFileDialog.getExistingDirectory(self, "Select Directory to Save Recording")
            if not parent_dir:
                self.btn_record.setChecked(False)
                return
                
            import os
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            folder_name = f"recording_{timestamp}"
            save_dir = os.path.join(parent_dir, folder_name)
            os.makedirs(save_dir, exist_ok=True)
            
            self.recording_file_path = os.path.join(save_dir, "raw_data.txt")
            
            try:
                self.recording_file_handle = open(self.recording_file_path, 'w', buffering=1) 
                
                self.is_recording = True
                self.recording_data = [] 
                self.recording_fft = []  
                
                self.btn_record.setText("Stop Recording")
                self.text_raw.append(f">> RECORDING STARTED: {folder_name}/raw_data.txt")
                
            except Exception as e:
                self.text_raw.append(f"!! RECORDING INIT FAILED: {e}")
                if hasattr(self, 'recording_file_handle'): self.recording_file_handle.close()
                self.btn_record.setChecked(False)

        else:
            self.is_recording = False
            self.btn_record.setText("Record")
            
            if hasattr(self, 'recording_file_handle'):
                self.recording_file_handle.close()
                self.recording_file_handle = None
                
            self.text_raw.append(f">> RECORDING STOPPED.")

    def update_loop(self):
        if self.is_simulating:
            for _ in range(3): 
                d = self.sim_generator.next_sample()
                self.append_data(d)
                
            if np.random.random() < 0.1:
                freqs = np.linspace(0, 50, 64)
                mags = np.random.random(64) * 10
                mags[10] = 50 
                self.fft_data['freqs'] = freqs
                self.fft_data['mags'] = mags

        if len(self.data['timestamp']) > 1:
            
            def update_plot_curves(curves, settings):
                visible_vals = []
                window_size = settings.spin_window.value()
                
                for key, curve in curves.items():
                    if key in self.data and curve.isVisible():
                        y_data = self.data[key][-window_size:]
                        curve.setData(y_data)
                        if settings.chk_center.isChecked():
                            visible_vals.extend(y_data)
                settings.apply_dc_center(visible_vals)

            if self.plot_time_1.isVisible():
                update_plot_curves(self.curves_p1, self.settings_p1)
                
            if self.plot_time_2.isVisible():
                update_plot_curves(self.curves_p2, self.settings_p2)
                    
        if len(self.fft_data['freqs']) > 0 and len(self.fft_data['mags']) > 0 and self.plot_fft.isVisible():
             self.curve_fft.setData(self.fft_data['freqs'], self.fft_data['mags'])
        else:
            pass

    def mark_event(self, label):
        if not self.data['timestamp']:
            return
        
        t = self.data['timestamp'][-1]
        self.recorded_events.append({
            'timestamp': t,
            'label': label,
            'data_index': len(self.data['timestamp'])
        })
        print(f"Event Marked: {label} at T={t}")

    def export_data(self):
        parent_dir = QFileDialog.getExistingDirectory(self, "Select Directory to Save Recording")
        if not parent_dir:
            return

        try:
            import os
            from datetime import datetime
            
            timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            folder_name = f"recording_{timestamp}"
            save_dir = os.path.join(parent_dir, folder_name)
            os.makedirs(save_dir, exist_ok=True)
            
            signals_path = os.path.join(save_dir, "signals.csv")
            
            with open(signals_path, 'w', newline='') as f:
                writer = csv.writer(f)
                keys = ['timestamp', 'recv_ts', 'mlx', 'mly', 'mlz', 'mag', 'mhx', 'mhy', 'mhz', 'rmx', 'rmy', 'rmz', 'cur', 'slip', 's_ind', 'srv', 'grp']
                writer.writerow(keys + ['label'])
                
                source_data = []
                
                if self.recording_data:
                    for row_data in self.recording_data:
                        row = []
                        t = row_data.get('t', 0)
                        row.append(t)
                        row.append(row_data.get('recv_ts', 0))
                        for k in keys[2:]: 
                            row.append(row_data.get(k, 0))
                        
                        lbl = ""
                        for ev in self.recorded_events:
                            if abs(ev['timestamp'] - t) < 1.0: 
                                lbl = ev['label']
                                break
                        row.append(lbl)
                        source_data.append(row)
                else:
                    events_map = {e['timestamp']: e['label'] for e in self.recorded_events}
                    
                    for i in range(len(self.data['timestamp'])):
                        t = self.data['timestamp'][i]
                        recv_t = self.data['recv_ts'][i] if i < len(self.data.get('recv_ts', [])) else 0
                        row = [t, recv_t] 
                        for k in keys[2:]:
                            row.append(self.data[k][i])
                        
                        lbl = events_map.get(t, "")
                        row.append(lbl)
                        source_data.append(row)
                        
                writer.writerows(source_data)
            
            print(f"Exported signals to {signals_path}")

            if self.recording_fft:
                fft_path = os.path.join(save_dir, "FFT.csv")
                    
                with open(fft_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    num_bins = len(self.recording_fft[0].get('data', []))
                    header = ['timestamp', 'recv_ts'] + [f'bin_{i}' for i in range(num_bins)]
                    writer.writerow(header)
                    
                    for frame in self.recording_fft:
                        t = frame.get('t', 0)
                        recv_ts = frame.get('recv_ts', 0)
                        bins = frame.get('data', [])
                        row = [t, recv_ts] + bins
                        writer.writerow(row)
                        
                print(f"Exported FFT data to {fft_path}")
            
            self.text_raw.append(f">> SAVED RECORDING: {folder_name}")

        except Exception as e:
            print(f"Export failed: {e}")
            self.text_raw.append(f"!! EXPORT FAILED: {e}")

    def load_replay_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Replay Data", "", "Recording (*.txt *.csv)")
        if not path:
            return
            
        try:
            self.replay_data = []
            self.replay_fft_data = []
            
            if path.endswith('.txt'):
                print("Loading RAW JSON recording...")
                with open(path, 'r') as f:
                    lines = f.readlines()
                    
                t_counter = 0
                for line in lines:
                    line = line.strip()
                    if not line: continue
                    try:
                        item = json.loads(line)
                        
                        if item.get('type') == 'fft':
                            item['t'] = float(t_counter)
                            self.replay_fft_data.append(item)
                        else:
                            t_counter += 1
                            item['t'] = float(t_counter)
                            
                            for k in ['mlx', 'mly', 'mlz', 'mag', 'mhx', 'mhy', 'mhz', 'rmx', 'rmy', 'rmz', 'cur', 'slip', 's_ind', 'srv', 'grp']:
                                if k not in item: item[k] = 0.0
                                
                            self.replay_data.append(item)
                    except json.JSONDecodeError:
                        pass
                        
                print(f"Loaded {len(self.replay_data)} samples and {len(self.replay_fft_data)} FFT frames from JSON.")
                
            else:
                temp_signals = []
                with open(path, 'r') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        item = {}
                        for k, v in row.items():
                            if k == 'label': item[k] = v
                            else:
                                try: item[k] = float(v)
                                except: item[k] = 0.0
                        
                        if 'mx' in row: item['mlx'] = row['mx']
                        if 'my' in row: item['mly'] = row['my']
                        if 'mz' in row: item['mlz'] = row['mz']
                        
                        temp_signals.append(item)
                
                t_counter = 0
                for item in temp_signals:
                    t_counter += 1
                    item['t'] = float(t_counter)
                    self.replay_data.append(item)
                
            if self.replay_data:
                self.configure_view_from_row(self.replay_data[0])
                
                self.slider_replay.setRange(0, len(self.replay_data) - 1)
                self.slider_replay.setValue(0)
                self.replay_index = 0
                self.update_replay_plot_snapshot()
                print(f"Replay Ready: {len(self.replay_data)} samples.")
                
        except Exception as e:
            print(f"Replay Load Failed: {e}")
            import traceback
            traceback.print_exc()

    def configure_view_from_row(self, row):
        def check_group(grp_chk, sub_chks, keys):
            has_data = False
            for k in keys:
                if abs(row.get(k, 0)) > 0.000001:
                    has_data = True
                    break
            
            if has_data:
                grp_chk.setChecked(True)
                for k in keys:
                    if k in sub_chks and abs(row.get(k, 0)) > 0.000001:
                         sub_chks[k][1].setChecked(True)
            else:
                grp_chk.setChecked(False)
        
        check_group(self.grp_mag, self.chk_mag, ['mlx', 'mly', 'mlz', 'mag'])
        check_group(self.grp_mag_hp, self.chk_mag_hp, ['mhx', 'mhy', 'mhz'])
        check_group(self.grp_raw, self.chk_raw, ['rmx', 'rmy', 'rmz'])
        check_group(self.grp_cur, self.chk_cur, ['cur'])
        check_group(self.grp_slip, self.chk_slip, ['slip', 's_ind'])
        check_group(self.grp_srv, self.chk_srv, ['srv', 'grp'])

    def toggle_replay(self):
        if self.replay_timer.isActive():
            self.replay_timer.stop()
            self.btn_play_replay.setText("Play")
        else:
            if not self.replay_data:
                return
            self.replay_timer.start(33) 
            self.btn_play_replay.setText("Pause")

    def on_replay_slider_move(self, val):
        self.replay_index = val
        self.update_replay_plot_snapshot()

    def update_replay_loop(self):
        if self.replay_index < len(self.replay_data) - 1:
            self.replay_index += 1
            self.slider_replay.setValue(self.replay_index)
            self.update_replay_plot_snapshot()
        else:
            self.toggle_replay() 

    def update_replay_plot_snapshot(self):
        if not self.replay_data:
            return
            
        window = self.settings_replay.spin_window.value()
        start_idx = max(0, self.replay_index - window)
        end_idx = self.replay_index + 1
        
        subset = self.replay_data[start_idx:end_idx]
        
        t = [d.get('t', 0) for d in subset]
        
        visible_values = []

        def update_replay_curves(curves, settings):
            for key, curve in curves.items():
                if curve.isVisible():
                    y = [d.get(key, 0) for d in subset]
                    curve.setData(t, y)
                    if settings.chk_center.isChecked():
                        visible_values.extend(y)

        update_replay_curves(self.replay_curves_p1, self.settings_replay) # Replay Settings used for P1
        if self.settings_replay_2.isVisible():
             update_replay_curves(self.replay_curves_p2, self.settings_replay_2)

        self.settings_replay.apply_dc_center(visible_values)
        
        cur_t = self.replay_data[self.replay_index].get('t', 0)
        
        if self.replay_fft_data:
            best_frame = None
            for frame in self.replay_fft_data:
                if frame['t'] > cur_t:
                    break
                best_frame = frame
                
            if best_frame:
                fft_vals = best_frame['data']
                sample_rate = self.spin_fft_rate.value()
                num_bins = len(fft_vals)
                freqs = np.linspace(0, sample_rate / 2, num_bins)
                self.curve_replay_fft.setData(freqs, fft_vals)
            else:
                self.curve_replay_fft.clear()
        
        self.lbl_replay_time.setText(f"{cur_t:.2f} ms")
        
        if t:
            self.plot_replay.setXRange(min(t), max(t) + 0.1, padding=0)
            if hasattr(self, 'plot_replay_2') and self.plot_replay_2.isVisible():
                self.plot_replay_2.setXRange(min(t), max(t) + 0.1, padding=0)

    def import_data(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import Data", "", "CSV Files (*.csv);;JSON Files (*.json)")
        if not path:
            return

        try:
            for k in self.data:
                self.data[k] = []
            self.recorded_events = []
            
            if path.endswith('.csv'):
                with open(path, 'r') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            t = float(row.get('timestamp', 0))
                            lbl = row.get('label', '')
                            
                            row_mapped = row.copy()
                            if 'mx' in row: row_mapped['mlx'] = row['mx']
                            if 'my' in row: row_mapped['mly'] = row['my']
                            if 'mz' in row: row_mapped['mlz'] = row['mz']

                            for k in ['mlx', 'mly', 'mlz', 'mag', 'mhx', 'mhy', 'mhz', 'rmx', 'rmy', 'rmz', 'cur', 'slip', 's_ind', 'srv', 'grp']:
                                self.data[k].append(float(row_mapped.get(k, 0)))
                            
                            self.data['timestamp'].append(t)
                            
                            if lbl:
                                self.recorded_events.append({'timestamp': t, 'label': lbl})
                        except ValueError:
                            continue
                            
            elif path.endswith('.json'):
                with open(path, 'r') as f:
                    content = json.load(f)
                    if isinstance(content, list):
                        for item in content:
                            self.append_data(item)
                    elif isinstance(content, dict) and 'data' in content:
                         pass
                         
            print(f"Imported from {path}")
            self.update_loop()
            
        except Exception as e:
            print(f"Import failed: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion") 
    
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(COLOR_BG))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.Base, QColor(COLOR_PANEL))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(COLOR_BG))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.Text, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.Button, QColor(COLOR_PANEL))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(COLOR_TEXT))
    palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    palette.setColor(QPalette.ColorRole.Link, QColor(COLOR_ACCENT_1))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(COLOR_ACCENT_1))
    palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
    app.setPalette(palette)
    
    window = AdaptiveGripperGUI()
    window.show()
    sys.exit(app.exec())