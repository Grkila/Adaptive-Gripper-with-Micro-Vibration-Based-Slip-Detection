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
                             QMenuBar, QMenu, QToolBar, QLineEdit, QListWidget, QSizePolicy)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QThread, QObject, QPoint, QRect
from PyQt6.QtGui import QColor, QPalette, QFont, QAction, QPainter, QBrush, QPen, QRadialGradient, QLinearGradient, QConicalGradient
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
# Main Application
# ==========================================
class StyleEditorDialog(QDialog):
    def __init__(self, current_style, parent=None, threshold_data=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Plot Style")
        self.result_style = current_style.copy()
        self.result_threshold = None
        
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
        
        # --- Threshold Section ---
        
        # Spacer
        form.addRow(QLabel(""))
        form.addRow(QLabel("<b>Threshold Alert</b>"))
        
        self.chk_thresh = QCheckBox("Enable Threshold")
        form.addRow("", self.chk_thresh)
        
        self.spin_thresh = QDoubleSpinBox()
        self.spin_thresh.setRange(-10000, 10000)
        self.spin_thresh.setDecimals(2)
        self.spin_thresh.setEnabled(False)
        form.addRow("Value:", self.spin_thresh)
        
        self.edit_thresh_name = QLineEdit()
        self.edit_thresh_name.setPlaceholderText("Alert Name")
        self.edit_thresh_name.setEnabled(False)
        form.addRow("Label:", self.edit_thresh_name)

        self.chk_thresh.toggled.connect(self.toggle_thresh_fields)
        
        if threshold_data:
            self.chk_thresh.setChecked(True)
            self.spin_thresh.setValue(threshold_data.get('value', 0))
            self.edit_thresh_name.setText(threshold_data.get('name', ''))
            self.original_threshold_id = threshold_data.get('id')
        else:
            self.original_threshold_id = None
            
        layout.addLayout(form)
        
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept_data)
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

    def toggle_thresh_fields(self, checked):
        self.spin_thresh.setEnabled(checked)
        self.edit_thresh_name.setEnabled(checked)

    def accept_data(self):
        if self.chk_thresh.isChecked():
            # Create threshold dict
            self.result_threshold = {
                'value': self.spin_thresh.value(),
                'name': self.edit_thresh_name.text() if self.edit_thresh_name.text() else "Alert",
                'color': self.result_style['color'], # Inherit color
                'style': Qt.PenStyle.DashLine, # Default style for threshold line
                'id': self.original_threshold_id # Preserve ID if editing
            }
        else:
            self.result_threshold = None # Signal to delete if it existed
            
        self.accept()

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

class LampWidget(QWidget):
    """Industrial-style indicator lamp widget"""
    def __init__(self, name, color, parent=None):
        super().__init__(parent)
        self.name = name
        self.base_color = QColor(color)
        self.active = False
        self.setMinimumSize(130, 150)
        self.setMaximumSize(130, 150)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        
    def set_status(self, active):
        if self.active != active:
            self.active = active
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            
            w = self.width()
            h = self.height()
            
            # Lamp layout parameters
            cx = w // 2
            cy = 55  # Fixed from top
            lamp_radius = 32
            
            # === METAL HOUSING/BEZEL ===
            # Outer bezel ring with metallic gradient
            bezel_outer = 38
            bezel_grad = QLinearGradient(cx - bezel_outer, cy - bezel_outer, 
                                         cx + bezel_outer, cy + bezel_outer)
            bezel_grad.setColorAt(0.0, QColor("#d0d0d0"))
            bezel_grad.setColorAt(0.3, QColor("#f5f5f5"))
            bezel_grad.setColorAt(0.7, QColor("#888888"))
            bezel_grad.setColorAt(1.0, QColor("#505050"))
            
            painter.setBrush(QBrush(bezel_grad))
            painter.setPen(QPen(QColor("#222"), 2))
            painter.drawEllipse(QPoint(cx, cy), bezel_outer, bezel_outer)
            
            # Inner black seal ring
            painter.setBrush(QBrush(QColor("#181818")))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(QPoint(cx, cy), lamp_radius + 2, lamp_radius + 2)
            
            # === LAMP LENS/BULB ===
            if self.active:
                # ACTIVE STATE: Bright, glowing
                # Inner hotspot gradient
                bulb_grad = QRadialGradient(cx - 8, cy - 8, lamp_radius * 1.8)
                bulb_grad.setColorAt(0.0, QColor("#ffffff"))
                bulb_grad.setColorAt(0.15, self.base_color.lighter(180))
                bulb_grad.setColorAt(0.5, self.base_color)
                bulb_grad.setColorAt(0.85, self.base_color.darker(140))
                bulb_grad.setColorAt(1.0, self.base_color.darker(180))
                
                painter.setBrush(QBrush(bulb_grad))
                painter.setPen(QPen(QColor("#0a0a0a"), 1))
                painter.drawEllipse(QPoint(cx, cy), lamp_radius, lamp_radius)
                
                # Outer glow halo
                for i in range(3):
                    glow_c = QColor(self.base_color)
                    glow_c.setAlpha(30 - i * 8)
                    painter.setBrush(Qt.BrushStyle.NoBrush)
                    painter.setPen(QPen(glow_c, 3 - i))
                    r = lamp_radius + 6 + i * 3
                    painter.drawEllipse(QPoint(cx, cy), r, r)
                    
            else:
                # INACTIVE STATE: Dull, dark
                off_color = QColor(self.base_color)
                off_color.setHsv(off_color.hue(), 
                                off_color.saturation() // 3, 
                                35)
                
                bulb_grad = QRadialGradient(cx - 10, cy - 10, lamp_radius * 1.5)
                bulb_grad.setColorAt(0.0, off_color.lighter(160))
                bulb_grad.setColorAt(0.6, off_color)
                bulb_grad.setColorAt(1.0, off_color.darker(250))
                
                painter.setBrush(QBrush(bulb_grad))
                painter.setPen(QPen(QColor("#0a0a0a"), 1))
                painter.drawEllipse(QPoint(cx, cy), lamp_radius, lamp_radius)

            # === GLASS REFLECTION/GLOSS ===
            # Top highlight (curved reflection)
            hi_w = int(lamp_radius * 0.7)
            hi_h = int(lamp_radius * 0.4)
            hi_x = cx - hi_w // 2
            hi_y = int(cy - lamp_radius * 0.5)
            
            highlight_grad = QLinearGradient(hi_x, hi_y, hi_x, hi_y + hi_h)
            c_high = QColor(255, 255, 255, 150 if self.active else 80)
            c_trans = QColor(255, 255, 255, 0)
            highlight_grad.setColorAt(0.0, c_high)
            highlight_grad.setColorAt(1.0, c_trans)
            
            painter.setBrush(QBrush(highlight_grad))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawEllipse(hi_x, hi_y, hi_w, hi_h)

            # === LABEL NAMEPLATE ===
            plate_y = h - 42
            plate_w = w - 12
            plate_h = 28
            plate_x = 6
            
            # Screw holes (decorative)
            screw_d = 6
            painter.setBrush(QBrush(QColor("#0a0a0a")))
            painter.setPen(QPen(QColor("#555"), 1))
            painter.drawEllipse(plate_x + 2, plate_y + plate_h//2 - screw_d//2, screw_d, screw_d)
            painter.drawEllipse(plate_x + plate_w - screw_d - 2, plate_y + plate_h//2 - screw_d//2, screw_d, screw_d)
            
            # Nameplate body with engraved look
            label_rect = QRect(plate_x + 12, plate_y, plate_w - 24, plate_h)
            
            # Plate background (brushed metal effect)
            plate_grad = QLinearGradient(label_rect.left(), label_rect.top(), 
                                         label_rect.right(), label_rect.bottom())
            plate_grad.setColorAt(0.0, QColor("#4a4a4a"))
            plate_grad.setColorAt(0.5, QColor("#3a3a3a"))
            plate_grad.setColorAt(1.0, QColor("#2a2a2a"))
            
            painter.setBrush(QBrush(plate_grad))
            painter.setPen(QPen(QColor("#555"), 1))
            painter.drawRoundedRect(label_rect, 3, 3)
            
            # Engraved border
            painter.setPen(QPen(QColor("#1a1a1a"), 1))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            inner_rect = label_rect.adjusted(2, 2, -2, -2)
            painter.drawRoundedRect(inner_rect, 2, 2)
            
            # Label text (engraved look with shadow)
            # Shadow
            painter.setPen(QColor("#0a0a0a"))
            font = painter.font()
            font.setFamily("Arial")
            font.setBold(True)
            font.setPointSize(8)
            painter.setFont(font)
            shadow_rect = label_rect.adjusted(1, 1, 1, 1)
            painter.drawText(shadow_rect, Qt.AlignmentFlag.AlignCenter, self.name)
            
            # Main text
            painter.setPen(QColor("#cccccc"))
            painter.drawText(label_rect, Qt.AlignmentFlag.AlignCenter, self.name)
            
        except Exception as e:
            print(f"LampWidget paint error: {e}")
        finally:
            painter.end()

class LampPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent, Qt.WindowType.Window)
        self.setWindowTitle("THRESHOLD MONITOR PANEL")
        self.resize(550, 450)
        
        # Industrial Control Panel Style
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #3a3a3a, stop:1 #2b2b2b);
                font-family: 'Segoe UI', 'Arial';
                border: 2px solid #555;
            }
        """)
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(15, 15, 15, 15)
        
        # Title Bar
        title_label = QLabel("THRESHOLD STATUS")
        title_label.setStyleSheet("""
            QLabel {
                background-color: #222;
                color: #0ff;
                font-size: 14pt;
                font-weight: bold;
                font-family: 'Consolas', 'Courier New';
                padding: 8px;
                border: 2px solid #444;
                border-radius: 3px;
                letter-spacing: 2px;
            }
        """)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title_label)
        
        # Lamp Grid Container with industrial panel look
        lamp_container = QWidget()
        lamp_container.setStyleSheet("""
            QWidget {
                background-color: #353535;
                border: 3px inset #444;
                border-radius: 5px;
            }
        """)
        
        self.layout = QGridLayout(lamp_container)
        self.layout.setSpacing(15)
        self.layout.setContentsMargins(20, 20, 20, 20)
        
        main_layout.addWidget(lamp_container, stretch=1)
        
        self.lamps = {}  # id -> LampWidget
        
    def update_lamps(self, thresholds, active_ids):
        # Remove old lamps not in thresholds
        current_ids = set(t['id'] for t in thresholds)
        for lid in list(self.lamps.keys()):
            if lid not in current_ids:
                self.layout.removeWidget(self.lamps[lid])
                self.lamps[lid].deleteLater()
                del self.lamps[lid]
        
        # Add/Update lamps in grid
        row, col = 0, 0
        max_cols = 4
        
        for i, thresh in enumerate(thresholds):
            lid = thresh['id']
            
            if lid not in self.lamps:
                lamp = LampWidget(thresh['name'], thresh['color'])
                self.lamps[lid] = lamp
            else:
                # Update existing lamp properties
                self.lamps[lid].name = thresh['name']
                self.lamps[lid].base_color = QColor(thresh['color'])
            
            # Position in grid
            self.layout.addWidget(self.lamps[lid], row, col)
            col += 1
            if col >= max_cols:
                col = 0
                row += 1
            
            # Update lamp status (on/off)
            self.lamps[lid].set_status(lid in active_ids)

class ThresholdEditorDialog(QDialog):
    def __init__(self, signals, current_data=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Threshold")
        self.result_data = {}
        
        layout = QVBoxLayout(self)
        form = QFormLayout()
        
        self.combo_signal = QComboBox()
        for s in signals:
            self.combo_signal.addItem(s)
            
        self.spin_value = QDoubleSpinBox()
        self.spin_value.setRange(-10000, 10000)
        self.spin_value.setDecimals(2)
        
        self.edit_name = QLineEdit()
        self.edit_name.setPlaceholderText("e.g. Max Current")
        
        self.btn_color = QPushButton()
        self.color = "#ff0000"
        self.btn_color.setStyleSheet(f"background-color: {self.color}")
        self.btn_color.clicked.connect(self.pick_color)
        
        self.combo_style = QComboBox()
        self.styles = {
            "Solid": Qt.PenStyle.SolidLine,
            "Dash": Qt.PenStyle.DashLine,
            "Dot": Qt.PenStyle.DotLine
        }
        for k, v in self.styles.items():
            self.combo_style.addItem(k, v)
            
        form.addRow("Signal:", self.combo_signal)
        form.addRow("Threshold Value:", self.spin_value)
        form.addRow("Label Name:", self.edit_name)
        form.addRow("Color:", self.btn_color)
        form.addRow("Line Style:", self.combo_style)
        
        layout.addLayout(form)
        
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.validate)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        
        if current_data:
            idx = self.combo_signal.findText(current_data['signal'])
            if idx >= 0: self.combo_signal.setCurrentIndex(idx)
            self.spin_value.setValue(current_data['value'])
            self.edit_name.setText(current_data['name'])
            self.color = current_data['color']
            self.btn_color.setStyleSheet(f"background-color: {self.color}")
            
            idx_style = self.combo_style.findData(current_data['style'])
            if idx_style >= 0: self.combo_style.setCurrentIndex(idx_style)

    def pick_color(self):
        c = QColorDialog.getColor(QColor(self.color), self, "Select Color")
        if c.isValid():
            self.color = c.name()
            self.btn_color.setStyleSheet(f"background-color: {self.color}")

    def validate(self):
        if not self.edit_name.text().strip():
            self.edit_name.setText(f"{self.combo_signal.currentText()} > {self.spin_value.value()}")
            
        self.result_data = {
            'signal': self.combo_signal.currentText(),
            'value': self.spin_value.value(),
            'name': self.edit_name.text(),
            'color': self.color,
            'style': self.combo_style.currentData()
        }
        self.accept()

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
        
        self.fft_data = {'freqs': [], 'mags': [], 'freq_resolution': 1.0, 'fft_size': 0}
        self.recorded_events = []
        self.current_segment_start = None

        # Thresholds
        self.thresholds = [] 
        self.threshold_lines = {} 
        self.lamp_panel = LampPanel()
        
        # Recording & Replay
        self.is_recording = False
        self.recording_data = [] 
        self.recording_fft = [] 
        self.replay_data = []    
        self.replay_fft_data = [] 
        self.replay_index = 0
        
        self.serial_thread = None
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
        
        h_rate = QHBoxLayout()
        h_rate.addWidget(QLabel("Sample Rate (Hz):"))
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
        
        self.lbl_freq = QLabel("Waiting for FFT data...")
        self.lbl_freq.setWordWrap(True)
        self.lbl_freq.setStyleSheet("font-size: 9pt;")
        v_ana.addWidget(self.lbl_freq)
        
        grp_ana.setLayout(v_ana)
        sidebar_layout.addWidget(grp_ana)
        
        # --- Thresholds ---
        grp_thresh = QGroupBox("Thresholds")
        v_thresh = QVBoxLayout()
        
        self.list_thresholds = QListWidget()
        self.list_thresholds.setFixedHeight(100)
        v_thresh.addWidget(self.list_thresholds)
        
        h_btns = QHBoxLayout()
        btn_add = QPushButton("Add")
        btn_add.clicked.connect(self.add_threshold)
        btn_edit = QPushButton("Edit")
        btn_edit.clicked.connect(self.edit_threshold)
        btn_del = QPushButton("Del")
        btn_del.clicked.connect(self.remove_threshold)
        h_btns.addWidget(btn_add)
        h_btns.addWidget(btn_edit)
        h_btns.addWidget(btn_del)
        v_thresh.addLayout(h_btns)
        
        btn_lamps = QPushButton("Show Lamp Panel")
        btn_lamps.clicked.connect(self.toggle_lamp_panel)
        v_thresh.addWidget(btn_lamps)
        
        grp_thresh.setLayout(v_thresh)
        sidebar_layout.addWidget(grp_thresh)
        # ------------------
        
        sidebar_layout.addStretch()
        main_layout.addWidget(scroll_sidebar)

        self.tabs = QTabWidget()
        
        tab_viz = QWidget()
        layout_viz = QVBoxLayout(tab_viz)
        
        self.viz_splitter = QSplitter(Qt.Orientation.Vertical)
        self.viz_splitter.addWidget(self.plot_time_1)
        self.viz_splitter.addWidget(self.plot_time_2)
        
        self.plot_fft = pg.PlotWidget(title="Real-Time FFT (DTFT Style)")
        self.plot_fft.showGrid(x=True, y=True, alpha=0.5)
        self.plot_fft.setLabel('bottom', "Frequency", units='Hz')
        self.plot_fft.setLabel('left', "Magnitude")
        self.plot_fft.getAxis('bottom').setTicks([])  # We'll set custom ticks
        
        # Add vertical line at 30Hz for reference (highpass cutoff)
        self.fft_ref_line_30hz = pg.InfiniteLine(angle=90, pos=30, 
                                                  pen=pg.mkPen('r', width=2, style=pg.QtCore.Qt.PenStyle.DashLine),
                                                  label='30Hz Cutoff',
                                                  labelOpts={'position':0.95, 'color': (255, 0, 0)})
        self.plot_fft.addItem(self.fft_ref_line_30hz)
        
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

    def add_threshold(self):
        # Use dynamic keys if possible, otherwise fallback
        signals = sorted(list(self.curve_styles.keys()))
        dlg = ThresholdEditorDialog(signals, parent=self)
        if dlg.exec():
            data = dlg.result_data
            data['id'] = str(time.time())
            self.thresholds.append(data)
            self.update_threshold_list()
            self.update_threshold_lines()
            self.lamp_panel.update_lamps(self.thresholds, [])

    def edit_threshold(self):
        row = self.list_thresholds.currentRow()
        if row < 0: return
        
        old_data = self.thresholds[row]
        signals = sorted(list(self.curve_styles.keys()))
        dlg = ThresholdEditorDialog(signals, current_data=old_data, parent=self)
        if dlg.exec():
            data = dlg.result_data
            data['id'] = old_data['id']
            self.thresholds[row] = data
            self.update_threshold_list()
            self.update_threshold_lines()
            self.lamp_panel.update_lamps(self.thresholds, [])

    def remove_threshold(self):
        row = self.list_thresholds.currentRow()
        if row < 0: return
        
        del self.thresholds[row]
        self.update_threshold_list()
        self.update_threshold_lines()
        self.lamp_panel.update_lamps(self.thresholds, [])

    def update_threshold_list(self):
        self.list_thresholds.clear()
        for t in self.thresholds:
            self.list_thresholds.addItem(f"{t['name']} ({t['signal']} > {t['value']})")

    def toggle_lamp_panel(self):
        if self.lamp_panel.isVisible():
            self.lamp_panel.hide()
        else:
            self.lamp_panel.show()

    def update_threshold_lines(self):
        # Clear all existing threshold lines from both plots
        for line_set in self.threshold_lines.values():
            for line in line_set:
                try:
                    self.plot_time_1.removeItem(line)
                except: 
                    pass
                try:
                    self.plot_time_2.removeItem(line)
                except: 
                    pass
        self.threshold_lines.clear()
        
        # Don't create lines if in FFT mode
        if self.action_show_fft.isChecked():
            return
        
        # Recreate threshold lines based on current signal visibility
        for t in self.thresholds:
            sig = t['signal']
            
            # Check if this signal curve exists and is visible on each plot
            visible_p1 = (sig in self.curves_p1 and 
                         self.curves_p1[sig].isVisible() and 
                         self.plot_time_1.isVisible())
            
            visible_p2 = (sig in self.curves_p2 and 
                         self.curves_p2[sig].isVisible() and 
                         self.plot_time_2.isVisible())
            
            lines_created = []
            
            # Create line for Plot 1 if signal is visible there
            if visible_p1:
                pen1 = pg.mkPen(color=t['color'], width=2, style=t['style'])
                opts1 = {'position': 0.1, 'color': t['color'], 'movable': False}
                line_p1 = pg.InfiniteLine(angle=0, pos=t['value'], pen=pen1, label=t['name'], labelOpts=opts1)
                line_p1.setZValue(100)  # Ensure threshold lines are on top
                self.plot_time_1.addItem(line_p1)
                lines_created.append(line_p1)
                
            # Create line for Plot 2 if signal is visible there
            if visible_p2:
                pen2 = pg.mkPen(color=t['color'], width=2, style=t['style'])
                opts2 = {'position': 0.1, 'color': t['color'], 'movable': False}
                line_p2 = pg.InfiniteLine(angle=0, pos=t['value'], pen=pen2, label=t['name'], labelOpts=opts2)
                line_p2.setZValue(100)  # Ensure threshold lines are on top
                self.plot_time_2.addItem(line_p2)
                lines_created.append(line_p2)
            
            # Store references to created lines
            if lines_created:
                self.threshold_lines[t['id']] = lines_created

    def check_thresholds(self):
        """Check threshold conditions and update lamp panel in real-time"""
        if not self.thresholds: 
            return
        
        active_ids = set()
        visible_thresholds = []
        
        for t in self.thresholds:
            sig = t['signal']
            
            # Check if signal is active/visible on EITHER plot
            is_enabled = False
            if sig in self.curves_p1 and self.curves_p1[sig].isVisible(): 
                is_enabled = True
            if sig in self.curves_p2 and self.curves_p2[sig].isVisible(): 
                is_enabled = True
            
            # Only process visible thresholds
            if is_enabled:
                visible_thresholds.append(t)
                
                # Check if threshold is exceeded
                if sig in self.data and self.data[sig]:
                    val = self.data[sig][-1]
                    if val > t['value']:
                        active_ids.add(t['id'])
        
        # Only show lamps for thresholds with visible signals
        self.lamp_panel.update_lamps(visible_thresholds, active_ids)


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
        
        # Update threshold lines when plot visibility changes
        self.update_threshold_lines()
    
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
        
        self.plot_replay_fft = pg.PlotWidget(title="Replay FFT (DTFT Style)")
        self.plot_replay_fft.showGrid(x=True, y=True, alpha=0.3)
        self.plot_replay_fft.setLabel('bottom', "Frequency", units='Hz')
        self.plot_replay_fft.setLabel('left', "Magnitude")
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

        # Replay FFT Bar Graph with border for better visibility
        self.curve_replay_fft = pg.BarGraphItem(
            x=[], height=[], width=1.0, 
            brush=COLOR_ACCENT_1, 
            pen=pg.mkPen(COLOR_ACCENT_1, width=1)
        )
        self.plot_replay_fft.addItem(self.curve_replay_fft)
        
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
                # Update threshold lines when group is toggled
                self.update_threshold_lines()
            
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
        def toggle_fft_mode(checked):
            self.send_stream_command("fft", checked)
            # Automatically toggle the view
            self.action_show_fft.setChecked(checked)
            
        self.chk_cmd_fft.toggled.connect(toggle_fft_mode)
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
                
        # Update thresholds visibility immediately
        self.update_threshold_lines()

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

        # FFT Bar Graph with border for better visibility
        self.curve_fft = pg.BarGraphItem(
            x=[], height=[], width=1.0, 
            brush=COLOR_ACCENT_1, 
            pen=pg.mkPen(COLOR_ACCENT_1, width=1)
        )
        self.plot_fft.addItem(self.curve_fft)

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
            
        # Find existing threshold for this key (if any)
        current_threshold = None
        for t in self.thresholds:
            if t['signal'] == key:
                current_threshold = t
                break
            
        dlg = StyleEditorDialog(self.curve_styles[key], self, threshold_data=current_threshold)
        if dlg.exec():
            # Update Style
            self.curve_styles[key] = dlg.result_style
            self.update_curve_style(key)
            
            # Update Threshold
            new_thresh = dlg.result_threshold
            
            if current_threshold and not new_thresh:
                # Removed
                self.thresholds = [t for t in self.thresholds if t['id'] != current_threshold['id']]
            elif not current_threshold and new_thresh:
                # Added
                new_thresh['signal'] = key
                new_thresh['id'] = str(time.time())
                self.thresholds.append(new_thresh)
            elif current_threshold and new_thresh:
                # Updated
                new_thresh['signal'] = key
                # Update in place
                for i, t in enumerate(self.thresholds):
                    if t['id'] == current_threshold['id']:
                        self.thresholds[i] = new_thresh
                        break
            
            self.update_threshold_list()
            self.update_threshold_lines()
            self.lamp_panel.update_lamps(self.thresholds, [])
    
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
        else:
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread = None
            
            self.is_connected = False
            self.action_connect.setText("Connect")
            
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
        """Process FFT data received from device"""
        num_bins = len(fft_vals)
        if num_bins < 2: 
            return
        
        # Get sample rate from config
        sample_rate = self.spin_fft_rate.value()
        
        # Calculate frequency bins dynamically
        # Arduino sends FFT_SAMPLES/2 bins from a full FFT_SAMPLES-point FFT
        # So if we receive num_bins, the original FFT size is num_bins * 2
        # Frequency resolution = sample_rate / fft_size
        fft_size = num_bins * 2
        freq_resolution = sample_rate / fft_size
        
        # Calculate center frequency for each bin
        freqs = np.arange(num_bins) * freq_resolution
        
        # Convert to numpy array for consistency
        mags = np.array(fft_vals)
        
        # Debug: Print first few bins to verify frequency mapping
        print(f"\n=== FFT Debug Info ===")
        print(f"Sample Rate: {sample_rate} Hz")
        print(f"Num Bins: {num_bins}")
        print(f"FFT Size: {fft_size}")
        print(f"Freq Resolution: {freq_resolution:.3f} Hz")
        print(f"Max Frequency: {freqs[-1]:.1f} Hz (should be ~{sample_rate/2:.1f} Hz)")
        print(f"\nFirst 10 bins:")
        for i in range(min(10, num_bins)):
            print(f"  Bin {i}: {freqs[i]:.2f} Hz -> Mag {mags[i]:.3f}")
        if num_bins > 10:
            print(f"  ...")
            print(f"  Bin {num_bins-1}: {freqs[-1]:.2f} Hz -> Mag {mags[-1]:.3f}")
        
        # Store FFT data
        self.fft_data['freqs'] = freqs
        self.fft_data['mags'] = mags
        self.fft_data['freq_resolution'] = freq_resolution
        self.fft_data['fft_size'] = fft_size
        
        # Find dominant frequency (skip DC component at index 0)
        try:
            if len(mags) > 1:
                idx_peak = np.argmax(mags[1:]) + 1
                dom_freq = freqs[idx_peak]
                dom_mag = mags[idx_peak]
                
                # Also show first few bin magnitudes for highpass filter verification
                first_bins_info = f"Bins 0-2: {mags[0]:.2f}, {mags[1]:.2f}, {mags[2]:.2f}" if len(mags) > 2 else ""
                
                self.lbl_freq.setText(
                    f"Dominant: {dom_freq:.1f} Hz (Mag: {dom_mag:.1f})\n"
                    f"Bins: {num_bins} | FFT Size: {fft_size} | Res: {freq_resolution:.2f} Hz\n"
                    f"{first_bins_info}"
                )
        except Exception as e:
            print(f"FFT peak detection error: {e}")
            self.lbl_freq.setText(f"FFT Bins: {num_bins} | FFT Size: {fft_size}")

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
                    
        # Update FFT plot if data is available and plot is visible
        if self.plot_fft.isVisible():
            if len(self.fft_data['freqs']) > 0 and len(self.fft_data['mags']) > 0:
                # Ensure arrays are same length to prevent jitter
                if len(self.fft_data['freqs']) == len(self.fft_data['mags']):
                    freqs = self.fft_data['freqs']
                    mags = self.fft_data['mags']
                    freq_res = self.fft_data.get('freq_resolution', 1.0)
                    
                    # Update bar graph - each bin is a bar
                    # Width of each bar is the frequency resolution (80% to leave gaps)
                    self.curve_fft.setOpts(x=freqs, height=mags, width=freq_res * 0.8)
                    
                    # Set x-axis range to show all bins
                    self.plot_fft.setXRange(freqs[0] - freq_res, freqs[-1] + freq_res, padding=0.02)
                    
                    # Update x-axis ticks to show frequency values
                    # Prioritize showing low frequencies clearly (0-50Hz region important for 30Hz cutoff)
                    num_bins = len(freqs)
                    
                    # Always show important frequencies: 0, 10, 20, 30, 40, 50Hz and then subsample
                    important_freqs = [0, 10, 20, 30, 40, 50, 100, 150, 200]
                    tick_indices = []
                    
                    for imp_f in important_freqs:
                        # Find closest bin to this frequency
                        idx = np.argmin(np.abs(freqs - imp_f))
                        if idx < num_bins and idx not in tick_indices:
                            tick_indices.append(idx)
                    
                    # Add additional ticks for higher frequencies
                    if num_bins > 20:
                        step = max(1, num_bins // 20)
                        for i in range(0, num_bins, step):
                            if i not in tick_indices:
                                tick_indices.append(i)
                    else:
                        # Show all bins
                        tick_indices = list(range(num_bins))
                    
                    tick_indices = sorted(set(tick_indices))
                    
                    # Major ticks with frequency labels
                    major_ticks = [(freqs[i], f"{freqs[i]:.1f}") for i in tick_indices]
                    # Minor ticks for all bins (without labels for cleaner look)
                    minor_ticks = [(freqs[i], "") for i in range(num_bins) if i not in tick_indices]
                    
                    self.plot_fft.getAxis('bottom').setTicks([major_ticks, minor_ticks])
            
        self.check_thresholds()

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
                
                # Auto-detect FFT
                has_fft = len(self.replay_fft_data) > 0
                self.chk_replay_fft.setChecked(has_fft)
                
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
            
        window = self.settings_replay_1.spin_window.value()
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

        update_replay_curves(self.replay_curves_p1, self.settings_replay_1) # Replay Settings used for P1
        if self.settings_replay_2.isVisible():
             update_replay_curves(self.replay_curves_p2, self.settings_replay_2)

        self.settings_replay_1.apply_dc_center(visible_values)
        
        cur_t = self.replay_data[self.replay_index].get('t', 0)
        
        # Update replay FFT if available
        if self.replay_fft_data:
            best_frame = None
            for frame in self.replay_fft_data:
                if frame['t'] > cur_t:
                    break
                best_frame = frame
                
            if best_frame and 'data' in best_frame:
                fft_vals = best_frame['data']
                sample_rate = self.spin_fft_rate.value()
                num_bins = len(fft_vals)
                
                # Calculate frequency bins dynamically (must match FFT processing)
                # Arduino sends FFT_SAMPLES/2 bins from FFT_SAMPLES-point FFT
                fft_size = num_bins * 2
                freq_resolution = sample_rate / fft_size
                freqs = np.arange(num_bins) * freq_resolution
                mags = np.array(fft_vals)
                
                # Update bar graph
                self.curve_replay_fft.setOpts(x=freqs, height=mags, width=freq_resolution * 0.8)
                
                # Set x-axis range to show all bins
                self.plot_replay_fft.setXRange(freqs[0] - freq_resolution, freqs[-1] + freq_resolution, padding=0.02)
                
                # Update x-axis ticks
                if num_bins <= 20:
                    tick_indices = range(num_bins)
                else:
                    step = max(1, num_bins // 20)
                    tick_indices = range(0, num_bins, step)
                
                # Major ticks with frequency labels
                major_ticks = [(freqs[i], f"{freqs[i]:.1f}") for i in tick_indices]
                # Minor ticks for all bins
                minor_ticks = [(freqs[i], "") for i in range(num_bins) if i not in tick_indices]
                
                self.plot_replay_fft.getAxis('bottom').setTicks([major_ticks, minor_ticks])
            else:
                self.curve_replay_fft.setOpts(x=[], height=[])
        else:
            self.curve_replay_fft.setOpts(x=[], height=[])
        
        self.lbl_replay_time.setText(f"{cur_t:.2f} ms")
        
        if t:
            self.plot_replay_1.setXRange(min(t), max(t) + 0.1, padding=0)
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