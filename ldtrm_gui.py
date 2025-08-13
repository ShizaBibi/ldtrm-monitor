from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QLineEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QComboBox, QDoubleSpinBox
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QPainter, QColor
import random
import pyqtgraph as pg

# ===== LED Widget =====
class LEDIndicator(QWidget):
    def __init__(self, color="red"):
        super().__init__()
        self.color = color
        self.setFixedSize(20, 20)

    def setColor(self, color):
        self.color = color
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(QColor(self.color))
        painter.drawEllipse(0, 0, self.width(), self.height())

# ===== Metric Box =====
class MetricBox(QWidget):
    def __init__(self, label_text, unit=""):
        super().__init__()
        layout = QVBoxLayout()
        self.value_label = QLabel("0.00")
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet("font-size: 28px; font-weight: bold; color: cyan;")
        
        self.label = QLabel(f"{label_text} {unit}")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 14px; color: white;")
        
        layout.addWidget(self.value_label)
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.setStyleSheet("background-color: #222; border-radius: 8px; padding: 6px;")

    def update_value(self, val):
        self.value_label.setText(f"{val:.2f}")

# ===== Main Panel =====
class LDTRMPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LDTRM 2kW Monitoring & Control Panel")
        self.setMinimumSize(1400, 800)
        self.setStyleSheet("background-color: black;")

        # === Top bar ===
        title = QLabel("LDTRM 2kW")
        title.setStyleSheet("font-size: 24px; font-weight: bold; color: white;")
        self.conn_led = LEDIndicator("red")
        
        top_layout = QHBoxLayout()
        top_layout.addWidget(title)
        top_layout.addStretch()
        top_layout.addWidget(QLabel("Status:", styleSheet="color:white;"))
        top_layout.addWidget(self.conn_led)

        # === Left column: PSU metrics + controls ===
        psu_grid = QGridLayout()
        self.psu_metrics = {}
        self.psu_leds = {}
        psu_names = ["V28", "V48", "+5V", "-5V", "IDC"]
        for i, name in enumerate(psu_names):
            box = MetricBox(name)
            led = LEDIndicator("red")
            self.psu_metrics[name] = box
            self.psu_leds[name] = led
            psu_grid.addWidget(box, i, 0)
            psu_grid.addWidget(led, i, 1)

        # Control buttons
        controls = QVBoxLayout()
        for btn_text in ["Connect", "Start", "Stop", "RF ON/OFF"]:
            btn = QPushButton(btn_text)
            btn.setStyleSheet("font-size: 16px; padding: 6px;")
            controls.addWidget(btn)
        controls.addStretch()

        left_layout = QVBoxLayout()
        left_layout.addLayout(psu_grid)
        left_layout.addLayout(controls)

        # === Center column: main metrics + inputs + graph ===
        main_metrics_grid = QGridLayout()
        self.metrics = {}
        metric_names = ["FWD PWR", "REF PWR", "VSWR", "VG1", "VG2", "VG3", "Temp"]
        positions = [(i // 3, i % 3) for i in range(len(metric_names))]
        for pos, name in zip(positions, metric_names):
            box = MetricBox(name)
            self.metrics[name] = box
            main_metrics_grid.addWidget(box, *pos)

        # --- Control Inputs ---
        control_inputs = QGridLayout()

        # Bias voltages
        self.vg1_input = QDoubleSpinBox(); self.vg1_input.setRange(0, 10); self.vg1_input.setValue(1.5)
        self.vg2_input = QDoubleSpinBox(); self.vg2_input.setRange(0, 10); self.vg2_input.setValue(1.5)
        self.vg3_input = QDoubleSpinBox(); self.vg3_input.setRange(0, 10); self.vg3_input.setValue(1.5)
        control_inputs.addWidget(QLabel("VG1 Set:"), 0, 0); control_inputs.addWidget(self.vg1_input, 0, 1)
        control_inputs.addWidget(QLabel("VG2 Set:"), 0, 2); control_inputs.addWidget(self.vg2_input, 0, 3)
        control_inputs.addWidget(QLabel("VG3 Set:"), 0, 4); control_inputs.addWidget(self.vg3_input, 0, 5)

        # Limits
        self.pwr_limit = QDoubleSpinBox(); self.pwr_limit.setRange(0, 2500); self.pwr_limit.setValue(2000)
        self.vswr_limit = QDoubleSpinBox(); self.vswr_limit.setRange(1, 10); self.vswr_limit.setValue(2.0)
        self.temp_limit = QDoubleSpinBox(); self.temp_limit.setRange(0, 150); self.temp_limit.setValue(85)
        control_inputs.addWidget(QLabel("PWR Limit (W):"), 1, 0); control_inputs.addWidget(self.pwr_limit, 1, 1)
        control_inputs.addWidget(QLabel("VSWR Limit:"), 1, 2); control_inputs.addWidget(self.vswr_limit, 1, 3)
        control_inputs.addWidget(QLabel("Temp Limit (°C):"), 1, 4); control_inputs.addWidget(self.temp_limit, 1, 5)

        # PSU undervoltage limits
        self.v28_min = QDoubleSpinBox(); self.v28_min.setRange(0, 30); self.v28_min.setValue(20)
        self.v48_min = QDoubleSpinBox(); self.v48_min.setRange(0, 60); self.v48_min.setValue(40)
        control_inputs.addWidget(QLabel("V28 Min:"), 2, 0); control_inputs.addWidget(self.v28_min, 2, 1)
        control_inputs.addWidget(QLabel("V48 Min:"), 2, 2); control_inputs.addWidget(self.v48_min, 2, 3)

        # Sequencer mode
        self.mode_select = QComboBox()
        self.mode_select.addItems(["Auto", "Manual"])
        control_inputs.addWidget(QLabel("Mode:"), 2, 4); control_inputs.addWidget(self.mode_select, 2, 5)

        # IP & Port
        self.ip_input = QLineEdit("192.168.0.100")
        self.port_input = QLineEdit("502")
        control_inputs.addWidget(QLabel("IP:"), 3, 0); control_inputs.addWidget(self.ip_input, 3, 1)
        control_inputs.addWidget(QLabel("Port:"), 3, 2); control_inputs.addWidget(self.port_input, 3, 3)

        # Graph
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground("#111")
        self.graph_widget.showGrid(x=True, y=True)
        self.graph_curve = self.graph_widget.plot(pen=pg.mkPen('cyan', width=2))
        self.graph_data = []

        center_layout = QVBoxLayout()
        center_layout.addLayout(main_metrics_grid)
        center_layout.addLayout(control_inputs)
        center_layout.addWidget(self.graph_widget)

        # === Right column: packet log ===
        self.packet_log = QTextEdit()
        self.packet_log.setReadOnly(True)
        self.packet_log.setStyleSheet("color: lime; background-color: #111;")

        # === Main layout ===
        main_layout = QHBoxLayout()
        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(center_layout, 2)
        main_layout.addWidget(self.packet_log, 2)

        layout = QVBoxLayout()
        layout.addLayout(top_layout)
        layout.addLayout(main_layout)
        self.setLayout(layout)

        # === Simulation timer ===
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(1000)

    # ===== Simulation update =====
    def update_simulation(self):
        # PSU values + LED status
        for name, box in self.psu_metrics.items():
            val = random.uniform(0, 50)
            box.update_value(val)
            self.psu_leds[name].setColor("green" if 5 < val < 48 else "red")
        
        # Main metrics
        for name, box in self.metrics.items():
            val = random.uniform(0, 100)
            box.update_value(val)

        # Graph update
        if len(self.graph_data) > 50:
            self.graph_data.pop(0)
        self.graph_data.append(float(self.metrics["FWD PWR"].value_label.text()))
        self.graph_curve.setData(self.graph_data)

        # Fake packet log
        self.packet_log.append("FAKE PACKET: " + " ".join(hex(random.randint(0, 255))[2:].zfill(2) for _ in range(12)))

if __name__ == "__main__":
    app = QApplication([])
    panel = LDTRMPanel()
    panel.show()
    app.exec()
