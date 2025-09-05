import socket
import struct
import random
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QTextEdit, QLineEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QComboBox, QDoubleSpinBox, QMessageBox, QCheckBox
)
from PySide6.QtCore import Qt, QTimer, QThread, Signal
from PySide6.QtGui import QPainter, QColor
import pyqtgraph as pg

# =========================
# Small UI helpers
# =========================
class LEDIndicator(QWidget):
    def __init__(self, color="red"):
        super().__init__()
        self.color = color
        self.setFixedSize(16, 16)
    def setColor(self, color):
        self.color = color; self.update()
    def paintEvent(self, _):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QColor(self.color)); p.setPen(Qt.NoPen)
        p.drawEllipse(0, 0, self.width(), self.height())

class MetricBox(QWidget):
    def __init__(self, label_text, unit=""):
        super().__init__()
        layout = QVBoxLayout(); layout.setContentsMargins(8, 8, 8, 8); layout.setSpacing(4)
        self.value_label = QLabel("0.00")
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet("font-size: 28px; font-weight: 700; color: #00e5ff;")
        self.label = QLabel(f"{label_text} {unit}")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 12px; color: #cfd8dc;")
        layout.addWidget(self.value_label); layout.addWidget(self.label)
        self.setLayout(layout)
        self.setStyleSheet("background-color: #1c1f24; border: 1px solid #2a2f36; border-radius: 10px;")

    def update_value(self, val):
        try:
            self.value_label.setText(f"{float(val):.2f}")
        except Exception:
            self.value_label.setText(str(val))

# =========================
# Network worker (thread)
# =========================
class NetworkWorker(QThread):
    packet = Signal(bytes)
    status = Signal(str)           # "connected" | "disconnected" | "error:<msg>"
    note   = Signal(str)           # debug/status text

    def __init__(self, mode, host, port):
        super().__init__()
        self.mode = mode.upper()   # "UDP" or "TCP"
        self.host = host
        self.port = int(port)
        self.running = False
        self.sock = None

    def run(self):
        self.running = True
        try:
            if self.mode == "UDP":
                # Bind locally to receive UDP datagrams on <port>
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.settimeout(1.0)
                self.sock.bind(("0.0.0.0", self.port))
                self.status.emit("connected")
                self.note.emit(f"[UDP] Listening on 0.0.0.0:{self.port}")
                while self.running:
                    try:
                        data, addr = self.sock.recvfrom(4096)
                        self.note.emit(f"[UDP] From {addr[0]}:{addr[1]} ({len(data)} bytes)")
                        self.packet.emit(data)
                    except socket.timeout:
                        continue
            else:
                # TCP client to device (host:port)
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)
                self.sock.connect((self.host, self.port))
                self.sock.settimeout(1.0)
                self.status.emit("connected")
                self.note.emit(f"[TCP] Connected to {self.host}:{self.port}")
                while self.running:
                    try:
                        data = self.sock.recv(4096)
                        if not data:
                            self.note.emit("[TCP] Peer closed")
                            break
                        self.packet.emit(data)
                    except socket.timeout:
                        continue
        except Exception as e:
            self.status.emit(f"error:{e}")
            self.note.emit(f"[ERR] {e}")
        finally:
            try:
                if self.sock:
                    self.sock.close()
            except Exception:
                pass
            self.status.emit("disconnected")

    # Optional: send command on TCP; UDP users can use sendto externally
    def send(self, byts: bytes):
        if not self.sock: raise RuntimeError("Socket not ready")
        try:
            if self.mode == "TCP":
                self.sock.sendall(byts)
            else:
                # For UDP, send to device host:port; often you’ll broadcast or target known IP
                self.sock.sendto(byts, (self.host, self.port))
            self.note.emit(f"[TX] {byts.hex(' ')}")
        except Exception as e:
            self.status.emit(f"error:{e}")
            self.note.emit(f"[TX-ERR] {e}")

    def stop(self):
        self.running = False

# =========================
# Parse packets (stub)
# Replace with your real byte layout when available
# =========================
def parse_packet(data: bytes) -> dict:
    """
    Return a dict of fields the GUI knows how to display.
    >>> example: {'V28': 27.9, 'V48': 48.1, '+5V': 5.01, '-5V': -4.98,
                  'IDC': 21.3, 'FWD PWR': 1034, 'REF PWR': 11.2,
                  'VSWR': 1.5, 'VG1': 1.45, 'VG2': 1.50, 'VG3': 1.47, 'Temp': 42.3}
    """
    # ---------- DEMO FORMAT (fake) ----------
    # If your MCU team hasn't given a spec yet, this supports two cases:
    # 1) CSV line: b"V28,27.95,V48,48.12,IDC,21.3,PF,1034,PR,11.2,T,42.3,VG1,1.45,VG2,1.50,VG3,1.47"
    # 2) Binary demo: b"LD" + 12 floats (big-endian) in order:
    #    V28, V48, p5, n5, IDC, PF, PR, Temp, VG1, VG2, VG3, Reserved
    try:
        txt = data.strip().decode("utf-8", errors="ignore")
        if "," in txt:
            parts = txt.split(",")
            kv = {parts[i]: parts[i+1] for i in range(0, len(parts)-1, 2)}
            # map to GUI keys safely
            out = {}
            if "V28" in kv: out["V28"] = float(kv["V28"])
            if "V48" in kv: out["V48"] = float(kv["V48"])
            if "+5V" in kv or "P5" in kv: out["+5V"] = float(kv.get("+5V", kv.get("P5")))
            if "-5V" in kv or "N5" in kv: out["-5V"] = float(kv.get("-5V", kv.get("N5")))
            if "IDC" in kv: out["IDC"] = float(kv["IDC"])
            if "PF" in kv: out["FWD PWR"] = float(kv["PF"])
            if "PR" in kv: out["REF PWR"] = float(kv["PR"])
            if "T" in kv or "TEMP" in kv: out["Temp"] = float(kv.get("T", kv.get("TEMP")))
            if "VG1" in kv: out["VG1"] = float(kv["VG1"])
            if "VG2" in kv: out["VG2"] = float(kv["VG2"])
            if "VG3" in kv: out["VG3"] = float(kv["VG3"])
            # VSWR derived if PF/PR present
            if "FWD PWR" in out and "REF PWR" in out:
                pf, pr = out["FWD PWR"], out["REF PWR"]
                rho = (pr / pf) ** 0.5 if pf > 0 else 0
                out["VSWR"] = (1 + rho) / (1 - rho) if rho < 1 else 99.9
            return out
        # Binary demo
        if len(data) >= 2 + 12 * 4 and data[:2] == b"LD":
            floats = struct.unpack(">12f", data[2:2+12*4])
            V28, V48, p5, n5, IDC, PF, PR, Temp, VG1, VG2, VG3, _ = floats
            rho = (PR / PF) ** 0.5 if PF > 0 else 0
            vswr = (1 + rho) / (1 - rho) if rho < 1 else 99.9
            return {"V28": V28, "V48": V48, "+5V": p5, "-5V": n5, "IDC": IDC,
                    "FWD PWR": PF, "REF PWR": PR, "Temp": Temp,
                    "VG1": VG1, "VG2": VG2, "VG3": VG3, "VSWR": vswr}
    except Exception:
        pass
    return {}  # unknown format; still log hex

# =========================
# Main GUI
# =========================
class LDTRMPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LDTRM 2kW — Monitoring & Control (v0.2.0)")
        self.setMinimumSize(1450, 820)
        self.setStyleSheet("background-color: #0f1216;")

        # ===== Top bar =====
        title = QLabel("LDTRM 2 kW")
        title.setStyleSheet("font-size: 22px; font-weight: 800; color: #ffffff;")
        self.conn_led = LEDIndicator("red")
        self.sim_check = QCheckBox("Simulation Mode")
        self.sim_check.setChecked(True)

        self.net_mode = QComboBox(); self.net_mode.addItems(["UDP", "TCP"])
        self.ip_input = QLineEdit("192.168.0.100")
        self.port_input = QLineEdit("4001")
        for w in (self.ip_input, self.port_input):
            w.setStyleSheet("color:#e0e6eb; background:#161b22; border:1px solid #2a2f36; border-radius:6px; padding:4px;")

        self.btn_connect = QPushButton("Connect")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_connect.setStyleSheet("background:#1976d2; color:white; padding:6px; border-radius:8px;")
        self.btn_disconnect.setStyleSheet("background:#455a64; color:white; padding:6px; border-radius:8px;")

        top = QHBoxLayout()
        top.addWidget(title); top.addStretch()
        top.addWidget(QLabel("Mode:", styleSheet="color:#cfd8dc;"))
        top.addWidget(self.net_mode)
        top.addWidget(QLabel("IP:", styleSheet="color:#cfd8dc;"))
        top.addWidget(self.ip_input)
        top.addWidget(QLabel("Port:", styleSheet="color:#cfd8dc;"))
        top.addWidget(self.port_input)
        top.addWidget(self.sim_check)
        top.addWidget(QLabel("Status:", styleSheet="color:#cfd8dc;")); top.addWidget(self.conn_led)
        top.addWidget(self.btn_connect); top.addWidget(self.btn_disconnect)

        # ===== Left: PSU + actions =====
        psu_grid = QGridLayout()
        self.psu_metrics, self.psu_leds = {}, {}
        for row, name in enumerate(["V28", "V48", "+5V", "-5V", "IDC"]):
            box = MetricBox(name); led = LEDIndicator("red")
            self.psu_metrics[name] = box; self.psu_leds[name] = led
            psu_grid.addWidget(box, row, 0); psu_grid.addWidget(led, row, 1)

        self.btn_start = QPushButton("Start"); self.btn_stop = QPushButton("Stop")
        self.btn_rf    = QPushButton("RF ON/OFF")
        for b in (self.btn_start, self.btn_stop, self.btn_rf):
            b.setStyleSheet("background:#263238; color:#eceff1; padding:8px; border-radius:10px;")
        left = QVBoxLayout(); left.addLayout(psu_grid); left.addWidget(self.btn_start); left.addWidget(self.btn_stop); left.addWidget(self.btn_rf); left.addStretch()

        # ===== Center: main metrics + control inputs + graph =====
        grid = QGridLayout(); self.metrics = {}
        for i, name in enumerate(["FWD PWR", "REF PWR", "VSWR", "VG1", "VG2", "VG3", "Temp"]):
            box = MetricBox(name); self.metrics[name] = box
            grid.addWidget(box, i//3, i%3)

        controls = QGridLayout()
        # Bias setpoints
        self.vg1_set = QDoubleSpinBox(); self.vg1_set.setRange(0, 10); self.vg1_set.setValue(1.50)
        self.vg2_set = QDoubleSpinBox(); self.vg2_set.setRange(0, 10); self.vg2_set.setValue(1.50)
        self.vg3_set = QDoubleSpinBox(); self.vg3_set.setRange(0, 10); self.vg3_set.setValue(1.50)
        # Limits
        self.pwr_limit = QDoubleSpinBox(); self.pwr_limit.setRange(0, 3000); self.pwr_limit.setValue(2000)
        self.vswr_limit= QDoubleSpinBox(); self.vswr_limit.setRange(1, 10);  self.vswr_limit.setValue(2.0)
        self.temp_limit= QDoubleSpinBox(); self.temp_limit.setRange(0, 150); self.temp_limit.setValue(85)
        # PSU thresholds
        self.v28_min = QDoubleSpinBox(); self.v28_min.setRange(0, 30); self.v28_min.setValue(20)
        self.v48_min = QDoubleSpinBox(); self.v48_min.setRange(0, 60); self.v48_min.setValue(40)

        row = 0
        for lbl, w in [("VG1 Set", self.vg1_set), ("VG2 Set", self.vg2_set), ("VG3 Set", self.vg3_set),
                       ("Pwr Limit (W)", self.pwr_limit), ("VSWR Limit", self.vswr_limit), ("Temp Limit (°C)", self.temp_limit),
                       ("V28 Min", self.v28_min), ("V48 Min", self.v48_min)]:
            r, c = divmod(row, 4); controls.addWidget(QLabel(lbl, styleSheet="color:#cfd8dc;"), r, c*2); controls.addWidget(w, r, c*2+1); row += 1

        self.btn_send = QPushButton("Send Setpoints/Limits")
        self.btn_send.setStyleSheet("background:#00897b; color:white; padding:8px; border-radius:10px;")
        controls.addWidget(self.btn_send, (row+1)//4, 0, 1, 8)

        # Graph (Forward Power)
        self.graph = pg.PlotWidget(); self.graph.setBackground("#111")
        self.graph.showGrid(x=True, y=True)
        self.curve_pf = self.graph.plot(pen=pg.mkPen('#00e5ff', width=2))  # forward power
        self.curve_pr = self.graph.plot(pen=pg.mkPen('#ffab00', width=2))  # reflected power
        self.data_pf, self.data_pr = [], []

        center = QVBoxLayout(); center.addLayout(grid); center.addLayout(controls); center.addWidget(self.graph)

        # ===== Right: packet log =====
        self.log = QTextEdit(); self.log.setReadOnly(True)
        self.log.setStyleSheet("color:#9cff57; background:#121417; border:1px solid #2a2f36; border-radius:10px;")

        # ===== Outermost layout =====
        mid = QHBoxLayout(); mid.addLayout(left, 1); mid.addLayout(center, 2); mid.addWidget(self.log, 2)
        root = QVBoxLayout(); root.addLayout(top); root.addLayout(mid)
        self.setLayout(root)

        # ===== State =====
        self.worker: NetworkWorker | None = None
        self.timer = QTimer(); self.timer.timeout.connect(self.tick_sim); self.timer.start(1000)

        # ===== Signals =====
        self.btn_connect.clicked.connect(self.do_connect)
        self.btn_disconnect.clicked.connect(self.do_disconnect)
        self.btn_send.clicked.connect(self.do_send_setpoints)

    # ---------- Simulation ----------
    def tick_sim(self):
        if not self.sim_check.isChecked():
            return  # network mode
        # Fake PSU values & LEDs
        vals = {
            "V28": random.uniform(22, 29), "V48": random.uniform(45, 49.5),
            "+5V": random.uniform(4.9, 5.1), "-5V": random.uniform(-5.1, -4.9),
            "IDC": random.uniform(0, 40)
        }
        for k,v in vals.items():
            self.psu_metrics[k].update_value(v)
            # simple in-range LED rule
            if k == "V28":
                self.psu_leds[k].setColor("green" if v >= self.v28_min.value() else "red")
            elif k == "V48":
                self.psu_leds[k].setColor("green" if v >= self.v48_min.value() else "red")
            elif k in ("+5V","-5V"):
                ok = (4.75 <= vals["+5V"] <= 5.25) and (-5.25 <= vals["-5V"] <= -4.75)
                self.psu_leds[k].setColor("green" if ok else "red")
            else:
                self.psu_leds[k].setColor("green")

        # Main metrics
        pf = random.uniform(0, self.pwr_limit.value())
        pr = random.uniform(0, max(1.0, pf*0.05))
        rho = (pr/pf)**0.5 if pf>0 else 0
        vswr = (1+rho)/(1-rho) if rho<1 else 99.9
        for k,v in {"FWD PWR":pf, "REF PWR":pr, "VSWR":vswr,
                    "VG1":self.vg1_set.value()+random.uniform(-0.03,0.03),
                    "VG2":self.vg2_set.value()+random.uniform(-0.03,0.03),
                    "VG3":self.vg3_set.value()+random.uniform(-0.03,0.03),
                    "Temp":random.uniform(30, 70)}.items():
            self.metrics[k].update_value(v)

        # Graph
        if len(self.data_pf) > 200: self.data_pf.pop(0); self.data_pr.pop(0)
        self.data_pf.append(pf); self.data_pr.append(pr)
        self.curve_pf.setData(self.data_pf); self.curve_pr.setData(self.data_pr)

        # Log
        fake = bytes(random.randint(0,255) for _ in range(24))
        self.log.append(f"[SIM RX] {fake.hex(' ')}")

    # ---------- Network control ----------
    def do_connect(self):
        if self.worker and self.worker.isRunning():
            QMessageBox.information(self, "Already connected", "Network worker is running.")
            return
        self.sim_check.setChecked(False)  # switch off sim when connecting
        mode = self.net_mode.currentText()
        host = self.ip_input.text().strip()
        port = self.port_input.text().strip()
        if not port.isdigit():
            QMessageBox.warning(self, "Port", "Port must be a number."); return
        self.worker = NetworkWorker(mode, host, int(port))
        self.worker.packet.connect(self.on_packet)
        self.worker.status.connect(self.on_status)
        self.worker.note.connect(self.append_log)
        self.worker.start()

    def do_disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker.wait(1500)
            self.worker = None

    def on_status(self, s: str):
        if s == "connected":
            self.conn_led.setColor("green")
            self.append_log("[NET] connected")
        elif s.startswith("error:"):
            self.conn_led.setColor("red")
            self.append_log(s)
        elif s == "disconnected":
            self.conn_led.setColor("red")
            self.append_log("[NET] disconnected")

    def append_log(self, line: str):
        self.log.append(line)

    def on_packet(self, data: bytes):
        # Log hex
        self.log.append(data.hex(" "))
        # Parse and render
        fields = parse_packet(data)
        if not fields:
            return
        # PSU
        for k in ("V28","V48","+5V","-5V","IDC"):
            if k in fields: self.psu_metrics[k].update_value(fields[k])
        # LEDs
        if "V28" in fields:
            self.psu_leds["V28"].setColor("green" if fields["V28"] >= self.v28_min.value() else "red")
        if "V48" in fields:
            self.psu_leds["V48"].setColor("green" if fields["V48"] >= self.v48_min.value() else "red")
        if "+5V" in fields and "-5V" in fields:
            ok = (4.75 <= fields["+5V"] <= 5.25) and (-5.25 <= fields["-5V"] <= -4.75)
            self.psu_leds["+5V"].setColor("green" if ok else "red")
            self.psu_leds["-5V"].setColor("green" if ok else "red")
        # Main metrics
        for k in ("FWD PWR","REF PWR","VSWR","VG1","VG2","VG3","Temp"):
            if k in fields: self.metrics[k].update_value(fields[k])
        # Graph update
        pf = float(self.metrics["FWD PWR"].value_label.text()); 
        pr = float(self.metrics["REF PWR"].value_label.text()) if self.metrics["REF PWR"].value_label.text() else 0.0
        if len(self.data_pf) > 200: self.data_pf.pop(0); self.data_pr.pop(0)
        self.data_pf.append(pf); self.data_pr.append(pr)
        self.curve_pf.setData(self.data_pf); self.curve_pr.setData(self.data_pr)

    # ---------- Send commands ----------
    def do_send_setpoints(self):
        """
        Demo command format (human-readible ASCII).
        Replace with your MCU's real protocol (binary or Modbus).
        """
        payloads = [
            f"SET VG1 {self.vg1_set.value():.2f}\n",
            f"SET VG2 {self.vg2_set.value():.2f}\n",
            f"SET VG3 {self.vg3_set.value():.2f}\n",
            f"SET PWR_LIMIT {self.pwr_limit.value():.1f}\n",
            f"SET VSWR_LIMIT {self.vswr_limit.value():.2f}\n",
            f"SET TEMP_LIMIT {self.temp_limit.value():.1f}\n",
            f"SET V28_MIN {self.v28_min.value():.2f}\n",
            f"SET V48_MIN {self.v48_min.value():.2f}\n",
        ]
        if not self.worker or not self.worker.isRunning():
            QMessageBox.information(self, "Not connected", "Connect to device (or run in SIM) first.")
            return
        try:
            for p in payloads:
                self.worker.send(p.encode("ascii"))
            self.append_log("[TX] setpoints/limits sent")
        except Exception as e:
            QMessageBox.critical(self, "Send failed", str(e))

    def closeEvent(self, e):
        try:
            self.do_disconnect()
        except Exception:
            pass
        e.accept()

if __name__ == "__main__":
    app = QApplication([])
    panel = LDTRMPanel()
    panel.show()
    app.exec()
