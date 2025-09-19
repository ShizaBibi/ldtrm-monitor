import sys, threading, random
from collections import deque
from typing import Tuple, Dict, List

import serial
import serial.tools.list_ports

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QTextEdit,
    QSplitter, QTableWidget, QTableWidgetItem, QCheckBox, QMessageBox,
    QComboBox, QTabWidget
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject

import pyqtgraph as pg

# Packet constants
SOF = 0xFF
FRAME_LEN = 174
SEQ_MAX = 0xFF

HDR_PS = bytes([0x50, 0x53])   # PS block header
HDR_DO = bytes([0x44, 0x00])   # DO block header
HDR_D1 = bytes([0x44, 0x01])   # D1 block header
HDR_D2 = bytes([0x44, 0x02])   # D2 block header
HDR_P = [
    bytes([0x50, 0x01]),
    bytes([0x50, 0x02]),
    bytes([0x50, 0x03]),
    bytes([0x50, 0x04]),
    bytes([0x50, 0x05]),
    bytes([0x50, 0x06]),
    bytes([0x50, 0x07]),
    bytes([0x50, 0x08]),
    bytes([0x50, 0x09]),
    bytes([0x50, 0x0A]),
]

def pack2(val: float) -> bytes:
    v = int(round(val * 100))
    if v > 65535:
        v = 65535
    elif v < 0:
        v = 0
    return v.to_bytes(2, "big")

def unpack2(b: bytes, pos: int) -> Tuple[float, int]:
    v = int.from_bytes(b[pos:pos+2], "big")
    pos += 2
    return v / 100.0, pos

def build_dummy_174_frame() -> bytes:
    frame = bytearray()

    # PS block
    frame += HDR_PS
    for _ in range(7):
        frame += pack2(random.uniform(0, 50))
    for _ in range(8):
        frame.append(random.randint(0, 1))

    # DO block
    frame += HDR_DO
    frame += pack2(random.uniform(0, 5))
    frame += pack2(random.uniform(0, 60))
    frame += pack2(random.uniform(20, 60))
    frame += pack2(random.uniform(0, 500))

    # D1 block
    frame += HDR_D1
    frame += pack2(random.uniform(0, 5))
    frame += pack2(random.uniform(0, 60))
    frame += pack2(random.uniform(20, 60))
    frame += pack2(random.uniform(0, 500))

    # Pallets P1-P5 (4 data vals + 2 reserved 0xFF 0xFF)
    for hdr in HDR_P[:5]:
        frame += hdr
        for _ in range(4):
            frame += pack2(random.uniform(0, 60))
        frame += b'\xFF\xFF'  # reserved bytes fixed

    # D2 block
    frame += HDR_D2
    frame += pack2(random.uniform(0, 5))
    frame += pack2(random.uniform(0, 60))
    frame += pack2(random.uniform(20, 60))
    frame += pack2(random.uniform(0, 500))

    # Pallets P6-P10 same as P1-P5
    for hdr in HDR_P[5:]:
        frame += hdr
        for _ in range(4):
            frame += pack2(random.uniform(0, 60))
        frame += b'\xFF\xFF'

    if len(frame) < FRAME_LEN:
        frame += bytes(FRAME_LEN - len(frame))
    return bytes(frame[:FRAME_LEN])

def parse_174_stream(buf: bytearray) -> List[Dict]:
    frames = []
    i = 0
    while i + FRAME_LEN <= len(buf):
        chunk = bytes(buf[i:i+FRAME_LEN])
        p = 0
        ps, drivers, pallets = {}, {}, []

        try:
            # PS block
            if chunk[p:p+2] == HDR_PS:
                p += 2
                ps_vals = []
                for _ in range(7):
                    val, p = unpack2(chunk, p)
                    ps_vals.append(val)
                statuses = [chunk[p+j] for j in range(8)]
                p += 8
                ps = {"val1": ps_vals[0], "val2": ps_vals[1], "val3": ps_vals[2], "val4": ps_vals[3],
                      "val5": ps_vals[4], "val6": ps_vals[5], "val7": ps_vals[6], "statuses": statuses}
            else:
                p += 24

            # DO block
            if chunk[p:p+2] == HDR_DO:
                p += 2
            else:
                p += 2
            d0_curr, p = unpack2(chunk, p)
            d0_volt, p = unpack2(chunk, p)
            d0_temp, p = unpack2(chunk, p)
            d0_rf, p = unpack2(chunk, p)
            drivers['0'] = {"current": d0_curr, "voltage": d0_volt,
                            "temp": d0_temp, "rf": d0_rf}

            # D1 block
            if chunk[p:p+2] == HDR_D1:
                p += 2
            else:
                p += 2
            d1_curr, p = unpack2(chunk, p)
            d1_volt, p = unpack2(chunk, p)
            d1_temp, p = unpack2(chunk, p)
            d1_rf, p = unpack2(chunk, p)
            drivers['1'] = {"current": d1_curr, "voltage": d1_volt,
                            "temp": d1_temp, "rf": d1_rf}

            # Pallets P1–P5 (4 data + reserved)
            for idx, hdr in enumerate(HDR_P[:5]):
                if chunk[p:p+2] == hdr:
                    p += 2
                else:
                    p += 2
                vals = []
                for _ in range(4):
                    v, p = unpack2(chunk, p)
                    vals.append(v)
                reserved = chunk[p:p+2]
                p += 2
                pallets.append({"id": f"P{idx+1}", "val1": vals[0], "val2": vals[1],
                                "val3": vals[2], "val4": vals[3], "reserved": reserved})

            # D2 block
            if chunk[p:p+2] == HDR_D2:
                p += 2
            else:
                p += 2
            d2_curr, p = unpack2(chunk, p)
            d2_volt, p = unpack2(chunk, p)
            d2_temp, p = unpack2(chunk, p)
            d2_rf, p = unpack2(chunk, p)
            drivers['2'] = {"current": d2_curr, "voltage": d2_volt,
                            "temp": d2_temp, "rf": d2_rf}

            # Pallets P6–P10
            for idx, hdr in enumerate(HDR_P[5:], start=6):
                if chunk[p:p+2] == hdr:
                    p += 2
                else:
                    p += 2
                vals = []
                for _ in range(4):
                    v, p = unpack2(chunk, p)
                    vals.append(v)
                reserved = chunk[p:p+2]
                p += 2
                pallets.append({"id": f"P{idx}", "val1": vals[0], "val2": vals[1],
                                "val3": vals[2], "val4": vals[3], "reserved": reserved})

        except Exception:
            pass

        frames.append({"seq": None, "PS": ps, "D0": drivers.get('0', {}),
                       "D1": drivers.get('1', {}), "D2": drivers.get('2', {}),
                       "Pallets": pallets, "RAW": chunk})
        i += FRAME_LEN

    del buf[:i]
    return frames

# ---------------- UI widgets ----------------
class LEDLabel(QLabel):
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.setFixedSize(26, 18)
        self.setStyleSheet("border-radius: 6px; background-color: gray; color: white; padding:2px;")
        self.setAlignment(Qt.AlignCenter)
        self.setText(name)

    def set_state(self, value):
        if value == 1:
            self.setStyleSheet("border-radius: 6px; background-color: #00e676; color: black; padding:2px;")
        else:
            self.setStyleSheet("border-radius: 6px; background-color: #ff3d00; color: white; padding:2px;")
        self.setText(self.name)

def make_ps_d_panel(title, is_ps=False):
    box = QGroupBox(title); box.setStyleSheet("QGroupBox{color:#cfd8dc;}")
    layout = QVBoxLayout(box)
    grid = QGridLayout()
    value_labels = {}
    status_labels = {}
    if is_ps:
        field_names = [("Val1", ""), ("Val2", ""), ("Val3", ""), ("Val4", ""), ("Val5", ""), ("Val6", ""), ("Val7", "")]
    else:
        field_names = [("Val1", ""), ("Val2", ""), ("Val3", ""), ("Val4", "")]
    for row, (name, unit) in enumerate(field_names):
        name_lbl = QLabel(name)
        name_lbl.setAlignment(Qt.AlignRight)
        val_lbl = QLabel(f"-- {unit}")
        val_lbl.setAlignment(Qt.AlignRight)
        val_lbl.setStyleSheet("font-size:14px; font-weight:600; color:#00e676;")
        grid.addWidget(name_lbl, row, 0)
        grid.addWidget(val_lbl, row, 1)
        value_labels[name] = val_lbl
    layout.addLayout(grid)
    if is_ps:
        status_layout = QHBoxLayout()
        status_names = ["DO", "AO", "FFS", "OT", "OC", "VU", "RF", "OP"]
        for name in status_names:
            lbl = LEDLabel(name)
            status_layout.addWidget(lbl)
            status_labels[name] = lbl
        layout.addLayout(status_layout)
    return box, value_labels, status_labels

class PSValues(QWidget):
    def __init__(self):
        super().__init__()
        self.panel, self.value_labels, self.status_labels = make_ps_d_panel("PS", is_ps=True)

    def get_widget(self):
        return self.panel

    def update(self, ps: Dict):
        keys = ["Val1", "Val2", "Val3", "Val4", "Val5", "Val6", "Val7"]
        for k in keys:
            self.value_labels[k].setText(f"{ps.get(k.lower(), ps.get(k, 0)):.2f}")
        statuses = ps.get('statuses', [0]*8)
        names = ["DO", "AO", "FFS", "OT", "OC", "VU", "RF", "OP"]
        for i, name in enumerate(names):
            if name in self.status_labels:
                self.status_labels[name].set_state(statuses[i] if i < len(statuses) else 0)

class D0Values(QWidget):
    def __init__(self):
        super().__init__()
        self.panel, self.value_labels, _ = make_ps_d_panel("D0")

    def get_widget(self):
        return self.panel

    def update(self, d: Dict):
        self.value_labels["Val1"].setText(f"{d.get('current', 0):.2f}")
        self.value_labels["Val2"].setText(f"{d.get('voltage', 0):.2f}")
        self.value_labels["Val3"].setText(f"{d.get('temp', 0):.2f}")
        self.value_labels["Val4"].setText(f"{d.get('rf', 0):.2f}")

class DriverGroup(QWidget):
    def __init__(self, title):
        super().__init__()
        self.box = QGroupBox(title)
        self.box.setStyleSheet("QGroupBox{color:#cfd8dc;}")
        layout = QVBoxLayout(self.box)
        self.panels = []
        grid = QGridLayout()
        for i in range(5):
            pbox = QGroupBox(f"P{i+1}")
            pgrid = QGridLayout(pbox)
            labels = {}
            for r, (n, u) in enumerate([("Val1", ""), ("Val2", ""), ("Val3", ""), ("Val4", "")]):
                lbl = QLabel(n)
                val = QLabel(f"-- {u}")
                val.setStyleSheet("font-weight:600; color:#00e676;")
                pgrid.addWidget(lbl, r, 0)
                pgrid.addWidget(val, r, 1)
                labels[n] = val
            grid.addWidget(pbox, i//2, i%2)
            self.panels.append(labels)
        layout.addLayout(grid)

    def get_widget(self):
        return self.box

    def update(self, pallets: List[Dict]):
        for i in range(5):
            p = pallets[i] if i < len(pallets) else {"val1": 0, "val2": 0, "val3": 0, "val4": 0}
            labels = self.panels[i]
            labels["Val1"].setText(f"{p.get('val1', 0):.2f}")
            labels["Val2"].setText(f"{p.get('val2', 0):.2f}")
            labels["Val3"].setText(f"{p.get('val3', 0):.2f}")
            labels["Val4"].setText(f"{p.get('val4', 0):.2f}")

class PalletTable(QTableWidget):
    def __init__(self):
        super().__init__(10, 5)
        self.setHorizontalHeaderLabels(["P#", "Val1", "Val2", "Val3", "Val4"])
        for r in range(10):
            for c in range(5):
                self.setItem(r, c, QTableWidgetItem("--"))
        self.setAlternatingRowColors(True)
        self.resizeColumnsToContents()

    def update(self, pls: List[Dict]):
        for i, p in enumerate(pls):
            self.setItem(i, 0, QTableWidgetItem(str(p.get("id", "--"))))
            self.setItem(i, 1, QTableWidgetItem(f"{p.get('val1', 0):.2f}"))
            self.setItem(i, 2, QTableWidgetItem(f"{p.get('val2', 0):.2f}"))
            self.setItem(i, 3, QTableWidgetItem(f"{p.get('val3', 0):.2f}"))
            self.setItem(i, 4, QTableWidgetItem(f"{p.get('val4', 0):.2f}"))
        self.resizeColumnsToContents()

# ---------------- Serial thread ----------------
class NetSignals(QObject):
    bytes_rx = Signal(bytes)
    state = Signal(bool, str)

class SerialThread(threading.Thread):
    def __init__(self, port, baud, sig: NetSignals):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.sig = sig
        self._stop = threading.Event()
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.sig.state.emit(True, f"Serial connected {self.port} @ {self.baud}")
            while not self._stop.is_set():
                try:
                    d = self.ser.read(4096)
                    if d:
                        self.sig.bytes_rx.emit(d)
                except Exception as e:
                    self.sig.state.emit(False, f"Serial read error: {e}")
                    break
        except Exception as e:
            self.sig.state.emit(False, f"Serial open error: {e}")
        finally:
            if self.ser:
                self.ser.close()
            self.sig.state.emit(False, "Serial disconnected")

    def send(self, b: bytes):
        try:
            if self.ser:
                self.ser.write(b)
        except Exception:
            pass

    def stop(self):
        self._stop.set()

# ---------------- MainWindow ----------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MR-36 Console (174B Serial) — PS + D0 + D1 + D2 + P1-P10")
        self.resize(1400, 920)
        central = QWidget(); self.setCentralWidget(central); v = QVBoxLayout(central)

        # Top bar for simulation + serial settings
        top = QHBoxLayout()
        self.sim_chk = QCheckBox("Simulation"); self.sim_chk.setChecked(True); top.addWidget(self.sim_chk)
        top.addSpacing(20)
        top.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports if ports else ["No ports"])
        top.addWidget(self.port_combo)
        top.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        top.addWidget(self.baud_combo)
        self.btn_conn = QPushButton("Connect Serial"); self.btn_conn.clicked.connect(self.toggle_conn)
        top.addWidget(self.btn_conn)
        self.connected_port_label = QLabel("Connected Port: None")
        top.addWidget(self.connected_port_label)
        v.addLayout(top)

        # Main split UI
        split = QSplitter(Qt.Horizontal); v.addWidget(split, 1)
        left_split = QSplitter(Qt.Vertical)
        left_split.setSizes([300, 620])

        # Info tabs
        self.info_tabs = QTabWidget()
        self.ps_values = PSValues()
        self.d0_values = D0Values()
        self.d1_values = DriverGroup("D1 (P1-P5)")
        self.d2_values = DriverGroup("D2 (P6-P10)")
        self.info_tabs.addTab(self.ps_values.get_widget(), "PS")
        self.info_tabs.addTab(self.d0_values.get_widget(), "D0")
        self.info_tabs.addTab(self.d1_values.get_widget(), "D1")
        self.info_tabs.addTab(self.d2_values.get_widget(), "D2")
        left_split.addWidget(self.info_tabs)

        # Graphs setup
        graphs_widget = QWidget()
        gv = QVBoxLayout(graphs_widget)
        gv.setContentsMargins(5,5,5,5)
        gv.setSpacing(10)

        # RF Sum Graph fixed range
        self.rf_sum_graph = pg.PlotWidget()
        self.rf_sum_graph.setBackground("#111")
        self.rf_sum_graph.setLabel('left', 'Total RF Sum (W)', color='white', size='12pt')
        self.rf_sum_graph.setLabel('bottom', 'Frames', color='white', size='12pt')
        self.rf_sum_graph.enableAutoRange(False, False)
        self.rf_sum_graph.setXRange(0, 600)
        self.rf_sum_graph.setYRange(0, 2000)
        self.rf_sum_curve = self.rf_sum_graph.plot(pen=pg.mkPen('#00e676', width=2))
        self.rf_sum_buf = deque([0.0]*600, maxlen=600)
        gv.addWidget(self.rf_sum_graph, 1)

        # PS Voltage graph fixed range
        self.ps_volt_graph = pg.PlotWidget()
        self.ps_volt_graph.setBackground("#111")
        self.ps_volt_graph.setLabel('left', 'PS Val2', color='white', size='12pt')
        self.ps_volt_graph.setLabel('bottom', 'Frames', color='white', size='12pt')
        self.ps_volt_graph.enableAutoRange(False, False)
        self.ps_volt_graph.setXRange(0, 600)
        self.ps_volt_graph.setYRange(0, 60)
        self.ps_volt_curve = self.ps_volt_graph.plot(pen=pg.mkPen('#ff9800', width=2))
        self.ps_volt_buf = deque([0.0]*600, maxlen=600)
        gv.addWidget(self.ps_volt_graph, 1)

        # Variable graph for selected entity & parameter
        dropdown_h = QHBoxLayout()
        dropdown_h.setSpacing(10)
        dropdown_h.addWidget(QLabel("Entity:"))
        self.entity_combo = QComboBox()
        entities_list = ["PS", "D0", "D1", "D2"] + [f"P{i}" for i in range(1,11)]
        self.entity_combo.addItems(entities_list)
        self.entity_combo.setCurrentText("D0")
        self.entity_combo.currentIndexChanged.connect(self.on_selection_change)
        dropdown_h.addWidget(self.entity_combo)

        dropdown_h.addWidget(QLabel("Parameter:"))
        self.param_combo = QComboBox()
        params_list = ["Val1", "Val2", "Val3", "Val4"]
        self.param_combo.addItems(params_list)
        self.param_combo.setCurrentText("Val1")
        self.param_combo.currentIndexChanged.connect(self.on_selection_change)
        dropdown_h.addWidget(self.param_combo)

        dropdown_h.addStretch()
        gv.addLayout(dropdown_h)

        self.var_graph = pg.PlotWidget()
        self.var_graph.setBackground("#111")
        self.var_graph.setLabel('bottom', 'Frames', color='white', size='12pt')
        self.var_graph.enableAutoRange(False, False)
        self.var_graph.setXRange(0, 600)
        self.var_graph.setYRange(0, 100)
        self.var_curve = self.var_graph.plot(pen=pg.mkPen('#00e676', width=2))
        gv.addWidget(self.var_graph, 2)

        left_split.addWidget(graphs_widget)
        split.addWidget(left_split)

        # Pallet table and send area on right
        right_widget = QWidget()
        rv = QVBoxLayout(right_widget)
        rv.setSpacing(10)
        self.pallet_table = PalletTable()
        rv.addWidget(self.pallet_table, 2)
        send_row = QHBoxLayout()
        send_row.setSpacing(10)
        self.hex_in = QLineEdit("FF ...")
        send_row.addWidget(self.hex_in, 1)
        self.hex_chk = QCheckBox("HEX")
        self.hex_chk.setChecked(True)
        send_row.addWidget(self.hex_chk)
        self.hex_verbose = QCheckBox("HEX raw")
        self.hex_verbose.setChecked(False)
        send_row.addWidget(self.hex_verbose)
        self.btn_send = QPushButton("Send")
        self.btn_send.clicked.connect(self.do_send)
        send_row.addWidget(self.btn_send)
        self.btn_send_dummy = QPushButton("Send Dummy")
        self.btn_send_dummy.clicked.connect(self.send_dummy)
        send_row.addWidget(self.btn_send_dummy)
        rv.addLayout(send_row)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        rv.addWidget(self.log, 1)

        split.addWidget(right_widget)
        split.setSizes([1000, 400])

        # Signals and serial setup
        self.signals = NetSignals()
        self.signals.bytes_rx.connect(self.on_network_bytes)
        self.signals.state.connect(self.on_state)
        self.serial_thread = None

        # Value deques for all entities & params (initialized with zeros)
        self.value_deques = {}
        for ent in entities_list:
            self.value_deques[ent] = {p: deque([0.0]*600, maxlen=600) for p in params_list}

        # Receive buffer
        self.rx_buffer = bytearray()

        # Timer to trigger simulation frame every 50 ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

        self.on_selection_change()

    def on_selection_change(self):
        ent = self.entity_combo.currentText()
        param = self.param_combo.currentText()
        units = {"Val1": "", "Val2": "", "Val3": "", "Val4": ""}
        unit = units.get(param, "")
        self.var_graph.setLabel('left', f'{param} ({unit})', color='white', size='12pt')

        self.var_graph.setYRange(0, 100)

        if ent in self.value_deques and param in self.value_deques[ent]:
            buf = self.value_deques[ent][param]
            x = list(range(len(buf)))
            self.var_curve.setData(x, list(buf))

    def tick(self):
        if self.sim_chk.isChecked():
            frame = build_dummy_174_frame()
            self.signals.bytes_rx.emit(frame)
            if random.randint(0,9) == 0:
                self.append_log("[SIM] Generated dummy frame", color="blue")

    def toggle_conn(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread = None
            self.btn_conn.setText("Connect Serial")
            self.connected_port_label.setText("Connected Port: None")
            return
        if self.sim_chk.isChecked():
            QMessageBox.information(self, "Simulation ON", "Disable Simulation to open serial.")
            return
        port = self.port_combo.currentText()
        if not port or port == "No ports":
            QMessageBox.warning(self, "Port", "No serial port selected.")
            return
        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            QMessageBox.warning(self, "Baud", "Invalid baud rate.")
            return
        self.serial_thread = SerialThread(port, baud, self.signals)
        self.serial_thread.start()
        self.btn_conn.setText("Disconnect Serial")
        self.connected_port_label.setText(f"Connected Port: {port} @ {baud} baud")

    def on_state(self, ok: bool, msg: str):
        self.append_log(msg)
        if not ok and self.serial_thread:
            self.connected_port_label.setText("Connected Port: None")

    def do_send(self):
        txt = self.hex_in.text().strip()
        if self.hex_chk.isChecked():
            cleaned = txt.replace(" ", "").replace(",", "")
            if len(cleaned) % 2 != 0:
                QMessageBox.warning(self, "HEX", "Hex length must be even")
                return
            try:
                data = bytes.fromhex(cleaned)
            except Exception:
                QMessageBox.warning(self, "HEX", "Invalid hex")
                return
            self.send_bytes(data)
        else:
            self.send_bytes(txt.encode("utf-8"))

    def send_dummy(self):
        pkt = build_dummy_174_frame()
        self.send_bytes(pkt)

    def send_bytes(self, b: bytes):
        if self.sim_chk.isChecked():
            self.append_log(f"[SIM TX] {b.hex(' ')}", color="green" if self.hex_verbose.isChecked() else None)
            self.signals.bytes_rx.emit(b)
            return
        if not self.serial_thread:
            QMessageBox.information(self, "Not connected", "Connect serial or enable simulation")
            return
        try:
            self.serial_thread.send(b)
            self.append_log(f"[TX] {b.hex(' ')}", color=None if not self.hex_verbose.isChecked() else "green")
        except Exception as e:
            self.append_log(f"[TX-ERR] {e}", color="red")

    def on_network_bytes(self, b: bytes):
        self.rx_buffer.extend(b)
        frames = parse_174_stream(self.rx_buffer)
        for f in frames:
            ps = f["PS"]
            d0 = f["D0"]
            d1 = f["D1"]
            d2 = f["D2"]
            pallets = f["Pallets"]

            self.ps_values.update(ps)
            self.d0_values.update(d0)
            self.d1_values.update(pallets[:5])
            self.d2_values.update(pallets[5:])
            self.pallet_table.update(pallets)

            # RF sum for plotting
            rf_sum = d0.get("rf", 0) + sum(p.get("val4", 0) for p in pallets)
            self.rf_sum_buf.append(rf_sum)
            self.rf_sum_curve.setData(list(range(len(self.rf_sum_buf))), list(self.rf_sum_buf))

            # PS Val2 plotting arbitrarily as voltage substitute
            self.ps_volt_buf.append(ps.get("val2", 0))
            self.ps_volt_curve.setData(list(range(len(self.ps_volt_buf))), list(self.ps_volt_buf))

            # Update value deques for variable graph
            for key, val in zip(["Val1", "Val2", "Val3", "Val4"], 
                                [ps.get("val1",0), ps.get("val2",0), ps.get("val3",0), ps.get("val4",0)]):
                self.value_deques["PS"][key].append(val)

            self.value_deques["D0"]["Val1"].append(d0.get("current", 0.0))
            self.value_deques["D0"]["Val2"].append(d0.get("voltage", 0.0))
            self.value_deques["D0"]["Val3"].append(d0.get("temp", 0.0))
            self.value_deques["D0"]["Val4"].append(d0.get("rf", 0.0))

            # D1 aggregated pallets
            if pallets:
                d1_p = pallets[:5]
                n1 = max(1, len(d1_p))
                for idx, key in enumerate(["val1", "val2", "val3", "val4"]):
                    self.value_deques["D1"][f"Val{idx+1}"].append(sum(p[key] for p in d1_p) / n1)
                d2_p = pallets[5:]
                n2 = max(1, len(d2_p))
                for idx, key in enumerate(["val1", "val2", "val3", "val4"]):
                    self.value_deques["D2"][f"Val{idx+1}"].append(sum(p[key] for p in d2_p) / n2)
            else:
                for key in ["Val1", "Val2", "Val3", "Val4"]:
                    self.value_deques["D1"][key].append(0.0)
                    self.value_deques["D2"][key].append(0.0)

            for i, p in enumerate(pallets):
                ent = f"P{i+1}"
                if ent not in self.value_deques:
                    continue
                self.value_deques[ent]["Val1"].append(p.get("val1", 0.0))
                self.value_deques[ent]["Val2"].append(p.get("val2", 0.0))
                self.value_deques[ent]["Val3"].append(p.get("val3", 0.0))
                self.value_deques[ent]["Val4"].append(p.get("val4", 0.0))

            self.on_selection_change()

            if self.hex_verbose.isChecked():
                self.append_log(f"[CONT] RFsum={rf_sum:.2f} raw:{f['RAW'].hex(' ')}", color="green")
            else:
                self.append_log(f"[CONT] RFsum={rf_sum:.2f}", color="green")

        if len(self.rx_buffer) > (1 << 20):
            self.rx_buffer = self.rx_buffer[-(1 << 20):]

    def append_log(self, text: str, color: str = None):
        if self.log.document().blockCount() > 1000:
            self.log.clear()
        if color == "green":
            self.log.setTextColor(Qt.darkGreen)
        elif color == "red":
            self.log.setTextColor(Qt.red)
        elif color == "blue":
            self.log.setTextColor(Qt.blue)
        else:
            self.log.setTextColor(Qt.white)
        self.log.append(text)
        self.log.setTextColor(Qt.white)
        self.log.ensureCursorVisible()

    def closeEvent(self, e):
        if self.serial_thread:
            self.serial_thread.stop()
        e.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
