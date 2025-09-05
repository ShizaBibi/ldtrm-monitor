import sys, socket, random, threading
from collections import deque

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QTextEdit,
    QDoubleSpinBox, QCheckBox, QMessageBox, QSplitter, QTableWidget,
    QTableWidgetItem
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
import pyqtgraph as pg

# ============================================================
#                      PACKET DEFINITION (190 + 1 header)
# ============================================================
HEADER = 0xFF
FRAME_LEN = 191   # 1 header + 190 payload

# ---- Payload structure (fixed, big-endian) ----
# [Mega ID (2)]
# Drivers (3 * 16 each) -> per driver:
#   Temp (2 int, 2 dec), Current (2 int, 2 dec), Voltage (2 int, 2 dec), Control (2), Reserved (2) = 16
# Pallets (10 * 14 each) -> per pallet:
#   Temp (2 int, 2 dec), Current (2 int, 2 dec), Voltage (2 int, 2 dec), RF (2) = 14

def _pack_i_d(val: float):
    """Pack a float as 2-byte int + 2-byte dec (scaled by 100), big-endian."""
    i = int(val)
    d = int(round((val - i) * 100))
    if d < 0:
        d = 0
    if i < 0:
        i = 0
    return i.to_bytes(2, "big") + d.to_bytes(2, "big")

def _unpack_i_d(b, pos):
    """Unpack 2-byte int + 2-byte dec -> float, return (value, new_pos)."""
    i = int.from_bytes(b[pos:pos+2], "big"); pos += 2
    d = int.from_bytes(b[pos:pos+2], "big"); pos += 2
    return i + d / 100.0, pos

def build_dummy_frame(mega_id: int = 0x01) -> bytes:
    """
    Build a single full frame:
    0xFF + [190 bytes payload]
    """
    payload = bytearray()

    # Mega ID (2 bytes)
    payload += mega_id.to_bytes(2, "big")

    # 3 Drivers × 16 bytes
    for d in range(3):
        temp = random.uniform(30.0, 80.0)      # °C
        cur  = random.uniform(0.0, 10.0)       # A
        volt = random.uniform(24.0, 52.0)      # V (generic driver voltage)
        control = random.randint(0, 0xFFFF)    # 2 bytes
        payload += _pack_i_d(temp)
        payload += _pack_i_d(cur)
        payload += _pack_i_d(volt)
        payload += control.to_bytes(2, "big")
        payload += (0).to_bytes(2, "big")      # reserved to make 16

    # 10 Pallets × 14 bytes
    for p in range(10):
        temp = random.uniform(25.0, 75.0)     # °C
        cur  = random.uniform(0.0, 6.5)       # A
        volt = random.uniform(26.0, 49.0)     # V
        rf   = random.randint(0, 2000)        # W, integer only (2 bytes)
        payload += _pack_i_d(temp)
        payload += _pack_i_d(cur)
        payload += _pack_i_d(volt)
        payload += rf.to_bytes(2, "big")

    assert len(payload) == 190, f"Payload size {len(payload)} != 190"
    return bytes([HEADER]) + bytes(payload)

def parse_frames(stream: bytes):
    """
    Generator that scans a byte stream, yields parsed frames as:
    {
      "mega_id": int,
      "drivers": [
         {"temp": float, "current": float, "voltage": float, "control": int},
         ...
      ],
      "pallets": [
         {"index": 1..10, "temp": float, "current": float, "voltage": float, "rf": int},
         ...
      ],
      "RAW": full_frame_bytes
    }
    """
    i, n = 0, len(stream)
    while i <= n - FRAME_LEN:
        if stream[i] != HEADER:
            i += 1
            continue

        start = i
        end = i + FRAME_LEN
        raw = stream[start:end]
        # if not enough bytes for full frame, break
        if len(raw) < FRAME_LEN:
            break

        # Parse payload
        p = 1  # after header
        payload = raw[p:]
        pos = 0

        try:
            mega_id = int.from_bytes(payload[pos:pos+2], "big"); pos += 2

            drivers = []
            for _ in range(3):
                temp, pos = _unpack_i_d(payload, pos)
                current, pos = _unpack_i_d(payload, pos)
                voltage, pos = _unpack_i_d(payload, pos)
                control = int.from_bytes(payload[pos:pos+2], "big"); pos += 2
                pos += 2  # reserved
                drivers.append({
                    "temp": temp,
                    "current": current,
                    "voltage": voltage,
                    "control": control
                })

            pallets = []
            for idx in range(10):
                temp, pos = _unpack_i_d(payload, pos)
                current, pos = _unpack_i_d(payload, pos)
                voltage, pos = _unpack_i_d(payload, pos)
                rf = int.from_bytes(payload[pos:pos+2], "big"); pos += 2
                pallets.append({
                    "index": idx + 1,
                    "temp": temp,
                    "current": current,
                    "voltage": voltage,
                    "rf": rf
                })

            # Final sanity
            if pos != 190:
                # payload length mismatch: skip
                i += 1
                continue

            yield {
                "mega_id": mega_id,
                "drivers": drivers,
                "pallets": pallets,
                "RAW": raw
            }
            i += FRAME_LEN

        except Exception:
            # parsing error -> resync
            i += 1

# ============================================================
#                        UI WIDGETS
# ============================================================
class LedIndicator(QLabel):
    def __init__(self, color="red", d=12):
        super().__init__()
        self.d = d
        self.setFixedSize(d, d)
        self.set_color(color)
    def set_color(self, color):
        r = self.d // 2
        self.setStyleSheet(f"border-radius:{r}px; background:{color}; border:1px solid #111;")

def make_card(title, unit=""):
    box = QGroupBox(title); box.setStyleSheet("QGroupBox{color:#cfd8dc;}")
    h = QHBoxLayout()
    val = QLabel("--" + (f" {unit}" if unit else "")); val.setAlignment(Qt.AlignCenter)
    val.setStyleSheet("font-size:16px; font-weight:600; color:#00e5ff;")
    h.addWidget(val, 1)
    box.setLayout(h)
    return box, val

class DriverPanel(QWidget):
    def __init__(self, name="Driver"):
        super().__init__()
        v = QVBoxLayout(self)
        t = QLabel(name); t.setStyleSheet("font-size:18px; font-weight:700; color:white;")
        v.addWidget(t)

        grid = QGridLayout()
        self.temp_box, self.temp_lbl = make_card("Temp", "°C")
        self.cur_box, self.cur_lbl   = make_card("Current", "A")
        self.volt_box, self.volt_lbl = make_card("Voltage", "V")
        self.ctrl_box, self.ctrl_lbl = make_card("Control", "")

        grid.addWidget(self.temp_box, 0, 0)
        grid.addWidget(self.cur_box, 0, 1)
        grid.addWidget(self.volt_box, 1, 0)
        grid.addWidget(self.ctrl_box, 1, 1)

        v.addLayout(grid)

        # Trend graph for RF-like overview (we'll plot pallet RF sum)
        self.graph = pg.PlotWidget()
        self.graph.setBackground("#111"); self.graph.showGrid(x=True, y=True)
        self.graph.addLegend()
        self.cur_trend = self.graph.plot(pen=pg.mkPen('#00e5ff', width=2), name="Pallet RF Sum")
        v.addWidget(self.graph, 1)
        self.buf_trend = deque(maxlen=200)

    def apply_driver(self, d: dict, rf_sum: float = None):
        self.temp_lbl.setText(f"{d['temp']:.2f} °C")
        self.cur_lbl.setText(f"{d['current']:.2f} A")
        self.volt_lbl.setText(f"{d['voltage']:.2f} V")
        self.ctrl_lbl.setText(f"0x{d['control']:04X}")
        if rf_sum is not None:
            self.buf_trend.append(rf_sum)
            self.cur_trend.setData(list(self.buf_trend))

class PalletTable(QGroupBox):
    def __init__(self):
        super().__init__("Pallets (P1–P10)")
        self.setStyleSheet("QGroupBox{color:#cfd8dc;}")
        v = QVBoxLayout(self)
        self.table = QTableWidget(10, 5)
        self.table.setHorizontalHeaderLabels(["P#", "Temp (°C)", "Current (A)", "Voltage (V)", "RF (W)"])
        self.table.verticalHeader().setVisible(False)
        self.table.setStyleSheet("QTableWidget{background:#14181d; color:#e0f7fa; gridline-color:#263238;}")
        for r in range(10):
            self.table.setItem(r, 0, QTableWidgetItem(str(r+1)))
        v.addWidget(self.table)

    def apply_pallets(self, pallets):
        for r, p in enumerate(pallets):
            self.table.setItem(r, 0, QTableWidgetItem(str(p["index"])))
            self.table.setItem(r, 1, QTableWidgetItem(f"{p['temp']:.2f}"))
            self.table.setItem(r, 2, QTableWidgetItem(f"{p['current']:.2f}"))
            self.table.setItem(r, 3, QTableWidgetItem(f"{p['voltage']:.2f}"))
            self.table.setItem(r, 4, QTableWidgetItem(f"{p['rf']}"))

# ============================================================
#                     NETWORK ABSTRACTIONS
# ============================================================
class NetSignals(QObject):
    bytes_rx = Signal(bytes)
    state    = Signal(bool, str)   # connected?, message

class TcpClient(threading.Thread):
    def __init__(self, host, port, sig: NetSignals):
        super().__init__(daemon=True); self.host=host; self.port=port; self.sig=sig
        self._stop = threading.Event(); self.sock=None
    def run(self):
        try:
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.sock.settimeout(5); self.sock.connect((self.host,self.port))
            self.sock.settimeout(0.25); self.sig.state.emit(True,f"TCP Client connected to {self.host}:{self.port}")
            while not self._stop.is_set():
                try:
                    d=self.sock.recv(4096)
                    if not d: break
                    self.sig.bytes_rx.emit(d)
                except socket.timeout:
                    continue
        except Exception as e:
            self.sig.state.emit(False,f"TCP Client error: {e}")
        finally:
            try:
                if self.sock: self.sock.close()
            except: pass
            self.sig.state.emit(False,"TCP Client disconnected")
    def send(self,b:bytes):
        try:
            if self.sock: self.sock.sendall(b)
        except: pass
    def stop(self): self._stop.set()

class TcpServer(threading.Thread):
    def __init__(self, port, sig: NetSignals):
        super().__init__(daemon=True); self.port=port; self.sig=sig
        self._stop=threading.Event(); self.srv=None; self.cli=None
    def run(self):
        try:
            self.srv=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.srv.bind(("",self.port)); self.srv.listen(1)
            self.srv.settimeout(0.5)
            self.sig.state.emit(True,f"TCP Server listening on 0.0.0.0:{self.port}")
            while not self._stop.is_set():
                if not self.cli:
                    try:
                        c,addr=self.srv.accept(); c.settimeout(0.25); self.cli=c
                        self.sig.state.emit(True,f"TCP Server client {addr[0]}:{addr[1]}")
                    except socket.timeout:
                        continue
                else:
                    try:
                        d=self.cli.recv(4096)
                        if not d: self.cli.close(); self.cli=None; continue
                        self.sig.bytes_rx.emit(d)
                    except socket.timeout:
                        continue
        except Exception as e:
            self.sig.state.emit(False,f"TCP Server error: {e}")
        finally:
            try:
                if self.cli: self.cli.close()
                if self.srv: self.srv.close()
            except: pass
            self.sig.state.emit(False,"TCP Server stopped")
    def send(self,b:bytes):
        try:
            if self.cli: self.cli.sendall(b)
        except: pass
    def stop(self): self._stop.set()

class UdpNode(threading.Thread):
    def __init__(self, local_port, remote_ip, remote_port, sig: NetSignals):
        super().__init__(daemon=True); self.lp=local_port; self.rip=remote_ip; self.rp=remote_port; self.sig=sig
        self._stop=threading.Event(); self.sock=None; self.peer=None
    def run(self):
        try:
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.sock.bind(("",self.lp))
            self.sock.settimeout(0.25)
            self.sig.state.emit(True,f"UDP bound on 0.0.0.0:{self.lp}")
            if self.rip and self.rp: self.peer=(self.rip,self.rp)
            while not self._stop.is_set():
                try:
                    d,addr=self.sock.recvfrom(4096)
                    self.sig.bytes_rx.emit(d)
                    if not self.peer: self.peer=addr  # learn first sender
                except socket.timeout:
                    continue
        except Exception as e:
            self.sig.state.emit(False,f"UDP error: {e}")
        finally:
            try:
                if self.sock: self.sock.close()
            except: pass
            self.sig.state.emit(False,"UDP stopped")
    def send(self,b:bytes):
        try:
            if self.sock and self.peer: self.sock.sendto(b,self.peer)
        except: pass
    def stop(self): self._stop.set()

# ============================================================
#                         MAIN WINDOW
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LDTRM 3-Driver / 10-Pallet Console — TCP/UDP (Hercules-like)")
        self.resize(1450, 900)
        self.setStyleSheet("background:#0f1216; color:#cfd8dc;")

        central = QWidget(); self.setCentralWidget(central)
        vmain = QVBoxLayout(central)

        # ---------- Top bar ----------
        top = QHBoxLayout()
        title = QLabel("LDTRM Console (190-byte Protocol)"); title.setStyleSheet("font-size:20px; font-weight:800; color:white;")
        self.mode = QLineEdit("TCP Client"); self.mode.setFixedWidth(110)
        self.sim = QCheckBox("Simulation"); self.sim.setChecked(True)
        self.local_port = QLineEdit("5000"); self.local_port.setFixedWidth(90)
        self.remote_ip = QLineEdit("192.168.18.91"); self.remote_ip.setFixedWidth(150)
        self.remote_port = QLineEdit("5000"); self.remote_port.setFixedWidth(90)
        self.btn_conn = QPushButton("Connect"); self.btn_conn.clicked.connect(self.toggle_conn)
        self.net_led = LedIndicator("red")
        self.mega_lbl = QLabel("MegaID: --"); self.mega_lbl.setStyleSheet("font-weight:700; color:#fff;")

        top.addWidget(title); top.addStretch()
        top.addWidget(self.mega_lbl)
        top.addWidget(QLabel("Mode:")); top.addWidget(self.mode)
        top.addWidget(QLabel("Local Port:")); top.addWidget(self.local_port)
        top.addWidget(QLabel("Remote IP:")); top.addWidget(self.remote_ip)
        top.addWidget(QLabel("Remote Port:")); top.addWidget(self.remote_port)
        top.addWidget(self.sim); top.addWidget(self.net_led); top.addWidget(self.btn_conn)
        vmain.addLayout(top)

        # ---------- Splitter ----------
        split = QSplitter(); split.setOrientation(Qt.Vertical); vmain.addWidget(split,1)

        # Upper: Drivers row
        drivers_widget = QWidget(); dh = QHBoxLayout(drivers_widget)
        self.d0 = DriverPanel("Driver 0")
        self.d1 = DriverPanel("Driver 1")
        self.d2 = DriverPanel("Driver 2")
        dh.addWidget(self.d0,1); dh.addWidget(self.d1,1); dh.addWidget(self.d2,1)
        split.addWidget(drivers_widget)

        # Middle: Pallets table
        self.pallets = PalletTable()
        pallets_wrap = QWidget(); pv = QVBoxLayout(pallets_wrap); pv.addWidget(self.pallets)
        split.addWidget(pallets_wrap)

        # Lower: Right logs / send
        bottom = QWidget(); bv = QVBoxLayout(bottom)

        send_box = QGroupBox("Send"); sh = QHBoxLayout()
        self.hex_check = QCheckBox("HEX"); self.hex_check.setChecked(True)
        self.send_edit = QLineEdit("FF ...")  # you can paste hex here if needed
        self.btn_send = QPushButton("Send"); self.btn_send.clicked.connect(self.do_send)
        self.btn_send_dummy = QPushButton("Send Dummy Frame"); self.btn_send_dummy.clicked.connect(self.send_dummy_once)
        sh.addWidget(self.send_edit,1); sh.addWidget(self.hex_check); sh.addWidget(self.btn_send); sh.addWidget(self.btn_send_dummy)
        send_box.setLayout(sh); bv.addWidget(send_box,0)

        self.log = QTextEdit(); self.log.setReadOnly(True)
        self.log.setStyleSheet("background:#14181d; color:#9cff57;")
        bv.addWidget(self.log,1)

        split.addWidget(bottom)
        split.setSizes([400, 300, 300])

        # RX buffer
        self.rx_buffer = bytearray()

        # Network signals
        self.signals = NetSignals()
        self.signals.bytes_rx.connect(self.on_rx)
        self.signals.state.connect(self.on_state)
        self.net_thread = None

        # Simulation timer
        self.timer = QTimer(self); self.timer.timeout.connect(self.tick_sim); self.timer.start(1000)

    # ---------------- Simulation ----------------
    def tick_sim(self):
        if not self.sim.isChecked(): return
        data = build_dummy_frame()  # Single frame
        self.on_rx(data)

    # ---------------- Network control -----------
    def toggle_conn(self):
        if self.net_thread:
            self.net_thread.stop(); self.net_thread = None
            self.btn_conn.setText("Connect"); self.net_led.set_color("red")
            return
        if self.sim.isChecked():
            QMessageBox.information(self,"Simulation ON","Turn off Simulation to open sockets.")
            return
        mode = self.mode.text().strip().lower()
        try:
            lp = int(self.local_port.text()); rp = int(self.remote_port.text())
        except ValueError:
            QMessageBox.warning(self,"Port","Invalid port number"); return
        rip = self.remote_ip.text().strip()

        if mode == "tcp client":
            self.net_thread = TcpClient(rip, rp, self.signals)
        elif mode == "tcp server":
            self.net_thread = TcpServer(lp, self.signals)
        elif mode == "udp":
            self.net_thread = UdpNode(lp, rip, rp, self.signals)
        else:
            QMessageBox.warning(self,"Mode","Type one of: TCP Client, TCP Server, UDP")
            return
        self.net_thread.start(); self.btn_conn.setText("Disconnect")

    def on_state(self, connected: bool, msg: str):
        self.net_led.set_color("green" if connected else "red")
        self.log.append(msg)

    # ---------------- RX path -------------------
    def on_rx(self, data: bytes):
        # Log like Hercules
        self.log.append(data.hex(" "))

        # Buffer and parse
        self.rx_buffer.extend(data)
        if len(self.rx_buffer) > 1<<20:  # 1 MB cap
            self.rx_buffer[:] = self.rx_buffer[-(1<<20):]

        # Parse all full frames from buffer
        consumed_upto = 0
        for frame in parse_frames(bytes(self.rx_buffer)):
            raw = frame["RAW"]
            # find this raw in buffer to mark consumption point
            idx = self.rx_buffer.find(raw, consumed_upto)
            if idx != -1:
                consumed_upto = idx + len(raw)

            # ---- Apply to UI ----
            self.mega_lbl.setText(f"MegaID: {frame['mega_id']}")
            drivers = frame["drivers"]
            pallets = frame["pallets"]

            rf_sum = sum(p["rf"] for p in pallets) if pallets else 0.0
            if len(drivers) >= 1: self.d0.apply_driver(drivers[0], rf_sum)
            if len(drivers) >= 2: self.d1.apply_driver(drivers[1], rf_sum)
            if len(drivers) >= 3: self.d2.apply_driver(drivers[2], rf_sum)

            self.pallets.apply_pallets(pallets)

        # Trim buffer up to last consumed full frame
        if consumed_upto > 0:
            self.rx_buffer = self.rx_buffer[consumed_upto:]
        else:
            # soft resync: keep last 2*FRAME_LEN bytes to catch boundary
            if len(self.rx_buffer) > 2*FRAME_LEN:
                self.rx_buffer = self.rx_buffer[-2*FRAME_LEN:]

    # ---------------- TX helpers ----------------
    def _send_bytes(self, b: bytes):
        if self.sim.isChecked():
            self.log.append("[SIM TX] " + b.hex(" "))
            return
        if not self.net_thread:
            QMessageBox.information(self,"Not connected","Open a socket first."); return
        try:
            self.net_thread.send(b)
            self.log.append("[TX] " + b.hex(" "))
        except Exception as e:
            self.log.append(f"[TX-ERR] {e}")

    def do_send(self):
        txt = self.send_edit.text().strip()
        if self.hex_check.isChecked():
            cleaned = txt.replace(" ", "").replace(",", "")
            if len(cleaned) % 2 != 0:
                QMessageBox.warning(self,"HEX","Hex length must be even"); return
            try:
                data = bytes.fromhex(cleaned)
            except ValueError:
                QMessageBox.warning(self,"HEX","Invalid hex string"); return
            self._send_bytes(data)
        else:
            self._send_bytes(txt.encode("utf-8"))

    def send_dummy_once(self):
        self._send_bytes(build_dummy_frame())

    # ---------------- Close ---------------------
    def closeEvent(self, e):
        try:
            if self.net_thread: self.net_thread.stop()
        except: pass
        e.accept()

# -------------------- main ---------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec())
