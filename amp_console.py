import sys, socket, random, threading
from collections import deque
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QTabWidget, QGroupBox,
    QTextEdit, QDoubleSpinBox, QCheckBox, QMessageBox, QSplitter,
    QFrame
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
import pyqtgraph as pg

# ============================================================
#                      PACKET DEFINITION
# ============================================================
HEADER = 0xFF
HR = 0x52
HF = 0x46
HV = 0x56
HT = 0x54
HC = 0x43
HX = 0x58
HY = 0x59
HM = 0x4D
HZ = 0x5A  # Assuming 0x5A for 'Z', as document has typo with 0x4D for HZ
FRAME_LEN = 42  # Total length including HEADER

def build_dummy_frame(amp_id: int = 0x01) -> bytes:
    # Generate random values for the fields
    rfin = random.randint(0, 65535)
    fwd = random.randint(0, 2000)
    bb = fwd // 100
    cc = fwd % 100
    vswr_raw = random.randint(100, 200)  # 1.00 to 2.00 *100
    temp = random.uniform(30, 95)
    ee = int(temp)
    gg = int((temp - ee) * 100)
    current = random.uniform(0, 5)
    ii = int(current)
    jj = int((current - ii) * 100)
    v48 = random.uniform(46, 49)
    kk = int(v48)
    ll = int((v48 - kk) * 100)
    v28 = random.uniform(26, 29)
    mm = int(v28)
    nn = int((v28 - mm) * 100)
    v5_neg = random.uniform(4.8, 5.2)  # Absolute, will negate
    oo = int(v5_neg)
    pp = int((v5_neg - oo) * 100)
    v5 = random.uniform(4.8, 5.2)
    qq = int(v5)
    rr = int((v5 - qq) * 100)

    # Pack into bytes with AMP_ID for dual amp support (though document doesn't specify AMP_ID, adding for GUI)
    packet = bytes([HEADER, HR]) + bytes([rfin // 256, rfin % 256]) + bytes([HF]) + bytes([bb // 256, bb % 256, cc // 256, cc % 256]) + bytes([HV]) + bytes([vswr_raw // 256, vswr_raw % 256]) + bytes([HT]) + bytes([ee // 256, ee % 256, gg // 256, gg % 256]) + bytes([HC]) + bytes([ii // 256, ii % 256, jj // 256, jj % 256]) + bytes([HX]) + bytes([kk // 256, kk % 256, ll // 256, ll % 256]) + bytes([HY]) + bytes([mm // 256, mm % 256, nn // 256, nn % 256]) + bytes([HM]) + bytes([oo // 256, oo % 256, pp // 256, pp % 256]) + bytes([HZ]) + bytes([qq // 256, qq % 256, rr // 256, rr % 256])

    return packet

def parse_frames(stream: bytes):
    i, n = 0, len(stream)
    while i <= n - FRAME_LEN:
        if stream[i] == HEADER and stream[i + 1] == HR and i + FRAME_LEN <= n:
            raw = stream[i:i + FRAME_LEN]
            pos = 2
            rfin = raw[pos] * 256 + raw[pos + 1]; pos += 2
            if raw[pos] != HF:
                i += 1
                continue
            pos += 1
            bb = raw[pos] * 256 + raw[pos + 1]; pos += 2
            cc = raw[pos] * 256 + raw[pos + 1]; pos += 2
            fwd = bb * 100 + cc
            if raw[pos] != HV:
                i += 1
                continue
            pos += 1
            dd = raw[pos] * 256 + raw[pos + 1]; pos += 2
            vswr = dd / 100.0
            if raw[pos] != HT:
                i += 1
                continue
            pos += 1
            ee = raw[pos] * 256 + raw[pos + 1]; pos += 2
            gg = raw[pos] * 256 + raw[pos + 1]; pos += 2
            temp = ee + gg / 100.0
            if raw[pos] != HC:
                i += 1
                continue
            pos += 1
            ii = raw[pos] * 256 + raw[pos + 1]; pos += 2
            jj = raw[pos] * 256 + raw[pos + 1]; pos += 2
            idc = ii + jj / 100.0
            if raw[pos] != HX:
                i += 1
                continue
            pos += 1
            kk = raw[pos] * 256 + raw[pos + 1]; pos += 2
            ll = raw[pos] * 256 + raw[pos + 1]; pos += 2
            v48 = kk + ll / 100.0
            if raw[pos] != HY:
                i += 1
                continue
            pos += 1
            mm = raw[pos] * 256 + raw[pos + 1]; pos += 2
            nn = raw[pos] * 256 + raw[pos + 1]; pos += 2
            v28 = mm + nn / 100.0
            if raw[pos] != HM:
                i += 1
                continue
            pos += 1
            oo = raw[pos] * 256 + raw[pos + 1]; pos += 2
            pp = raw[pos] * 256 + raw[pos + 1]; pos += 2
            v5_neg = - (oo + pp / 100.0)
            if raw[pos] != HZ:
                i += 1
                continue
            pos += 1
            qq = raw[pos] * 256 + raw[pos + 1]; pos += 2
            rr = raw[pos] * 256 + raw[pos + 1]; pos += 2
            v5 = qq + rr / 100.0

            # Calculate REF from VSWR and FWD
            rho = (vswr - 1) / (vswr + 1) if vswr > 1 else 0.0
            ref = fwd * (rho ** 2)

            # Derive FAULT
            fault = temp >= 80 or vswr >= 1.9

            # Assign to Amp A or Amp B (assuming single frame per parse, duplicate for dual in GUI)
            yield {
                "amp": 1,  # Default to Amp A, GUI will handle Amp B duplication
                "FWD": fwd,
                "REF": ref,
                "TEMP": temp,
                "V48": v48,
                "V28": v28,
                "IDC": idc,
                "VSWR": vswr,
                "V5_NEG": v5_neg,
                "V5": v5,
                "RFIN": rfin,
                "VG1": random.uniform(1.2, 1.6),
                "VG2": random.uniform(1.2, 1.6),
                "VG3": random.uniform(1.2, 1.6),
                "RF_IN_OK": True,
                "RF_OUT_OK": True,
                "FAULT": fault,
                "RAW": raw
            }
            i += FRAME_LEN
        else:
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

def make_card(title, unit="", led=False):
    box = QGroupBox(title); box.setStyleSheet("QGroupBox{color:#cfd8dc;}")
    h = QHBoxLayout()
    val = QLabel("--" + (f" {unit}" if unit else "")); val.setAlignment(Qt.AlignCenter)
    val.setStyleSheet("font-size:16px; font-weight:600; color:#00e5ff;")
    h.addWidget(val, 1)
    ledw = None
    if led:
        ledw = LedIndicator("red"); h.addWidget(ledw, 0)
    box.setLayout(h)
    return box, val, ledw

def make_phase_shifters():
    g = QGroupBox("Phase Shifters"); grid = QGridLayout(); btns=[]
    for r in range(3):          # 3 groups
        for c in range(6):      # 6 channels each
            b = QPushButton(f"G{r+1}-{c+1}"); b.setCheckable(True)
            b.setStyleSheet("""
                QPushButton {background:#263238; color:#eceff1; border-radius:6px; padding:4px; min-width:48px}
                QPushButton:checked {background:#00bfa5; color:white;}
            """)
            grid.addWidget(b, r, c); btns.append(b)
    g.setLayout(grid); return g, btns

class AmplifierPanel(QWidget):
    def __init__(self, name="Amp A"):
        super().__init__()
        v = QVBoxLayout(self)
        t = QLabel(name); t.setStyleSheet("font-size:18px; font-weight:700; color:white;")
        v.addWidget(t)
        grid = QGridLayout()
        self.v28_box, self.v28_lbl, self.v28_led = make_card("V28","V",True)
        self.v48_box, self.v48_lbl, self.v48_led = make_card("V48","V",True)
        self.idc_box, self.idc_lbl, self.idc_led = make_card("IDC","A",True)
        self.vswr_box, self.vswr_lbl, self.vswr_led = make_card("VSWR","",True)
        self.temp_box, self.temp_lbl, self.temp_led = make_card("Temp","°C",True)
        self.fwd_box, self.fwd_lbl, _ = make_card("FWD","W")
        self.ref_box, self.ref_lbl, _ = make_card("REF","W")
        self.rfin_box, self.rfin_lbl, _ = make_card("RFin","")
        self.v5_neg_box, self.v5_neg_lbl, self.v5_neg_led = make_card("-5V","V",True)
        self.v5_box, self.v5_lbl, self.v5_led = make_card("5V","V",True)
        grid.addWidget(self.v28_box,0,0); grid.addWidget(self.v48_box,0,1); grid.addWidget(self.idc_box,0,2)
        grid.addWidget(self.vswr_box,1,0); grid.addWidget(self.temp_box,1,1); grid.addWidget(self.fwd_box,2,0); grid.addWidget(self.ref_box,2,1)
        grid.addWidget(self.rfin_box,3,0); grid.addWidget(self.v5_neg_box,3,1); grid.addWidget(self.v5_box,3,2)

        rf = QGroupBox("RF"); rh = QHBoxLayout()
        rh.addWidget(QLabel("IN")); self.rf_in_led = LedIndicator("red"); rh.addWidget(self.rf_in_led)
        rh.addWidget(QLabel("OUT")); self.rf_out_led = LedIndicator("red"); rh.addWidget(self.rf_out_led)
        rf.setLayout(rh); grid.addWidget(rf,1,2)
        v.addLayout(grid)

        self.graph = pg.PlotWidget(); self.graph.setBackground("#111"); self.graph.showGrid(x=True,y=True); self.graph.addLegend()
        self.cur_fwd = self.graph.plot(pen=pg.mkPen('#00e5ff', width=2), name="Forward")
        self.cur_ref = self.graph.plot(pen=pg.mkPen('#ffab00', width=2), name="Reflected")
        v.addWidget(self.graph,2)

        bias = QGroupBox("Bias (VG1–VG3)"); bg = QGridLayout()
        bg.addWidget(QLabel("Current:"), 0, 0)
        self.vg1_cur = QLabel("-- V"); self.vg1_cur.setStyleSheet("font-size:16px; font-weight:600; color:#00e5ff;")
        self.vg2_cur = QLabel("-- V"); self.vg2_cur.setStyleSheet("font-size:16px; font-weight:600; color:#00e5ff;")
        self.vg3_cur = QLabel("-- V"); self.vg3_cur.setStyleSheet("font-size:16px; font-weight:600; color:#00e5ff;")
        bg.addWidget(QLabel("VG1"), 0, 1); bg.addWidget(self.vg1_cur, 0, 2)
        bg.addWidget(QLabel("VG2"), 0, 3); bg.addWidget(self.vg2_cur, 0, 4)
        bg.addWidget(QLabel("VG3"), 0, 5); bg.addWidget(self.vg3_cur, 0, 6)
        bg.addWidget(QLabel("Set:"), 1, 0)
        self.vg1 = QDoubleSpinBox(); self.vg1.setRange(0,3); self.vg1.setDecimals(2); self.vg1.setValue(1.50)
        self.vg2 = QDoubleSpinBox(); self.vg2.setRange(0,3); self.vg2.setDecimals(2); self.vg2.setValue(1.50)
        self.vg3 = QDoubleSpinBox(); self.vg3.setRange(0,3); self.vg3.setDecimals(2); self.vg3.setValue(1.50)
        bg.addWidget(self.vg1, 1, 2); bg.addWidget(self.vg2, 1, 4); bg.addWidget(self.vg3, 1, 6)
        self.btn_apply = QPushButton("Apply"); bg.addWidget(self.btn_apply, 1, 7)
        bias.setLayout(bg); v.addWidget(bias)

        self.ps_group, self.ps_buttons = make_phase_shifters(); v.addWidget(self.ps_group)

        self.warn = QLabel(""); self.warn.setStyleSheet("color:#ff5252; font-weight:700;"); v.addWidget(self.warn)
        self.buf_fwd, self.buf_ref = deque(maxlen=200), deque(maxlen=200)

    def apply_fields(self, d: dict):
        self.v28_lbl.setText(f"{d['V28']:.2f} V")
        self.v48_lbl.setText(f"{d['V48']:.2f} V")
        self.idc_lbl.setText(f"{d['IDC']:.2f} A")
        self.vswr_lbl.setText(f"{d['VSWR']:.2f}")
        self.temp_lbl.setText(f"{d['TEMP']:.2f} °C")
        self.fwd_lbl.setText(f"{d['FWD']:.0f} W")
        self.ref_lbl.setText(f"{d['REF']:.0f} W")
        self.rfin_lbl.setText(f"{d['RFIN']:.0f}")
        self.v5_neg_lbl.setText(f"{d['V5_NEG']:.2f} V")
        self.v5_lbl.setText(f"{d['V5']:.2f} V")

        self.v28_led.set_color("green" if 26 <= d['V28'] <= 29 else "red")
        self.v48_led.set_color("green" if 46 <= d['V48'] <= 49 else "red")
        self.idc_led.set_color("green" if d['IDC'] < 5 else "red")
        self.vswr_led.set_color("green" if d['VSWR'] < 1.8 else "red")
        self.temp_led.set_color("green" if d['TEMP'] < 80 else "red")
        self.rf_in_led.set_color("green" if d['RF_IN_OK'] else "red")
        self.rf_out_led.set_color("green" if d['RF_OUT_OK'] else "red")
        self.v5_neg_led.set_color("green" if -5.2 <= d['V5_NEG'] <= -4.8 else "red")
        self.v5_led.set_color("green" if 4.8 <= d['V5'] <= 5.2 else "red")
        if d['TEMP'] >= 80 or d['VSWR'] >= 1.9 or d.get("FAULT", False):
            self.warn.setText("⚠️ SHUTDOWN: Unsafe condition detected!")
        else:
            self.warn.setText("")
        self.buf_fwd.append(d['FWD']); self.buf_ref.append(d['REF'])
        self.cur_fwd.setData(list(self.buf_fwd)); self.cur_ref.setData(list(self.buf_ref))

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
        self.setWindowTitle("LDTRM Dual Amplifier Console — TCP/UDP (Hercules-like)")
        self.resize(1450, 900)
        self.setStyleSheet("background:#0f1216; color:#cfd8dc;")

        central = QWidget(); self.setCentralWidget(central)
        vmain = QVBoxLayout(central)

        # ---------- Top bar (Hercules-style) ----------
        top = QHBoxLayout()
        title = QLabel("LDTRM Dual Amplifier Console"); title.setStyleSheet("font-size:20px; font-weight:800; color:white;")
        self.mode = QLineEdit("TCP Client"); self.mode.setFixedWidth(110)
        # Quick selector hint: you can type "TCP Client", "TCP Server", "UDP"
        self.sim = QCheckBox("Simulation"); self.sim.setChecked(True)
        self.local_port = QLineEdit("5000"); self.local_port.setFixedWidth(90)
        self.remote_ip = QLineEdit("192.168.18.91"); self.remote_ip.setFixedWidth(150)  # Updated to match Arduino
        self.remote_port = QLineEdit("5000"); self.remote_port.setFixedWidth(90)
        self.btn_conn = QPushButton("Connect"); self.btn_conn.clicked.connect(self.toggle_conn)
        self.net_led = LedIndicator("red")

        top.addWidget(title); top.addStretch()
        top.addWidget(QLabel("Mode:")); top.addWidget(self.mode)
        top.addWidget(QLabel("Local Port:")); top.addWidget(self.local_port)
        top.addWidget(QLabel("Remote IP:")); top.addWidget(self.remote_ip)
        top.addWidget(QLabel("Remote Port:")); top.addWidget(self.remote_port)
        top.addWidget(self.sim); top.addWidget(self.net_led); top.addWidget(self.btn_conn)
        vmain.addLayout(top)

        # ---------- Splitter: left amps / right logs+send ----------
        split = QSplitter(); split.setOrientation(Qt.Horizontal); vmain.addWidget(split,1)

        amps = QWidget(); ah = QHBoxLayout(amps)
        self.ampA = AmplifierPanel("Amp A"); self.ampB = AmplifierPanel("Amp B")
        ah.addWidget(self.ampA,1); ah.addWidget(self.ampB,1)
        split.addWidget(amps)

        right = QWidget(); rv = QVBoxLayout(right)

        # Send panel like Hercules
        send_box = QGroupBox("Send"); sh = QHBoxLayout()
        self.hex_check = QCheckBox("HEX"); self.hex_check.setChecked(True)
        self.send_edit = QLineEdit("FF 52 ...")  # Placeholder, update with full dummy frame
        self.btn_send = QPushButton("Send"); self.btn_send.clicked.connect(self.do_send)
        self.btn_send_dummy = QPushButton("Send Dummy Frames (A+B)"); self.btn_send_dummy.clicked.connect(self.send_dummy_both)
        sh.addWidget(self.send_edit,1); sh.addWidget(self.hex_check); sh.addWidget(self.btn_send); sh.addWidget(self.btn_send_dummy)
        send_box.setLayout(sh); rv.addWidget(send_box,0)

        # Logs
        self.log = QTextEdit(); self.log.setReadOnly(True)
        self.log.setStyleSheet("background:#14181d; color:#9cff57;")
        rv.addWidget(self.log,1)
        split.addWidget(right)
        split.setSizes([900, 500])

        # RX buffer
        self.rx_buffer = bytearray()

        # Network signals
        self.signals = NetSignals()
        self.signals.bytes_rx.connect(self.on_rx)
        self.signals.state.connect(self.on_state)
        self.net_thread = None  # TcpClient/TcpServer/UdpNode

        # Simulation timer
        self.timer = QTimer(self); self.timer.timeout.connect(self.tick_sim); self.timer.start(1000)

    # ---------------- Simulation ----------------
    def tick_sim(self):
        if not self.sim.isChecked(): return
        data = build_dummy_frame()  # Single frame for now, GUI will duplicate
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
        if len(self.rx_buffer) > 65536:
            self.rx_buffer[:] = self.rx_buffer[-65536:]
        for frame in parse_frames(bytes(self.rx_buffer)):
            self.ampA.apply_fields(frame)  # Apply to Amp A
            self.ampB.apply_fields(frame)  # Duplicate for Amp B (assuming single frame format for now)
        # Trim buffer up to last complete frame
        last_start = bytes(self.rx_buffer).rfind(bytes([HEADER, HR]))
        if last_start >= 0 and len(self.rx_buffer) > last_start + FRAME_LEN:
            self.rx_buffer = self.rx_buffer[last_start:]

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
            # Allow "FF 01 02" or "ff0102"
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

    def send_dummy_both(self):
        self._send_bytes(build_dummy_frame() + build_dummy_frame())  # Send two frames for A and B

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