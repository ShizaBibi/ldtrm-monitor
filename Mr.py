import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
    QTextEdit, QLineEdit, QHBoxLayout
)

class PacketViewer(QWidget):
    def __init__(self, port="COM5", baud=57600):
        super().__init__()
        self.setWindowTitle("Aggregator Packet Viewer")

        # Serial setup
        self.ser = serial.Serial(port, baud, timeout=1)

        # Widgets
        layout = QVBoxLayout()
        self.packetBox = QTextEdit()
        self.packetBox.setReadOnly(True)

        self.requestInput = QLineEdit()
        self.requestInput.setPlaceholderText("Enter request: e.g. 55 D1 02 00")
        self.sendBtn = QPushButton("Send Request")
        self.responseLbl = QLabel("Response: ---")

        # Layout
        hl = QHBoxLayout()
        hl.addWidget(self.requestInput)
        hl.addWidget(self.sendBtn)

        layout.addWidget(QLabel("Incoming Packets (188B):"))
        layout.addWidget(self.packetBox)
        layout.addLayout(hl)
        layout.addWidget(self.responseLbl)
        self.setLayout(layout)

        # Connections
        self.sendBtn.clicked.connect(self.send_request)

        # Timer to read packets
        from PyQt5.QtCore import QTimer
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_data)
        self.timer.start(200)

    def read_data(self):
        if self.ser.in_waiting >= 188:
            data = self.ser.read(188)
            hexstr = " ".join(f"{b:02X}" for b in data)
            self.packetBox.append(hexstr)

    def send_request(self):
        try:
            req_str = self.requestInput.text().strip().split()
            req = bytes(int(x, 16) for x in req_str)
            self.ser.write(req)
            resp = self.ser.read(4)
            if resp:
                self.responseLbl.setText("Response: " + " ".join(f"{b:02X}" for b in resp))
        except Exception as e:
            self.responseLbl.setText(f"Error: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    # ⚠️ Change COM port below to match your Mega 3
    viewer = PacketViewer(port="COM5", baud=57600)
    viewer.show()
    sys.exit(app.exec_())
