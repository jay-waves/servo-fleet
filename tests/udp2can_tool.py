      
import argparse
import html
import socket
import sys
from datetime import datetime

from PyQt5.QtCore import QObject, QThread, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
	QApplication,
	QHBoxLayout,
	QCheckBox,
	QComboBox,
	QGridLayout,
	QGroupBox,
	QLabel,
	QLineEdit,
	QMainWindow,
	QMessageBox,
	QPushButton,
	QTextEdit,
	QVBoxLayout,
	QWidget,
)


FORWARD_PROTO_FRAME_STX = 0xAA
FORWARD_PROTO_FRAME_ETX = 0xBB
# Type of frame:
# BIT0: 0 = CAN -> UDP, 1 = UDP -> CAN
# BIT1-BIT7: RESERVED
FORWARD_PROTO_TYPE_CAN_TO_UDP = 0x00
FORWARD_PROTO_TYPE_UDP_TO_CAN = 0x01

CAN_MSG_ENTRY_SIZE = 14
CAN_ID_EXTD_FLAG = 0x80000000


def crc16_ccitt(data: bytes) -> int:
	"""CRC16-CCITT: poly=0x1021, init=0x0000, MSB-first."""
	crc = 0x0000
	for byte in data:
		crc ^= byte << 8
		for _ in range(8):
			if crc & 0x8000:
				crc = ((crc << 1) & 0xFFFF) ^ 0x1021
			else:
				crc = (crc << 1) & 0xFFFF
	return crc & 0xFFFF


def build_frame(frame_type: int, messages: list) -> bytes:
	payload = bytearray()
	payload.append(len(messages) & 0xFF)

	for item in messages:
		port = int(item.get("port", 0)) & 0xFF
		can_id = int(item["id"])
		is_ext = bool(item.get("ext", False))
		data = bytes(item.get("data", b""))
		if len(data) > 8:
			raise ValueError("CAN data length must be <= 8")

		id_net = can_id | (CAN_ID_EXTD_FLAG if is_ext else 0)

		entry = bytearray(CAN_MSG_ENTRY_SIZE)
		entry[0] = port
		entry[1:5] = id_net.to_bytes(4, byteorder="big", signed=False)
		entry[5] = len(data)
		entry[6:6 + len(data)] = data
		payload.extend(entry)

	length = len(payload)
	header = bytearray()
	header.append(FORWARD_PROTO_FRAME_STX)
	header.append(frame_type & 0xFF)
	header.extend(length.to_bytes(2, byteorder="big", signed=False))

	crc_input = bytes(header[1:]) + bytes(payload)  # type + length + payload
	crc = crc16_ccitt(crc_input)

	frame = bytes(header) + bytes(payload) + bytes([crc & 0xFF, (crc >> 8) & 0xFF, FORWARD_PROTO_FRAME_ETX])
	return frame


def parse_frame(raw: bytes) -> dict:
	if len(raw) < 8:
		raise ValueError("frame too short")
	if raw[0] != FORWARD_PROTO_FRAME_STX:
		raise ValueError("invalid STX")

	frame_type = raw[1]
	payload_len = int.from_bytes(raw[2:4], byteorder="big", signed=False)
	expected_total = 4 + payload_len + 2 + 1
	if len(raw) != expected_total:
		raise ValueError(f"invalid frame length, expected {expected_total}, got {len(raw)}")

	crc_calc = crc16_ccitt(raw[1:4 + payload_len])
	crc_recv = raw[4 + payload_len] | (raw[5 + payload_len] << 8)
	if crc_calc != crc_recv:
		raise ValueError(f"CRC mismatch calc=0x{crc_calc:04X}, recv=0x{crc_recv:04X}")

	if raw[6 + payload_len] != FORWARD_PROTO_FRAME_ETX:
		raise ValueError("invalid ETX")

	payload = raw[4:4 + payload_len]
	if payload_len < 1:
		raise ValueError("payload too short")

	msg_count = payload[0]
	if payload_len != 1 + msg_count * CAN_MSG_ENTRY_SIZE:
		raise ValueError("payload length does not match msg_count")

	msgs = []
	offset = 1
	for _ in range(msg_count):
		port = payload[offset]
		id_net = int.from_bytes(payload[offset + 1:offset + 5], byteorder="big", signed=False)
		dlc = payload[offset + 5]
		if dlc > 8:
			raise ValueError("invalid CAN DLC > 8")

		is_ext = (id_net & CAN_ID_EXTD_FLAG) != 0
		can_id = id_net & ~CAN_ID_EXTD_FLAG if is_ext else id_net
		data = bytes(payload[offset + 6:offset + 6 + dlc])

		msgs.append({
			"port": port,
			"id": can_id,
			"ext": is_ext,
			"len": dlc,
			"data": data,
		})
		offset += CAN_MSG_ENTRY_SIZE

	return {
		"type": frame_type,
		"msg_count": msg_count,
		"messages": msgs,
	}


def parse_hex_data(text: str) -> bytes:
	clean = text.replace(" ", "").replace(",", "").replace("-", "")
	if clean == "":
		return b""
	if len(clean) % 2 != 0:
		raise ValueError("hex data length must be even")
	try:
		data = bytes.fromhex(clean)
	except ValueError as exc:
		raise ValueError("invalid hex data") from exc
	if len(data) > 8:
		raise ValueError("hex data must be <= 8 bytes")
	return data


class UdpRxWorker(QObject):
	frame_rx = pyqtSignal(object, str, int)
	parse_error = pyqtSignal(str, str, int)
	log = pyqtSignal(str)

	def __init__(self, local_port: int):
		super().__init__()
		self.local_port = local_port
		self._running = True
		self._sock = None

	def stop(self):
		self._running = False
		if self._sock is not None:
			try:
				self._sock.close()
			except OSError:
				pass

	def run(self):
		self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self._sock.settimeout(0.2)
		self._sock.bind(("0.0.0.0", self.local_port))
		self.log.emit(f"UDP listening on 0.0.0.0:{self.local_port}")

		while self._running:
			try:
				data, (ip, port) = self._sock.recvfrom(2048)
			except socket.timeout:
				continue
			except OSError:
				break

			try:
				frame = parse_frame(data)
				self.frame_rx.emit(frame, ip, port)
			except ValueError as exc:
				self.parse_error.emit(str(exc), ip, port)

	def send_frame(self, frame: bytes, target_ip: str, target_port: int):
		if self._sock is None:
			self.log.emit("TX failed: socket is not ready")
			return
		try:
			self._sock.sendto(frame, (target_ip, target_port))
		except OSError as exc:
			self.log.emit(f"TX failed: {exc}")


class MainWindow(QMainWindow):
	def __init__(self, local_port: int, target_ip: str, target_port: int):
		super().__init__()
		self.local_port = local_port
		self.target_ip = target_ip
		self.target_port = target_port
		self.repeat_timer = QTimer(self)
		self.repeat_timer.timeout.connect(self.send_current_frame)

		self.setWindowTitle("UDP2CAN Debug Tool")
		self.resize(900, 640)

		self._build_ui()
		self.update_send_button_text()
		self._start_rx_worker()

	def _build_ui(self):
		root = QWidget()
		self.setCentralWidget(root)
		root_layout = QVBoxLayout(root)

		target_info = QLabel(
			f"Local Port(RX/TX source): {self.local_port}    Target: {self.target_ip}:{self.target_port}"
		)
		root_layout.addWidget(target_info)

		log_tools = QHBoxLayout()
		log_tools.addWidget(QLabel("接收日志:"))
		self.btn_clear_log = QPushButton("清空日志")
		self.btn_clear_log.clicked.connect(self.text_out_clear)
		log_tools.addWidget(self.btn_clear_log)
		log_tools.addStretch(1)
		root_layout.addLayout(log_tools)

		self.text_out = QTextEdit()
		self.text_out.setObjectName("text_out")
		self.text_out.setReadOnly(True)
		root_layout.addWidget(self.text_out, stretch=1)

		send_group = QGroupBox("发送")
		send_layout = QGridLayout(send_group)

		send_layout.addWidget(QLabel("帧ID (hex):"), 0, 0)
		self.edit_id = QLineEdit("7FF")
		self.edit_id.setPlaceholderText("示例: 123 或 0x18FF50E5")
		send_layout.addWidget(self.edit_id, 0, 1)

		self.chk_ext = QCheckBox("扩展帧")
		send_layout.addWidget(self.chk_ext, 0, 2)

		send_layout.addWidget(QLabel("CAN端口:"), 1, 0)
		self.combo_port = QComboBox()
		self.combo_port.addItem("CAN0", 0)
		self.combo_port.addItem("CAN1", 1)
		self.combo_port.addItem("CAN2", 2)
		send_layout.addWidget(self.combo_port, 1, 1)

		send_layout.addWidget(QLabel("数据 (hex):"), 2, 0)
		self.edit_data = QLineEdit("FF FF 00 82")
		self.edit_data.setPlaceholderText("示例: 11 22 33 44 或 11223344")
		send_layout.addWidget(self.edit_data, 2, 1, 1, 2)

		self.chk_repeat = QCheckBox("重复发送")
		self.chk_repeat.toggled.connect(self.on_repeat_toggled)
		send_layout.addWidget(self.chk_repeat, 3, 0)

		self.edit_interval_ms = QLineEdit("1000")
		self.edit_interval_ms.setPlaceholderText("间隔 ms")
		send_layout.addWidget(self.edit_interval_ms, 3, 1)

		self.btn_send = QPushButton("发送")
		self.btn_send.clicked.connect(self.on_send_clicked)
		send_layout.addWidget(self.btn_send, 3, 2)

		root_layout.addWidget(send_group)

	def _start_rx_worker(self):
		self.rx_thread = QThread(self)
		self.rx_worker = UdpRxWorker(self.local_port)
		self.rx_worker.moveToThread(self.rx_thread)

		self.rx_thread.started.connect(self.rx_worker.run)
		self.rx_worker.log.connect(self.log)
		self.rx_worker.frame_rx.connect(self.on_frame_rx)
		self.rx_worker.parse_error.connect(self.on_parse_error)

		self.rx_thread.start()

	def closeEvent(self, event):
		self.rx_worker.stop()
		self.rx_thread.quit()
		self.rx_thread.wait(1500)
		super().closeEvent(event)

	def text_out_clear(self):
		self.text_out.clear()

	def update_send_button_text(self):
		if not self.chk_repeat.isChecked():
			self.btn_send.setText("发送")
		elif self.repeat_timer.isActive():
			self.btn_send.setText("正在发送")
		else:
			self.btn_send.setText("开始发送")

	def on_repeat_toggled(self, checked: bool):
		if not checked and self.repeat_timer.isActive():
			self.repeat_timer.stop()
			self.log("TX repeat stopped")
		self.update_send_button_text()

	def log(self, text: str):
		ts = datetime.now().strftime("%H:%M:%S")
		color = None
		if text.startswith("TX"):
			color = "blue"
		elif text.startswith("RX"):
			color = "darkgreen"

		line = f"[{ts}] {text}"
		if color is not None:
			self.text_out.append(f'<span style="color:{color};">{html.escape(line)}</span>')
		else:
			self.text_out.append(html.escape(line))

	def on_parse_error(self, reason: str, ip: str, port: int):
		self.log(f"RX invalid frame from {ip}:{port}, reason={reason}")

	def on_frame_rx(self, frame: dict, ip: str, port: int):
		frame_type = frame["type"]
		if (frame_type & 0xFE) != 0:
			type_name = f"RESERVED_BITS_SET(0x{frame_type:02X})"
		elif (frame_type & 0x01) == FORWARD_PROTO_TYPE_CAN_TO_UDP:
			type_name = "CAN_TO_UDP(bit0=0)"
		else:
			type_name = "UDP_TO_CAN(bit0=1)"
		self.log(f"Frame from {ip}:{port}, type={type_name}, msg_count={frame['msg_count']}")

		for msg in frame["messages"]:
			data_hex = " ".join(f"{b:02X}" for b in msg["data"])
			fmt = "EXT" if msg["ext"] else "STD"
			self.log(
				f"RX CAN port={msg['port']}, id=0x{msg['id']:X}, fmt={fmt}, len={msg['len']}, data=[{data_hex}]"
			)

	def build_current_frame(self):
		id_text = self.edit_id.text().strip()
		if id_text == "":
			raise ValueError("帧ID不能为空")

		can_id = int(id_text, 16)
		is_ext = self.chk_ext.isChecked()

		if is_ext:
			if can_id > 0x1FFFFFFF:
				raise ValueError("扩展帧ID超范围(0x1FFFFFFF)")
		else:
			if can_id > 0x7FF:
				raise ValueError("标准帧ID超范围(0x7FF)")

		data = parse_hex_data(self.edit_data.text().strip())
		can_port = int(self.combo_port.currentData())

		frame = build_frame(
			FORWARD_PROTO_TYPE_UDP_TO_CAN,
			[{"port": can_port, "id": can_id, "ext": is_ext, "data": data}],
		)
		return frame, can_id, is_ext, data, can_port

	def send_current_frame(self):
		frame, can_id, is_ext, data, can_port = self.build_current_frame()
		self.rx_worker.send_frame(frame, self.target_ip, self.target_port)

		data_hex = " ".join(f"{b:02X}" for b in data)
		self.log(
			f"TX to {self.target_ip}:{self.target_port}, port={can_port}, id=0x{can_id:X}, "
			f"fmt={'EXT' if is_ext else 'STD'}, len={len(data)}, data=[{data_hex}]"
		)

	def on_send_clicked(self):
		try:
			if self.chk_repeat.isChecked():
				interval_ms = int(self.edit_interval_ms.text().strip())
				if interval_ms <= 0:
					raise ValueError("重复发送间隔必须大于0ms")
				self.send_current_frame()
				self.repeat_timer.start(interval_ms)
				self.log(f"TX repeat started, interval={interval_ms}ms")
				self.update_send_button_text()
			else:
				if self.repeat_timer.isActive():
					self.repeat_timer.stop()
					self.update_send_button_text()
				self.send_current_frame()
		except ValueError as exc:
			QMessageBox.warning(self, "参数错误", str(exc))
		except OSError as exc:
			QMessageBox.critical(self, "发送失败", str(exc))


def main():
	parser = argparse.ArgumentParser(description="UDP2CAN debug tool for Ethernet2Can protocol")
	parser.add_argument("-l", "--local-port", dest="local_port", type=int, required=True,
		help="local UDP listen port")
	parser.add_argument("-r", "--remote-ip", dest="target_ip", type=str, required=True,
		help="target board IP")
	parser.add_argument("-p", "--remote-port", dest="target_port", type=int, required=True,
		help="target board UDP port")
	args = parser.parse_args()

	if args.local_port <= 0 or args.local_port > 65535:
		raise SystemExit("local_port must be in 1..65535")
	if args.target_port < 0 or args.target_port > 65535:
		raise SystemExit("target_port must be in 0..65535")

	app = QApplication(sys.argv)
	win = MainWindow(args.local_port, args.target_ip, args.target_port)
	win.show()
	sys.exit(app.exec_())


if __name__ == "__main__":
	main()