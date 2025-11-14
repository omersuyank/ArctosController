import json
import sys
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - dependency guard
    raise SystemExit(
        "pyserial paketi gerekli. Lutfen 'pip install pyserial' calistirin."
    ) from exc

from PyQt6.QtCore import QObject, Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)


REFRESH_INTERVAL_MS = 1500
BAUD_RATE = 115200
TRIM_NEWLINES = "\r\n"
MOVE_COMPLETE_DELAY_MS = 100  # Her hareketten sonra bekleme süresi (ms)


@dataclass
class MotorMapping:
    board_index: int
    motor_id: int


@dataclass
class Waypoint:
    """Bir waypoint, 4 motorun pozisyonlarını ve hareket parametrelerini içerir"""
    name: str
    positions: Dict[int, int]  # motor_index -> position
    max_speed: float
    acceleration: float
    wait_time_ms: int = 500  # Bu waypoint'e ulaştıktan sonra bekleme süresi

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        return cls(**data)


def list_serial_ports() -> List[str]:
    return [port.device for port in list_ports.comports()]


class SerialReaderThread(QThread):
    line_received = pyqtSignal(str)

    def __init__(self, ser: serial.Serial):
        super().__init__()
        self._serial = ser
        self._running = True

    def run(self):
        while self._running and self._serial.is_open:
            try:
                line = self._serial.readline().decode(errors="ignore")
                if line:
                    self.line_received.emit(line.strip(TRIM_NEWLINES))
            except serial.SerialException:
                break

    def stop(self):
        self._running = False
        self.wait(200)


class ArduinoConnection(QObject):
    log_line = pyqtSignal(str)
    status_changed = pyqtSignal(bool)
    position_received = pyqtSignal(str, List[int])  # label, [pos1, pos2, pos3]

    def __init__(self, label: str):
        super().__init__()
        self._label = label
        self._serial: Optional[serial.Serial] = None
        self._reader: Optional[SerialReaderThread] = None
        self._pending_position_query = False

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self, port: str):
        if self.is_connected:
            self.log_line.emit(f"{self._label}: Zaten bagli.")
            return
        try:
            self._serial = serial.Serial(port, BAUD_RATE, timeout=0.1)
        except serial.SerialException as exc:
            self.log_line.emit(f"{self._label}: Baglanti hatasi -> {exc}")
            self._serial = None
            self.status_changed.emit(False)
            return

        self._reader = SerialReaderThread(self._serial)
        self._reader.line_received.connect(self._handle_received_line)
        self._reader.start()
        self.log_line.emit(f"{self._label}: {port} baglandi.")
        self.status_changed.emit(True)

    def _handle_received_line(self, text: str):
        self.log_line.emit(f"{self._label} << {text}")
        # POS? komutuna cevap parse et
        if text.startswith("POS "):
            try:
                parts = text.split()[1:]  # "POS 100 200 300" -> ["100", "200", "300"]
                positions = [int(p) for p in parts[:3]]  # İlk 3 pozisyon
                if self._pending_position_query:
                    self.position_received.emit(self._label, positions)
                    self._pending_position_query = False
            except (ValueError, IndexError):
                pass

    def disconnect(self):
        if not self.is_connected:
            return
        self.log_line.emit(f"{self._label}: Baglanti kesiliyor...")
        if self._reader:
            self._reader.stop()
        try:
            self._serial.close()
        except serial.SerialException:
            pass
        finally:
            self._serial = None
            self._reader = None
            self.status_changed.emit(False)
            self.log_line.emit(f"{self._label}: Baglanti kapandi.")

    def send(self, command: str):
        if not self.is_connected:
            self.log_line.emit(f"{self._label}: Bagli degil -> {command}")
            return
        payload = f"{command.strip()}\n".encode()
        try:
            self._serial.write(payload)
            self.log_line.emit(f"{self._label} >> {command}")
        except serial.SerialException as exc:
            self.log_line.emit(f"{self._label}: Gonderim hatasi -> {exc}")
            self.disconnect()

    def query_position(self):
        """POS? komutu gönder ve cevabı bekle"""
        if not self.is_connected:
            return
        self._pending_position_query = True
        self.send("POS?")


class MotorRow(QWidget):
    move_absolute = pyqtSignal(int, int, float, float)
    move_relative = pyqtSignal(int, int, float, float)
    zero_requested = pyqtSignal(int)
    enable_requested = pyqtSignal(int, bool)

    def __init__(self, motor_index: int, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.motor_index = motor_index
        layout = QGridLayout(self)

        layout.addWidget(QLabel(f"Motor {motor_index}"), 0, 0)

        self.abs_spin = QSpinBox()
        self.abs_spin.setRange(-500000, 500000)
        self.abs_spin.setSuffix(" adim")
        layout.addWidget(QLabel("Hedef"), 0, 1)
        layout.addWidget(self.abs_spin, 0, 2)

        self.rel_spin = QSpinBox()
        self.rel_spin.setRange(-500000, 500000)
        self.rel_spin.setSuffix(" adim")
        layout.addWidget(QLabel("Delta"), 0, 3)
        layout.addWidget(self.rel_spin, 0, 4)

        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(1, 5000)
        self.speed_spin.setValue(800)
        self.speed_spin.setSuffix(" adim/sn")
        layout.addWidget(QLabel("Max Hiz"), 1, 1)
        layout.addWidget(self.speed_spin, 1, 2)

        self.accel_spin = QDoubleSpinBox()
        self.accel_spin.setRange(1, 10000)
        self.accel_spin.setValue(300)
        self.accel_spin.setSuffix(" adim/sn²")
        layout.addWidget(QLabel("Ivme"), 1, 3)
        layout.addWidget(self.accel_spin, 1, 4)

        action_layout = QHBoxLayout()
        btn_abs = QPushButton("Mutlak Git")
        btn_rel = QPushButton("Goreceli Git")
        btn_zero = QPushButton("Sifirla")
        btn_enable = QPushButton("Enable")
        btn_disable = QPushButton("Disable")
        action_layout.addWidget(btn_abs)
        action_layout.addWidget(btn_rel)
        action_layout.addWidget(btn_zero)
        action_layout.addWidget(btn_enable)
        action_layout.addWidget(btn_disable)
        layout.addLayout(action_layout, 2, 0, 1, 5)

        btn_abs.clicked.connect(self._on_move_abs)
        btn_rel.clicked.connect(self._on_move_rel)
        btn_zero.clicked.connect(lambda: self.zero_requested.emit(self.motor_index))
        btn_enable.clicked.connect(lambda: self.enable_requested.emit(self.motor_index, True))
        btn_disable.clicked.connect(lambda: self.enable_requested.emit(self.motor_index, False))

    def _on_move_abs(self):
        self.move_absolute.emit(
            self.motor_index,
            self.abs_spin.value(),
            self.speed_spin.value(),
            self.accel_spin.value(),
        )

    def _on_move_rel(self):
        self.move_relative.emit(
            self.motor_index,
            self.rel_spin.value(),
            self.speed_spin.value(),
            self.accel_spin.value(),
        )


class ConnectionGroup(QGroupBox):
    port_changed = pyqtSignal(int, str)
    connect_clicked = pyqtSignal(int, str)
    disconnect_clicked = pyqtSignal(int)

    def __init__(self, board_index: int, label: str):
        super().__init__(label)
        self.board_index = board_index
        layout = QHBoxLayout(self)

        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(QLabel("Port"))
        layout.addWidget(self.port_combo)

        self.refresh_button = QPushButton("Yenile")
        layout.addWidget(self.refresh_button)

        self.connect_button = QPushButton("Baglan")
        layout.addWidget(self.connect_button)

        self.disconnect_button = QPushButton("Kes")
        layout.addWidget(self.disconnect_button)

        self.status_label = QLabel("Durum: Kapali")
        layout.addWidget(self.status_label)

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.port_combo.currentTextChanged.connect(
            lambda port: self.port_changed.emit(self.board_index, port)
        )
        self.connect_button.clicked.connect(
            lambda: self.connect_clicked.emit(self.board_index, self.selected_port())
        )
        self.disconnect_button.clicked.connect(
            lambda: self.disconnect_clicked.emit(self.board_index)
        )

    def refresh_ports(self):
        current = self.port_combo.currentText()
        ports = list_serial_ports()
        self.port_combo.clear()
        self.port_combo.addItems(ports or ["<Port Yok>"])
        if current in ports:
            idx = ports.index(current)
            self.port_combo.setCurrentIndex(idx)

    def selected_port(self) -> str:
        port = self.port_combo.currentText()
        return "" if port == "<Port Yok>" else port

    def set_status(self, connected: bool):
        self.status_label.setText(f"Durum: {'Bagli' if connected else 'Kapali'}")


class MainWindow(QMainWindow):
    MOTOR_MAP: Dict[int, MotorMapping] = {
        1: MotorMapping(board_index=0, motor_id=1),
        2: MotorMapping(board_index=0, motor_id=2),
        3: MotorMapping(board_index=1, motor_id=1),
        4: MotorMapping(board_index=1, motor_id=2),
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Arctos 4-Eksen Kontrol Paneli")
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QGridLayout(central)

        self.connections = [
            ArduinoConnection("UNO A"),
            ArduinoConnection("UNO B"),
        ]

        self.connection_groups: List[ConnectionGroup] = []
        for idx, label in enumerate(["UNO A", "UNO B"]):
            group = ConnectionGroup(idx, f"{label} Baglanti")
            group.port_changed.connect(self._remember_port)
            group.connect_clicked.connect(self._handle_connect)
            group.disconnect_clicked.connect(self._handle_disconnect)
            self.connection_groups.append(group)
            root_layout.addWidget(group, idx, 0)

        self._preferred_ports: Dict[int, str] = {}

        motor_box = QGroupBox("Motor Kontrolleri")
        motor_layout = QGridLayout(motor_box)
        self.motor_rows: Dict[int, MotorRow] = {}
        for i in range(1, 5):
            row = MotorRow(i)
            row.move_absolute.connect(self._move_absolute)
            row.move_relative.connect(self._move_relative)
            row.zero_requested.connect(self._zero_motor)
            row.enable_requested.connect(self._enable_motor)
            motor_layout.addWidget(row, i - 1, 0)
            self.motor_rows[i] = row

        root_layout.addWidget(motor_box, 0, 1, 2, 1)

        sequence_box = QGroupBox("Senaryo")
        seq_layout = QHBoxLayout(sequence_box)
        self.sequence_input = QLineEdit()
        self.sequence_input.setPlaceholderText("Komut satiri (ornegin: MOVR 1 200)")
        seq_layout.addWidget(self.sequence_input)
        send_seq = QPushButton("Komut Gonder")
        seq_layout.addWidget(send_seq)
        send_seq.clicked.connect(self._send_raw_command)
        root_layout.addWidget(sequence_box, 2, 0, 1, 2)

        # Senaryo Yönetimi Bölümü
        scenario_box = QGroupBox("Senaryo Yönetimi")
        scenario_layout = QVBoxLayout(scenario_box)

        # Waypoint kaydetme
        waypoint_layout = QHBoxLayout()
        self.waypoint_name_input = QLineEdit()
        self.waypoint_name_input.setPlaceholderText("Waypoint adi (ornegin: Baslangic, Bardak Al, Servis)")
        waypoint_layout.addWidget(QLabel("Waypoint Adi:"))
        waypoint_layout.addWidget(self.waypoint_name_input)
        btn_save_waypoint = QPushButton("Mevcut Pozisyonu Kaydet")
        btn_read_positions = QPushButton("Pozisyonlari Oku")
        waypoint_layout.addWidget(btn_read_positions)
        waypoint_layout.addWidget(btn_save_waypoint)
        scenario_layout.addLayout(waypoint_layout)

        # Waypoint listesi
        waypoint_list_layout = QHBoxLayout()
        self.waypoint_list = QListWidget()
        self.waypoint_list.setMaximumHeight(150)
        waypoint_list_layout.addWidget(QLabel("Waypoint'ler:"))
        waypoint_list_layout.addWidget(self.waypoint_list)
        btn_delete_waypoint = QPushButton("Sil")
        btn_clear_waypoints = QPushButton("Temizle")
        waypoint_list_layout.addWidget(btn_delete_waypoint)
        waypoint_list_layout.addWidget(btn_clear_waypoints)
        scenario_layout.addLayout(waypoint_list_layout)

        # Senaryo listesi
        scenario_list_layout = QHBoxLayout()
        self.scenario_list = QListWidget()
        self.scenario_list.setMaximumHeight(150)
        scenario_list_layout.addWidget(QLabel("Senaryo:"))
        scenario_list_layout.addWidget(self.scenario_list)
        btn_add_to_scenario = QPushButton("Senaryoya Ekle")
        btn_remove_from_scenario = QPushButton("Senaryodan Cikar")
        scenario_list_layout.addWidget(btn_add_to_scenario)
        scenario_list_layout.addWidget(btn_remove_from_scenario)
        scenario_layout.addLayout(scenario_list_layout)

        # Senaryo kontrolü
        scenario_control_layout = QHBoxLayout()
        self.loop_checkbox = QCheckBox("Dongu Modu (Sonsuz Tekrar)")
        scenario_control_layout.addWidget(self.loop_checkbox)
        btn_run_scenario = QPushButton("Senaryoyu Calistir")
        btn_stop_scenario = QPushButton("Durdur")
        btn_save_scenario = QPushButton("Senaryoyu Kaydet")
        btn_load_scenario = QPushButton("Senaryo Yukle")
        scenario_control_layout.addWidget(btn_run_scenario)
        scenario_control_layout.addWidget(btn_stop_scenario)
        scenario_control_layout.addWidget(btn_save_scenario)
        scenario_control_layout.addWidget(btn_load_scenario)
        scenario_layout.addLayout(scenario_control_layout)

        self.scenario_status_label = QLabel("Durum: Hazir")
        scenario_layout.addWidget(self.scenario_status_label)

        root_layout.addWidget(scenario_box, 3, 0, 1, 2)

        # Bağlantılar
        btn_save_waypoint.clicked.connect(self._save_current_waypoint)
        btn_read_positions.clicked.connect(self._read_all_positions)
        btn_delete_waypoint.clicked.connect(self._delete_selected_waypoint)
        btn_clear_waypoints.clicked.connect(self._clear_waypoints)
        btn_add_to_scenario.clicked.connect(self._add_waypoint_to_scenario)
        btn_remove_from_scenario.clicked.connect(self._remove_from_scenario)
        btn_run_scenario.clicked.connect(self._run_scenario)
        btn_stop_scenario.clicked.connect(self._stop_scenario)
        btn_save_scenario.clicked.connect(self._save_scenario_to_file)
        btn_load_scenario.clicked.connect(self._load_scenario_from_file)

        # Veri yapıları
        self.waypoints: Dict[str, Waypoint] = {}
        self.scenario: List[str] = []  # Waypoint isimlerinin sırası
        self._scenario_running = False
        self._scenario_loop = False
        self._current_positions: Dict[int, Dict[int, int]] = {}  # board_idx -> {motor_id -> position}

        log_box = QGroupBox("Log")
        log_layout = QHBoxLayout(log_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        root_layout.addWidget(log_box, 4, 0, 1, 2)

        for idx, conn in enumerate(self.connections):
            conn.log_line.connect(self._append_log)
            conn.status_changed.connect(
                lambda state, group_idx=idx: self.connection_groups[group_idx].set_status(state)
            )
            conn.position_received.connect(self._handle_position_response)

    def _append_log(self, text: str):
        self.log_view.appendPlainText(text)

    def _remember_port(self, board_idx: int, port: str):
        if port:
            self._preferred_ports[board_idx] = port

    def _handle_connect(self, board_idx: int, port: str):
        if not port:
            QMessageBox.warning(self, "Port Yok", "Lutfen gecerli bir port secin.")
            return
        self.connections[board_idx].connect(port)

    def _handle_disconnect(self, board_idx: int):
        self.connections[board_idx].disconnect()

    def _send_raw_command(self):
        text = self.sequence_input.text().strip()
        if not text:
            return
        board_idx = QMessageBox.question(
            self,
            "Hedef Sec",
            "Komutu UNO A'ya gondermek icin Yes, UNO B icin No'ya basin.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        target_idx = 0 if board_idx == QMessageBox.StandardButton.Yes else 1
        self.connections[target_idx].send(text)
        self.sequence_input.clear()

    def _motor_target(self, logical_motor: int) -> Optional[Tuple[ArduinoConnection, int]]:
        mapping = self.MOTOR_MAP.get(logical_motor)
        if not mapping:
            QMessageBox.warning(self, "Eslesme Yok", f"Motor {logical_motor} icin eslesme tanimsiz.")
            return None
        conn = self.connections[mapping.board_index]
        if not conn.is_connected:
            QMessageBox.warning(self, "Baglanti Yok", f"{logical_motor}. motor icin kart bagli degil.")
            return None
        return conn, mapping.motor_id

    def _move_absolute(self, logical_motor: int, position: int, max_speed: float, accel: float):
        target = self._motor_target(logical_motor)
        if not target:
            return
        conn, motor_id = target
        conn.send(f"SET {motor_id} {max_speed:.2f} {accel:.2f}")
        conn.send(f"MOVA {motor_id} {position}")

    def _move_relative(self, logical_motor: int, delta: int, max_speed: float, accel: float):
        target = self._motor_target(logical_motor)
        if not target:
            return
        conn, motor_id = target
        conn.send(f"SET {motor_id} {max_speed:.2f} {accel:.2f}")
        conn.send(f"MOVR {motor_id} {delta}")

    def _zero_motor(self, logical_motor: int):
        target = self._motor_target(logical_motor)
        if not target:
            return
        conn, motor_id = target
        conn.send(f"ZERO {motor_id}")

    def _enable_motor(self, logical_motor: int, state: bool):
        target = self._motor_target(logical_motor)
        if not target:
            return
        conn, motor_id = target
        conn.send(f"EN {motor_id} {1 if state else 0}")

    def _handle_position_response(self, label: str, positions: List[int]):
        """Arduino'dan gelen pozisyon cevabını işle"""
        board_idx = 0 if label == "UNO A" else 1
        if board_idx not in self._current_positions:
            self._current_positions[board_idx] = {}
        # UNO A: motor 1,2 -> positions[0], positions[1]
        # UNO B: motor 3,4 -> positions[0], positions[1] (ama logical 3,4)
        if board_idx == 0:
            self._current_positions[board_idx][1] = positions[0] if len(positions) > 0 else 0
            self._current_positions[board_idx][2] = positions[1] if len(positions) > 1 else 0
        else:
            self._current_positions[board_idx][1] = positions[0] if len(positions) > 0 else 0
            self._current_positions[board_idx][2] = positions[1] if len(positions) > 1 else 0

    def _read_all_positions(self):
        """Tüm Arduino'lardan pozisyon bilgisi oku"""
        for conn in self.connections:
            if conn.is_connected:
                conn.query_position()
        self._append_log("Pozisyonlar okunuyor...")

    def _save_current_waypoint(self):
        """Mevcut pozisyonları waypoint olarak kaydet"""
        name = self.waypoint_name_input.text().strip()
        if not name:
            QMessageBox.warning(self, "Ad Gerekli", "Lutfen waypoint icin bir ad girin.")
            return

        # Önce pozisyonları oku
        self._read_all_positions()
        # Closure için name'i yakala
        waypoint_name = name
        QTimer.singleShot(500, lambda: self._do_save_waypoint(waypoint_name))

    def _do_save_waypoint(self, name: str):
        """Pozisyonlar okunduktan sonra waypoint'i kaydet"""
        positions = {}
        for logical_motor in range(1, 5):
            mapping = self.MOTOR_MAP.get(logical_motor)
            if mapping:
                board_positions = self._current_positions.get(mapping.board_index, {})
                positions[logical_motor] = board_positions.get(mapping.motor_id, 0)

        # Motor satırlarından hız ve ivme al
        speed = self.motor_rows[1].speed_spin.value()
        accel = self.motor_rows[1].accel_spin.value()

        waypoint = Waypoint(
            name=name,
            positions=positions,
            max_speed=speed,
            acceleration=accel,
            wait_time_ms=500
        )

        self.waypoints[name] = waypoint
        self._update_waypoint_list()
        self.waypoint_name_input.clear()
        self._append_log(f"Waypoint kaydedildi: {name}")

    def _update_waypoint_list(self):
        """Waypoint listesini güncelle"""
        self.waypoint_list.clear()
        for name in sorted(self.waypoints.keys()):
            wp = self.waypoints[name]
            pos_str = ", ".join([f"M{m}:{p}" for m, p in sorted(wp.positions.items())])
            item_text = f"{name} ({pos_str})"
            self.waypoint_list.addItem(item_text)

    def _delete_selected_waypoint(self):
        """Seçili waypoint'i sil"""
        current = self.waypoint_list.currentRow()
        if current < 0:
            return
        item = self.waypoint_list.item(current)
        if not item:
            return
        # İsimi çıkar (ilk boşluğa kadar)
        name = item.text().split()[0]
        if name in self.waypoints:
            del self.waypoints[name]
            # Senaryodan da çıkar
            if name in self.scenario:
                self.scenario.remove(name)
            self._update_waypoint_list()
            self._update_scenario_list()
            self._append_log(f"Waypoint silindi: {name}")

    def _clear_waypoints(self):
        """Tüm waypoint'leri temizle"""
        reply = QMessageBox.question(
            self, "Temizle", "Tum waypoint'ler silinecek. Emin misiniz?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.waypoints.clear()
            self.scenario.clear()
            self._update_waypoint_list()
            self._update_scenario_list()

    def _add_waypoint_to_scenario(self):
        """Seçili waypoint'i senaryoya ekle"""
        current = self.waypoint_list.currentRow()
        if current < 0:
            QMessageBox.warning(self, "Secim Yok", "Lutfen bir waypoint secin.")
            return
        item = self.waypoint_list.item(current)
        if not item:
            return
        name = item.text().split()[0]
        if name not in self.waypoints:
            return
        self.scenario.append(name)
        self._update_scenario_list()

    def _remove_from_scenario(self):
        """Senaryodan seçili waypoint'i çıkar"""
        current = self.scenario_list.currentRow()
        if current >= 0 and current < len(self.scenario):
            removed = self.scenario.pop(current)
            self._update_scenario_list()
            self._append_log(f"Senaryodan cikarildi: {removed}")

    def _update_scenario_list(self):
        """Senaryo listesini güncelle"""
        self.scenario_list.clear()
        for idx, name in enumerate(self.scenario):
            self.scenario_list.addItem(f"{idx + 1}. {name}")

    def _run_scenario(self):
        """Senaryoyu çalıştır"""
        if not self.scenario:
            QMessageBox.warning(self, "Senaryo Yok", "Lutfen once bir senaryo olusturun.")
            return

        if self._scenario_running:
            QMessageBox.warning(self, "Zaten Calisiyor", "Senaryo zaten calisiyor.")
            return

        # Bağlantı kontrolü
        for conn in self.connections:
            if not conn.is_connected:
                QMessageBox.warning(self, "Baglanti Yok", "Tum Arduino'lar bagli olmalidir.")
                return

        self._scenario_running = True
        self._scenario_loop = self.loop_checkbox.isChecked()
        self.scenario_status_label.setText("Durum: Calisiyor...")
        self._append_log(f"Senaryo basladi (Dongu: {'Evet' if self._scenario_loop else 'Hayir'})")
        self._execute_scenario_step(0)

    def _execute_scenario_step(self, step_index: int):
        """Senaryonun bir adımını çalıştır"""
        if not self._scenario_running:
            return

        if step_index >= len(self.scenario):
            if self._scenario_loop:
                # Döngü modu: başa dön
                self._append_log("Senaryo tamamlandi, dongu modu aktif - tekrar basliyor...")
                QTimer.singleShot(1000, lambda: self._execute_scenario_step(0))
            else:
                # Tek seferlik: bitir
                self._scenario_running = False
                self.scenario_status_label.setText("Durum: Tamamlandi")
                self._append_log("Senaryo tamamlandi.")
            return

        waypoint_name = self.scenario[step_index]
        waypoint = self.waypoints.get(waypoint_name)
        if not waypoint:
            self._append_log(f"HATA: Waypoint bulunamadi: {waypoint_name}")
            self._scenario_running = False
            self.scenario_status_label.setText("Durum: Hata")
            return

        self._append_log(f"Adim {step_index + 1}/{len(self.scenario)}: {waypoint_name}")

        # Tüm motorları bu waypoint'e götür
        commands_by_board: Dict[int, List[str]] = {0: [], 1: []}

        for logical_motor, target_pos in waypoint.positions.items():
            mapping = self.MOTOR_MAP.get(logical_motor)
            if mapping:
                board_idx = mapping.board_index
                motor_id = mapping.motor_id
                # SET komutu
                commands_by_board[board_idx].append(
                    f"SET {motor_id} {waypoint.max_speed:.2f} {waypoint.acceleration:.2f}"
                )
                # MOVA komutu
                commands_by_board[board_idx].append(
                    f"MOVA {motor_id} {target_pos}"
                )

        # Komutları gönder
        for board_idx, commands in commands_by_board.items():
            for cmd in commands:
                self.connections[board_idx].send(cmd)

        # Hareketin tamamlanmasını bekle (basit yaklaşım: sabit süre)
        wait_time = waypoint.wait_time_ms + MOVE_COMPLETE_DELAY_MS
        next_step = step_index + 1
        QTimer.singleShot(wait_time, lambda: self._execute_scenario_step(next_step))

    def _stop_scenario(self):
        """Senaryoyu durdur"""
        if self._scenario_running:
            self._scenario_running = False
            self._scenario_loop = False
            self.scenario_status_label.setText("Durum: Durduruldu")
            self._append_log("Senaryo durduruldu.")

    def _save_scenario_to_file(self):
        """Senaryoyu JSON dosyasına kaydet"""
        if not self.scenario:
            QMessageBox.warning(self, "Senaryo Yok", "Kaydedilecek senaryo yok.")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, "Senaryo Kaydet", "", "JSON Dosyalari (*.json)"
        )
        if not filename:
            return

        data = {
            "waypoints": {name: wp.to_dict() for name, wp in self.waypoints.items()},
            "scenario": self.scenario
        }

        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self._append_log(f"Senaryo kaydedildi: {filename}")
            QMessageBox.information(self, "Basarili", f"Senaryo kaydedildi:\n{filename}")
        except Exception as e:
            QMessageBox.critical(self, "Hata", f"Kaydetme hatasi:\n{e}")

    def _load_scenario_from_file(self):
        """JSON dosyasından senaryo yükle"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Senaryo Yukle", "", "JSON Dosyalari (*.json)"
        )
        if not filename:
            return

        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # Waypoint'leri yükle
            self.waypoints.clear()
            for name, wp_data in data.get("waypoints", {}).items():
                self.waypoints[name] = Waypoint.from_dict(wp_data)

            # Senaryoyu yükle
            self.scenario = data.get("scenario", [])

            self._update_waypoint_list()
            self._update_scenario_list()
            self._append_log(f"Senaryo yuklendi: {filename}")
            QMessageBox.information(self, "Basarili", f"Senaryo yuklendi:\n{filename}")
        except Exception as e:
            QMessageBox.critical(self, "Hata", f"Yukleme hatasi:\n{e}")

    def closeEvent(self, event):
        self._stop_scenario()
        for conn in self.connections:
            conn.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(1200, 700)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

