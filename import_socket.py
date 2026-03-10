import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Callable

# Константы протокола
CMD_PW_ON = b'\xA5\x30'
CMD_START = b'\xA5\x20'
CMD_STOP  = b'\xA5\x25'
M1_HEADER = 0x54
NODES_PER_PACK = 12
PACKET_FMT = '<BBHH' + 'HB' * NODES_PER_PACK + 'HHB'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# Таблица CRC8 из poelidar_protocol.h
CRC8_TABLE = bytes([
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
])

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc

@dataclass
class Point:
    angle: float      # градусы
    distance: float   # миллиметры
    intensity: float  # 0..255

class LidarM1:
    def __init__(self, host='192.168.0.7', port=25168):
        self.host = host
        self.port = port
        self.sock = None
        self._running = False
        self._thread = None
        self._current_scan = []          # буфер текущего оборота
        self._last_scan = []              # последний завершённый скан
        self._last_angle = None            # угол последней обработанной точки
        self.lock = threading.Lock()
        self.on_scan_complete: Optional[Callable[[List[Point]], None]] = None
        self.on_point_received: Optional[Callable[[Point], None]] = None

    def connect(self):
        """Устанавливает соединение, отправляет PW_ON и START."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        # Включение питания
        self.sock.send(CMD_PW_ON)
        time.sleep(2)   # согласно SDK
        # Запуск сканирования
        self.sock.send(CMD_START)
        time.sleep(1)

    def start(self):
        """Запускает фоновый поток чтения."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def stop(self):
        """Останавливает поток и закрывает соединение."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self.sock:
            try:
                self.sock.send(CMD_STOP)
            except:
                pass
            self.sock.close()
            self.sock = None

    def _reader(self):
        """Фоновый поток: читает пакеты и обрабатывает их."""
        while self._running:
            try:
                data = self._recv_all(PACKET_SIZE)
                if not data:
                    break
                self._process_packet(data)
            except Exception as e:
                print(f"Reader error: {e}")
                break

    def _recv_all(self, n):
        """Получает ровно n байт из сокета."""
        data = bytearray()
        while len(data) < n:
            chunk = self.sock.recv(n - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return data

    def _parse_packet(self, data):
        """Распаковывает и проверяет пакет. Возвращает None при ошибке."""
        if len(data) != PACKET_SIZE:
            return None
        # Проверка CRC
        if crc8(data[:-1]) != data[-1]:
            return None
        fields = struct.unpack(PACKET_FMT, data)
        header = fields[0]
        node_num = fields[1]
        if header != M1_HEADER or node_num != NODES_PER_PACK:
            return None
        start_angle = fields[3] * 0.01
        end_angle = fields[4 + 2*NODES_PER_PACK] * 0.01
        nodes = []
        for i in range(NODES_PER_PACK):
            dist = fields[4 + i*2]
            conf = fields[4 + i*2 + 1]
            nodes.append((dist, conf))
        return start_angle, end_angle, nodes

    def _process_packet(self, data):
        """Обрабатывает один пакет, разбирает точки и определяет завершение оборота."""
        parsed = self._parse_packet(data)
        if parsed is None:
            return
        start_angle, end_angle, nodes = parsed

        # Вычисляем шаг угла между точками в пакете
        if end_angle < start_angle:
            step = (end_angle + 360 - start_angle) / (NODES_PER_PACK - 1)
        else:
            step = (end_angle - start_angle) / (NODES_PER_PACK - 1)

        # Обрабатываем каждую точку пакета
        for i, (dist, conf) in enumerate(nodes):
            angle = (start_angle + step * i) % 360
            pt = Point(angle=angle, distance=float(dist), intensity=float(conf))

            with self.lock:
                # Если есть предыдущая точка и угол уменьшился (переход через 0°)
                if self._last_angle is not None and angle < self._last_angle - 300:
                    # Завершаем текущий оборот
                    if self._current_scan:
                        self._last_scan = self._current_scan.copy()
                        if self.on_scan_complete:
                            self.on_scan_complete(self._last_scan)
                    self._current_scan = []

                # Добавляем точку в текущий оборот
                self._current_scan.append(pt)
                self._last_angle = angle

                if self.on_point_received:
                    self.on_point_received(pt)

    def get_last_scan(self) -> List[Point]:
        """Возвращает последний завершённый скан (копию)."""
        with self.lock:
            return self._last_scan.copy()