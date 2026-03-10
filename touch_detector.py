import json
import time
import socket
import struct
from collections import deque

# Константы протокола (те же, что и раньше)
CMD_PW_ON = b'\xA5\x30'
CMD_START = b'\xA5\x20'
CMD_STOP  = b'\xA5\x25'
M1_HEADER = 0x54
NODES_PER_PACK = 12
PACKET_FMT = '<BBHH' + 'HB' * NODES_PER_PACK + 'HHB'

CRC8_TABLE = [
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
]

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc

class TouchDetector:
    def __init__(self, background_map_file="background_environment.json"):
        """Инициализация детектора касаний"""
        self.load_background_map(background_map_file)
        self.touch_threshold = 100  # мм - порог обнаружения касания
        self.min_touch_distance = 50  # мм - минимальное расстояние для касания
        
    def load_background_map(self, filename):
        """Загрузка фоновой карты"""
        with open(filename, 'r') as f:
            data = json.load(f)
        self.background_angles = data['angles']
        self.background_distances = data['distances']
        self.angular_resolution = data['angular_resolution']
        print(f"Background map loaded: {len(self.background_angles)} angles")
        
    def detect_touches(self, current_scan):
        """
        Обнаружение касаний путем сравнения текущего скана с фоновой картой
        
        Args:
            current_scan (list): Текущие данные скана
            
        Returns:
            list: Список обнаруженных касаний [{'angle': float, 'distance': float, 'difference': float}, ...]
        """
        touches = []
        
        # Для каждой точки текущего скана проверяем отклонение от фона
        for point in current_scan:
            angle = point['angle']
            current_distance = point['distance']
            
            # Находим ближайший угол в фоновой карте
            background_distance = self.get_background_distance(angle)
            
            # Если фоновое расстояние известно и текущее расстояние значительно меньше
            if background_distance != float('inf') and current_distance < background_distance:
                difference = background_distance - current_distance
                
                # Если разница больше порога, это касание
                if difference > self.touch_threshold and current_distance > self.min_touch_distance:
                    touches.append({
                        'angle': angle,
                        'distance': current_distance,
                        'background_distance': background_distance,
                        'difference': difference
                    })
                    
        return touches
    
    def get_background_distance(self, angle):
        """Получение фонового расстояния для заданного угла"""
        # Нормализуем угол
        angle = angle % 360
        
        # Находим ближайший индекс
        index = round(angle / self.angular_resolution)
        index = index % len(self.background_angles)
        return self.background_distances[index]
    
    def print_touches(self, touches):
        """Вывод информации об обнаруженных касаниях"""
        if not touches:
            print("No touches detected")
            return
            
        print(f"\n=== Detected {len(touches)} touch(es) ===")
        for i, touch in enumerate(touches):
            print(f"Touch {i+1}: angle={touch['angle']:.1f}°, "
                  f"distance={touch['distance']:.1f}mm, "
                  f"background={touch['background_distance']:.1f}mm, "
                  f"difference={touch['difference']:.1f}mm")

def collect_single_scan(host='192.168.0.7', port=25168):
    """Сбор одного скана от лидара"""
    print(f"Connecting to lidar {host}:{port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    
    # Включение питания
    print("Power on...")
    sock.send(CMD_PW_ON)
    time.sleep(2)
    
    # Запуск сканирования
    print("Start scanning...")
    sock.send(CMD_START)
    time.sleep(1)
    
    print("Collecting single scan...")
    
    # Собираем все точки
    scan_points = []
    last_angle = None
    scan_complete = False
    
    try:
        while not scan_complete:
            # Ищем заголовок 0x54
            while True:
                header_byte = sock.recv(1)
                if not header_byte:
                    break
                if header_byte[0] == M1_HEADER:
                    break
            
            # Читаем остальные 46 байт
            remaining_data = bytearray()
            while len(remaining_data) < 46:
                chunk = sock.recv(46 - len(remaining_data))
                if not chunk:
                    break
                remaining_data.extend(chunk)
            
            if len(remaining_data) < 46:
                break
                
            full_packet = bytes([M1_HEADER]) + remaining_data
            
            # Проверяем CRC
            if crc8(full_packet[:-1]) != full_packet[-1]:
                continue
                
            # Распаковываем пакет
            try:
                fields = struct.unpack(PACKET_FMT, full_packet)
                start_angle_raw = fields[3]
                start_angle = start_angle_raw * 0.01
                
                # Извлекаем узлы (точки)
                nodes = []
                for i in range(NODES_PER_PACK):
                    distance = fields[4 + i*2]
                    intensity = fields[4 + i*2 + 1]
                    nodes.append((distance, intensity))
                
                # Конечный угол
                end_angle_raw = fields[4 + 2*NODES_PER_PACK]
                end_angle = end_angle_raw * 0.01
                
                # Вычисляем шаг угла между точками
                if end_angle < start_angle:
                    step = (end_angle + 360 - start_angle) / (len(nodes) - 1)
                else:
                    step = (end_angle - start_angle) / (len(nodes) - 1)
                
                # Обрабатываем каждую точку
                for i, (distance, intensity) in enumerate(nodes):
                    angle = (start_angle + step * i) % 360
                    
                    # Проверяем переход через 0°
                    if last_angle is not None and angle < last_angle - 300:
                        scan_complete = True
                        break
                    
                    scan_points.append({
                        'angle': angle,
                        'distance': float(distance),
                        'intensity': float(intensity)
                    })
                    last_angle = angle
                    
                if scan_complete:
                    break
                    
            except Exception as e:
                continue
                
    finally:
        # Остановка сканирования
        try:
            sock.send(CMD_STOP)
        except:
            pass
        sock.close()
    
    print(f"Scan complete! Collected {len(scan_points)} points.")
    return scan_points

def main():
    """Основная функция для тестирования обнаружения касаний"""
    
    # Создаем детектор касаний
    detector = TouchDetector("background_environment.json")
    
    # Собираем текущий скан
    current_scan = collect_single_scan()
    
    # Обнаруживаем касания
    touches = detector.detect_touches(current_scan)
    
    # Выводим результаты
    detector.print_touches(touches)
    
    # Если есть касания, выводим дополнительную информацию
    if touches:
        angles = [t['angle'] for t in touches]
        distances = [t['distance'] for t in touches]
        print(f"\nTouch statistics:")
        print(f"  Angle range: {min(angles):.1f}° - {max(angles):.1f}°")
        print(f"  Distance range: {min(distances):.1f}mm - {max(distances):.1f}mm")

if __name__ == "__main__":
    main()
