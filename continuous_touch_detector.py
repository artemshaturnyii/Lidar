import json
import time
import socket
import struct
from datetime import datetime

# Константы протокола
CMD_PW_ON = b'\xA5\x30'
CMD_START = b'\xA5\x20'
CMD_STOP  = b'\xA5\x25'
M1_HEADER = 0x54
NODES_PER_PACK = 12
PACKET_FMT = '<BBHH' + 'HB' * NODES_PER_PACK + 'HHB'

# Полная таблица CRC8
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
    """Вычисление CRC8 для данных"""
    crc = 0
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc

class TouchDetector:
    def __init__(self, background_map_file="background_environment.json"):
        """Инициализация детектора касаний"""
        self.load_background_map(background_map_file)
        self.touch_sensitivity = 200   # мм - чувствительность касания
        self.min_touch_distance = 50   # мм - минимальное расстояние
        self.max_touch_distance = 10000 # мм - максимальное расстояние
        self.detection_cooldown = 1.0  # секунды - антидребезг
        self.last_detections = {}      # история срабатываний
        
    def load_background_map(self, filename):
        """Загрузка фоновой карты"""
        with open(filename, 'r') as f:
            data = json.load(f)
        self.background_angles = data['angles']
        self.background_distances = data['distances']
        self.angular_resolution = data['angular_resolution']
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Background map loaded")
        
        # Подсчитываем статистику
        finite_distances = [d for d in self.background_distances if d != float('inf')]
        if finite_distances:
            avg_background = sum(finite_distances) / len(finite_distances)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Average background distance: {avg_background:.1f}mm")
        
    def get_background_distance(self, angle):
        """Получение фонового расстояния для заданного угла"""
        angle = angle % 360
        index = round(angle / self.angular_resolution)
        index = index % len(self.background_angles)
        return self.background_distances[index]
    
    def is_touch_detected(self, angle, current_distance):
        """
        Определяет, является ли текущее измерение касанием
        """
        background_distance = self.get_background_distance(angle)
        
        # Проверяем только точки в допустимом диапазоне
        if not (self.min_touch_distance <= current_distance <= self.max_touch_distance):
            return False, "out_of_range"
            
        # Если в фоне было расстояние, а текущее значительно меньше
        if background_distance != float('inf'):
            difference = background_distance - current_distance
            
            # Если разница больше чувствительности, это касание
            if difference > self.touch_sensitivity:
                return True, f"approached_by_{difference:.1f}mm"
                
        return False, "no_significant_change"
    
    def detect_touches(self, current_scan):
        """
        Обнаружение касаний в текущем скане
        """
        touches = []
        
        for point in current_scan:
            angle = point['angle']
            distance = point['distance']
            
            is_touch, reason = self.is_touch_detected(angle, distance)
            
            if is_touch:
                background_dist = self.get_background_distance(angle)
                touches.append({
                    'angle': angle,
                    'distance': distance,
                    'background_distance': background_dist,
                    'difference': background_dist - distance,
                    'reason': reason
                })
                
        return touches
    
    def filter_recent_touches(self, touches):
        """
        Фильтрация повторных срабатываний по антидребезгу
        """
        filtered_touches = []
        current_time = time.time()
        
        for touch in touches:
            angle_key = round(touch['angle'])
            
            # Проверяем cooldown
            if (angle_key not in self.last_detections or 
                current_time - self.last_detections[angle_key] > self.detection_cooldown):
                
                filtered_touches.append(touch)
                self.last_detections[angle_key] = current_time
                
        return filtered_touches
    
    def print_touches(self, touches):
        """Вывод информации о касаниях"""
        if not touches:
            return
            
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Touches detected: {len(touches)}")
        for touch in touches:
            print(f"  Angle {touch['angle']:.1f}°: {touch['distance']:.1f}mm "
                  f"(background {touch['background_distance']:.1f}mm, "
                  f"diff {touch['difference']:.1f}mm)")

class LidarConnection:
    def __init__(self, host='192.168.0.7', port=25168):
        self.host = host
        self.port = port
        self.sock = None
        
    def connect(self):
        """Подключение к лидару"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.sock.send(CMD_PW_ON)
            time.sleep(2)
            self.sock.send(CMD_START)
            time.sleep(1)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Connected to lidar")
            return True
        except Exception as e:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Connection failed: {e}")
            return False
    
    def collect_scan(self):
        """Сбор одного скана"""
        if not self.sock:
            return None
            
        scan_points = []
        packet_count = 0
        max_packets = 50
        
        try:
            while packet_count < max_packets:
                # Поиск заголовка
                header_byte = None
                attempts = 0
                max_attempts = 100
                
                while header_byte != M1_HEADER and attempts < max_attempts:
                    try:
                        byte_data = self.sock.recv(1)
                        if not byte_data:
                            return None
                        header_byte = byte_data[0]
                        attempts += 1
                    except:
                        return None
                
                if header_byte != M1_HEADER:
                    break
                
                # Чтение остальных данных
                remaining_data = bytearray()
                bytes_needed = 46
                while len(remaining_data) < bytes_needed and bytes_needed > 0:
                    try:
                        chunk = self.sock.recv(bytes_needed - len(remaining_data))
                        if not chunk:
                            break
                        remaining_data.extend(chunk)
                    except:
                        return None
                
                if len(remaining_data) < 46:
                    continue
                
                full_packet = bytes([M1_HEADER]) + remaining_data
                
                # Проверка CRC
                if len(full_packet) >= 2 and crc8(full_packet[:-1]) == full_packet[-1]:
                    # Парсинг пакета
                    try:
                        fields = struct.unpack(PACKET_FMT, full_packet)
                        start_angle_raw = fields[3]
                        start_angle = start_angle_raw * 0.01
                        
                        # Извлечение точек
                        nodes = []
                        for i in range(NODES_PER_PACK):
                            distance = fields[4 + i*2]
                            intensity = fields[4 + i*2 + 1]
                            nodes.append((distance, intensity))
                        
                        end_angle_raw = fields[4 + 2*NODES_PER_PACK]
                        end_angle = end_angle_raw * 0.01
                        
                        # Вычисление шага
                        if end_angle < start_angle:
                            step = (end_angle + 360 - start_angle) / (len(nodes) - 1)
                        else:
                            step = (end_angle - start_angle) / (len(nodes) - 1)
                        
                        # Добавление точек
                        for i, (distance, intensity) in enumerate(nodes):
                            angle = (start_angle + step * i) % 360
                            scan_points.append({
                                'angle': float(angle),
                                'distance': float(distance),
                                'intensity': float(intensity)
                            })
                        
                        packet_count += 1
                        
                    except struct.error:
                        continue
                    except Exception:
                        continue
                        
        except Exception:
            pass
            
        return scan_points if scan_points else None
    
    def disconnect(self):
        """Отключение от лидара"""
        if self.sock:
            try:
                self.sock.send(CMD_STOP)
            except:
                pass
            self.sock.close()
            self.sock = None
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Disconnected from lidar")

def main():
    """Основной цикл обнаружения касаний"""
    
    print("=== Touch Detection System ===")
    print("Detecting approaches to background objects")
    print("Press Ctrl+C to stop")
    
    detector = TouchDetector("background_environment.json")
    lidar = LidarConnection()
    
    if not lidar.connect():
        return
    
    scan_count = 0
    try:
        while True:
            scan_data = lidar.collect_scan()
            if not scan_data:
                time.sleep(0.1)
                continue
            
            scan_count += 1
            
            # Обнаружение касаний
            touches = detector.detect_touches(scan_data)
            filtered_touches = detector.filter_recent_touches(touches)
            
            # Вывод касаний
            if filtered_touches:
                detector.print_touches(filtered_touches)
            elif scan_count % 100 == 0:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] Scan {scan_count} processed")
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Stopping...")
    finally:
        lidar.disconnect()

if __name__ == "__main__":
    main()
