import socket
import struct
import time
import numpy as np
from environment_map import EnvironmentMap

# Константы протокола (копируем из lidar_reader.py)
CMD_PW_ON = b'\xA5\x30'
CMD_START = b'\xA5\x20'
CMD_STOP  = b'\xA5\x25'
M1_HEADER = 0x54
NODES_PER_PACK = 12
PACKET_FMT = '<BBHH' + 'HB' * NODES_PER_PACK + 'HHB'

# Таблица CRC8 (копируем из lidar_reader.py)
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

def collect_scan_data(host='192.168.0.7', port=25168):
    """Собирает данные одного полного скана от лидара"""
    
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
    
    print("Collecting scan data...")
    
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
                    return scan_points
                if header_byte[0] == M1_HEADER:
                    break
            
            # Читаем остальные 46 байт
            remaining_data = bytearray()
            while len(remaining_data) < 46:
                chunk = sock.recv(46 - len(remaining_data))
                if not chunk:
                    return scan_points
                remaining_data.extend(chunk)
            
            full_packet = bytes([M1_HEADER]) + remaining_data
            
            # Проверяем CRC
            if crc8(full_packet[:-1]) != full_packet[-1]:
                print("CRC check failed, skipping packet")
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
                        print(f"Scan complete! Collected {len(scan_points)} points.")
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
                print(f"Packet parsing error: {e}")
                continue
                
    finally:
        # Остановка сканирования
        try:
            sock.send(CMD_STOP)
        except:
            pass
        sock.close()
    
    return scan_points

def main():
    """Основная функция для создания и отображения карты окружающей среды"""
    
    # Собираем данные скана
    scan_data = collect_scan_data()
    
    if not scan_data:
        print("No scan data collected!")
        return
    
    print(f"Collected {len(scan_data)} points from lidar")
    
    # Создаем карту окружающей среды
    env_map = EnvironmentMap(angular_resolution=1.0)  # 1° разрешение
    
    # Строим карту из скан-данных
    env_map.build_from_scan(scan_data)
    
    # Выводим информацию о карте
    env_map.print_map_info()
    
    # Сохраняем карту в файл
    env_map.save_to_file("background_environment.json")
    print("Background environment map saved!")
    
    # Показываем полярную диаграмму (если доступна)
    try:
        env_map.plot_polar("Background Environment Map")
    except ImportError:
        print("Matplotlib not available, skipping visualization")
    except Exception as e:
        print(f"Could not display polar plot: {e}")

if __name__ == "__main__":
    main()
