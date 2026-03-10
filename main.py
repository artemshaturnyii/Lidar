import time
import signal
import sys
import numpy as np
from import_socket import LidarM1
from calibrator import LidarCalibrator

# Попытка импорта matplotlib
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Matplotlib не установлен. График не будет показан. Установите: pip install matplotlib")

# Глобальная переменная для корректного завершения
calibrator = None
lidar = None

def signal_handler(sig, frame):
    print("\nПрерывание пользователя. Останавливаем лидар...")
    if lidar:
        lidar.stop()
    sys.exit(0)

def main():
    global lidar, calibrator

    # 1. Инициализация и подключение лидара
    lidar = LidarM1(host='192.168.0.7', port=25168)
    print("Подключение к лидару...")
    lidar.connect()
    lidar.start()
    print("Лидар запущен.")

    # 2. Создаём калибратор
    calibrator = LidarCalibrator(lidar)

    # 3. Запускаем калибровку (сбор 10 сканов)
    num_scans = 1
    print(f"Начинаем калибровку. Будет собрано {num_scans} полных сканов...")
    calibrator.start_calibration(num_scans=num_scans)

    # 4. Ожидаем завершения калибровки
    try:
        while calibrator.is_calibrating:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nКалибровка прервана пользователем.")
        lidar.stop()
        sys.exit(0)

    # 5. Калибровка завершена, выводим информацию
    print("\n=== Калибровка завершена ===")
    # Получаем углы и расстояния из карты
    angles = sorted(calibrator.background_map.keys())
    distances = [calibrator.background_map[a] for a in angles]
    print(f"Построена карта на {len(angles)} угловых интервалах.")
    print(f"Диапазон расстояний: {min(distances):.1f} – {max(distances):.1f} мм")

    # Пример: выведем значения для нескольких углов
    test_angles = [0, 45, 90, 135, 180, 225, 270, 315]
    print("\nФоновые расстояния для некоторых углов:")
    for a in test_angles:
        bg = calibrator.get_background_distance(a)
        print(f"  Угол {a:3}°: {bg:.1f} мм")

    # 6. Сохраняем карту в файл
    filename = "background.npz"
    calibrator.save_background(filename)
    print(f"\nКарта сохранена в файл '{filename}'.")

    # 7. Полярный график
    if HAS_MATPLOTLIB:
        plt.figure(figsize=(8,8))
        ax = plt.subplot(111, projection='polar')
        # Углы в радианах
        angles_rad = np.deg2rad(angles)
        ax.plot(angles_rad, distances, 'b-', linewidth=1)
        ax.set_title("Фоновая карта (полярное представление)")
        # Настройка: 0° — север, вращение по часовой стрелке (если лидар вращается так)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        plt.show()
    else:
        print("Чтобы увидеть полярный график, установите matplotlib: pip install matplotlib")

    # 8. Останавливаем лидар
    lidar.stop()
    print("Лидар остановлен.")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()