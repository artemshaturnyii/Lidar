import numpy as np
import json
import matplotlib.pyplot as plt

class EnvironmentMap:
    def __init__(self, angular_resolution=1.0):
        """
        Инициализация карты окружающей среды
        
        Args:
            angular_resolution (float): Разрешение карты в градусах (по умолчанию 1°)
        """
        self.angular_resolution = angular_resolution
        self.angles = np.arange(0, 360, angular_resolution)
        self.distances = np.full(len(self.angles), np.inf)  # Изначально все расстояния бесконечны
        self.is_built = False
        
    def build_from_scan(self, scan_points):
        """
        Построение карты из скан-данных
        
        Args:
            scan_points (list): Список точек [{'angle': float, 'distance': float, 'intensity': float}, ...]
        """
        print("Building environment map...")
        
        # Обнуляем карту
        self.distances = np.full(len(self.angles), np.inf)
        
        # Для каждого угла в карте собираем ближайшие измерения
        for i, target_angle in enumerate(self.angles):
            # Ищем все точки, которые попадают в сектор вокруг target_angle
            sector_half_width = self.angular_resolution / 2.0
            nearby_distances = []
            
            for point in scan_points:
                # Нормализуем разницу углов к [-180, 180]
                angle_diff = (point['angle'] - target_angle + 180) % 360 - 180
                if abs(angle_diff) <= sector_half_width:
                    nearby_distances.append(point['distance'])
            
            # Если есть измерения, берем минимальное расстояние (ближайший объект)
            if nearby_distances:
                self.distances[i] = min(nearby_distances)
        
        self.is_built = True
        print(f"Environment map built with {len(self.angles)} angular positions")
        
    def get_distance_at_angle(self, angle):
        """
        Получение расстояния до препятствия по заданному углу
        
        Args:
            angle (float): Угол в градусах (0-360)
            
        Returns:
            float: Расстояние до препятствия в мм, или np.inf если ничего не найдено
        """
        if not self.is_built:
            raise RuntimeError("Map is not built yet. Call build_from_scan() first.")
            
        # Нормализуем угол
        angle = angle % 360
        
        # Находим ближайший индекс
        index = int(round(angle / self.angular_resolution)) % len(self.angles)
        return self.distances[index]
    
    def detect_obstacle(self, angle, threshold_distance=500):
        """
        Проверка наличия препятствия на заданном угле
        
        Args:
            angle (float): Угол в градусах
            threshold_distance (float): Пороговое расстояние в мм
            
        Returns:
            bool: True если обнаружено препятствие ближе чем threshold_distance
        """
        distance = self.get_distance_at_angle(angle)
        return distance < threshold_distance
    
    def save_to_file(self, filename):
        """
        Сохранение карты в файл JSON
        
        Args:
            filename (str): Имя файла для сохранения
        """
        data = {
            'angular_resolution': self.angular_resolution,
            'angles': self.angles.tolist(),
            'distances': self.distances.tolist()
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f)
        print(f"Environment map saved to {filename}")
    
    def load_from_file(self, filename):
        """
        Загрузка карты из файла JSON
        
        Args:
            filename (str): Имя файла для загрузки
        """
        with open(filename, 'r') as f:
            data = json.load(f)
        
        self.angular_resolution = data['angular_resolution']
        self.angles = np.array(data['angles'])
        self.distances = np.array(data['distances'])
        self.is_built = True
        print(f"Environment map loaded from {filename}")
    
    def plot_polar(self, title="Environment Map"):
        """
        Построение полярной диаграммы карты
        """
        if not self.is_built:
            print("Map is not built yet!")
            return
            
        # Фильтруем бесконечные расстояния для отображения
        distances_to_plot = np.where(self.distances == np.inf, 0, self.distances)
        angles_rad = np.radians(self.angles)
        
        plt.figure(figsize=(10, 10))
        ax = plt.subplot(111, projection='polar')
        
        # Рисуем карту
        ax.plot(angles_rad, distances_to_plot, 'b-', linewidth=1, label='Environment')
        ax.fill(angles_rad, distances_to_plot, alpha=0.3, color='blue')
        
        # Настройки графика
        ax.set_theta_zero_location('N')  # 0° наверху
        ax.set_theta_direction(-1)       # По часовой стрелке
        ax.set_ylim(0, np.max(distances_to_plot) * 1.1 if np.max(distances_to_plot) > 0 else 2000)
        ax.set_title(title)
        ax.grid(True)
        
        plt.show()
    
    def print_map_info(self):
        """
        Вывод информации о карте
        """
        if not self.is_built:
            print("Map is not built yet!")
            return
            
        finite_distances = self.distances[self.distances != np.inf]
        
        print(f"\n=== Environment Map Info ===")
        print(f"Angular resolution: {self.angular_resolution}°")
        print(f"Total angles: {len(self.angles)}")
        print(f"Coverage angles: 0° - 360°")
        
        if len(finite_distances) > 0:
            print(f"Distance range: {np.min(finite_distances):.1f}mm - {np.max(finite_distances):.1f}mm")
            print(f"Average distance: {np.mean(finite_distances):.1f}mm")
        else:
            print("No obstacles detected")
        
        # Показываем несколько точек для примера
        print(f"\nSample points:")
        for i in range(0, min(20, len(self.angles)), 4):
            angle = self.angles[i]
            distance = self.distances[i]
            if distance == np.inf:
                print(f"  Angle {angle:6.1f}°: No obstacle (infinity)")
            else:
                print(f"  Angle {angle:6.1f}°: {distance:6.1f}mm")

def main():
    """
    Демонстрация работы с картой окружающей среды
    """
    # Создаем тестовую карту (в реальности будет создаваться из скан-данных)
    env_map = EnvironmentMap(angular_resolution=5.0)  # 5° разрешение для теста
    
    # Создаем искусственные данные для демонстрации
    test_points = []
    for angle in range(0, 360, 5):
        # Создаем "комнату" размером 2000мм с "столом" в центре
        if 45 <= angle <= 135:
            distance = 1500  # Стена справа
        elif 225 <= angle <= 315:
            distance = 1800  # Стена слева
        else:
            distance = 2000  # Дальние стены
            
        # Добавляем "стол" в диапазоне 0-45°
        if 10 <= angle <= 40:
            distance = min(distance, 800)  # Стол ближе
            
        test_points.append({
            'angle': float(angle),
            'distance': float(distance),
            'intensity': 100.0
        })
    
    # Строим карту
    env_map.build_from_scan(test_points)
    
    # Выводим информацию
    env_map.print_map_info()
    
    # Сохраняем карту
    env_map.save_to_file("environment_map.json")
    
    # Проверяем обнаружение препятствий
    print(f"\nObstacle detection tests:")
    print(f"At 25° (near table): {env_map.detect_obstacle(25, 1000)}")
    print(f"At 90° (wall): {env_map.detect_obstacle(90, 1000)}")
    print(f"At 180° (far wall): {env_map.detect_obstacle(180, 1000)}")
    
    # Показываем полярную диаграмму (если установлен matplotlib)
    try:
        env_map.plot_polar("Test Environment Map")
    except ImportError:
        print("Matplotlib not available, skipping polar plot")

if __name__ == "__main__":
    main()
