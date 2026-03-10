import numpy as np
from typing import List, Optional
from import_socket import LidarM1, Point

class LidarCalibrator:
    def __init__(self, lidar: LidarM1):
        self.lidar = lidar
        self.collected_scans = []  # список собранных сканов (каждый — список Point)
        self.background_map = None  # будет словарь {угол: расстояние} или интерполирующая функция
        self.is_calibrating = False
        self.target_scans = 0
        self.current_scan_count = 0

    def start_calibration(self, num_scans: int = 50):
        """Начинает сбор сканов для калибровки."""
        self.collected_scans = []
        self.target_scans = num_scans
        self.current_scan_count = 0
        self.is_calibrating = True
        # Подписываемся на получение полных сканов
        self.lidar.on_scan_complete = self._on_scan_calibration
        print(f"Калибровка: собираю {num_scans} сканов...")

    def _on_scan_calibration(self, scan: List[Point]):
        self.collected_scans.append(scan)
        self.current_scan_count += 1
        print(f"  Собран скан {self.current_scan_count}/{self.target_scans}")
        if self.current_scan_count >= self.target_scans:
            self.lidar.on_scan_complete = None          # отписываемся
            self._build_background_map()                # строим карту
            self.is_calibrating = False                 # теперь можно завершить ожидание
            print("Калибровка завершена. Фоновая карта построена.")
            
    def _build_background_map(self, step_deg: float = 1.0):
        """
        Строит фоновую карту, интерполируя все собранные сканы на равномерную сетку углов.
        Для каждого угла сетки берём среднее (или медиану) расстояний из всех сканов,
        используя ближайшие точки или интерполяцию.
        """
        if not self.collected_scans:
            return

        # Определяем сетку углов (0, step, 2*step, ..., 360-step)
        grid_angles = np.arange(0, 360, step_deg)
        grid_distances = np.full_like(grid_angles, np.nan, dtype=float)

        # Для каждого угла сетки соберём все измерения из всех сканов,
        # которые попадают в окрестность этого угла (например, ± step_deg/2)
        half_step = step_deg / 2.0
        for i, target_angle in enumerate(grid_angles):
            distances = []
            for scan in self.collected_scans:
                # Ищем точки, угол которых близок к target_angle
                # (учитываем переход через 0)
                for p in scan:
                    # Нормализуем разницу углов к [-180, 180]
                    diff = (p.angle - target_angle + 180) % 360 - 180
                    if abs(diff) <= half_step:
                        distances.append(p.distance)
            if distances:
                # Используем медиану для устойчивости к выбросам
                grid_distances[i] = np.median(distances)

        # Интерполируем пропущенные углы (если есть)
        # Заполняем линейной интерполяцией
        valid = ~np.isnan(grid_distances)
        if np.any(valid):
            grid_distances = np.interp(
                grid_angles,
                grid_angles[valid],
                grid_distances[valid],
                left=grid_distances[valid][0],
                right=grid_distances[valid][-1]
            )
        else:
            # Если ни одного измерения не нашлось — используем максимальное значение
            grid_distances.fill(np.nanmax([p.distance for scan in self.collected_scans for p in scan]))

        # Сохраняем карту как словарь для быстрого доступа по углу
        self.background_map = dict(zip(grid_angles, grid_distances))
        # Также можно сохранить как функцию интерполяции на будущее
        self._background_interp = lambda angle: np.interp(
            angle, grid_angles, grid_distances, period=360
        )

    def get_background_distance(self, angle: float) -> float:
        """Возвращает фоновое расстояние для заданного угла (в градусах)."""
        if self._background_interp is None:
            raise RuntimeError("Калибровка ещё не выполнена")
        return self._background_interp(angle)

    def save_background(self, filename: str):
        """Сохраняет фоновую карту в файл (например, numpy .npz)."""
        if self.background_map is None:
            return
        np.savez(filename, angles=list(self.background_map.keys()), distances=list(self.background_map.values()))

    def load_background(self, filename: str):
        """Загружает фоновую карту из файла."""
        data = np.load(filename)
        angles = data['angles']
        distances = data['distances']
        self.background_map = dict(zip(angles, distances))
        self._background_interp = lambda angle: np.interp(angle, angles, distances, period=360)