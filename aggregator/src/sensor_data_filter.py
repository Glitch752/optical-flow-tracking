from enum import Enum, auto


class SensorData:
    timestamp: float
    distance_mm: int
    distance_quality: int
    motion_x: int
    motion_y: int
    motion_quality: int
    
    def __init__(self, timestamp, distance_mm, distance_quality, motion_x, motion_y, motion_quality):
        self.timestamp = timestamp
        self.distance_mm = distance_mm
        self.distance_quality = distance_quality
        self.motion_x = motion_x
        self.motion_y = motion_y
        self.motion_quality = motion_quality
    
    def __str__(self):
        return f"SensorData(timestamp={self.timestamp}, distance_mm={self.distance_mm}, distance_quality={self.distance_quality}, motion_x={self.motion_x}, motion_y={self.motion_y}, motion_quality={self.motion_quality})"

class LocalizationData:
    filtered_values: list[SensorData]
    position_samples: list[tuple[int, int]]
    position: tuple[int, int]
    
    def __init__(self, start_position: tuple[int, int]):
        self.position = start_position
        self.filtered_values = []
        self.position_samples = []

class FilteringMode(Enum):
    # TODO
    
    # EXTENDED_KALMAN_ON_ALL = auto()
    # EXTENDED_KALMAN_PER_SENSOR = auto()
    # EXTENDED_KALMAN_ON_AVERAGE = auto()
    
    # UNSCENTED_KALMAN_ON_ALL = auto()
    # UNSCENTED_KALMAN_PER_SENSOR = auto()
    # UNSCENTED_KALMAN_ON_AVERAGE = auto()
    
    AVERAGE = auto()

class SensorDataLocalizer:
    # The outer list contains sample snapshots of a list of sensors' data
    samples: list[list[SensorData]]
    sample_storage_size: int
    localization_data: LocalizationData
    
    filtering_mode: FilteringMode
    
    def __init__(self, filtering_mode: FilteringMode):
        self.sample_storage_size = 100
        self.filtering_mode = filtering_mode
        self.localization_data = LocalizationData((0, 0))
        self.samples = []

    def add_sample_list(self, sample_list: list[SensorData]):
        self.samples.append(sample_list)
        if len(self.samples) > self.sample_storage_size:
            self.samples.pop(0)
    
    def _add_filtered_values(self, filtered_values: SensorData):
        self.localization_data.filtered_values.append(filtered_values)
        if len(self.localization_data.filtered_values) > self.sample_storage_size:
            self.localization_data.filtered_values.pop(0)
    
    def _update_position(self):
        # TODO: Correction factors
        velocity_x = self.localization_data.filtered_values[-1].motion_x
        velocity_y = self.localization_data.filtered_values[-1].motion_y
        
        self.localization_data.position = (
            self.localization_data.position[0] + velocity_x,
            self.localization_data.position[1] + velocity_y
        )
        
        self.localization_data.position_samples.append(self.localization_data.position)
        if len(self.localization_data.position_samples) > self.sample_storage_size:
            self.localization_data.position_samples.pop(0)

    def _get_localization_data_average(self):
        QUALITY_THRESHOLD = 50
        
        distance_mm_sum = 0
        motion_x_sum = 0
        motion_y_sum = 0
        distance_count = 0
        motion_count = 0
        
        average_samples = 2
        
        for sample_idx in range(min(average_samples, len(self.samples))):
            sample = self.samples[-sample_idx]
            for sensor_data in sample:
                if sensor_data.distance_quality > QUALITY_THRESHOLD:
                    distance_mm_sum += sensor_data.distance_mm
                    distance_count += 1
                if sensor_data.motion_quality > QUALITY_THRESHOLD:
                    motion_x_sum += sensor_data.motion_x
                    motion_y_sum += sensor_data.motion_y
                    motion_count += 1
        
        distance_mm_avg = distance_mm_sum / distance_count if distance_count > 0 else 0
        motion_x_avg = motion_x_sum / motion_count if motion_count > 0 else 0
        motion_y_avg = motion_y_sum / motion_count if motion_count > 0 else 0
        
        self._add_filtered_values(SensorData(
            self.samples[-1][0].timestamp,
            distance_mm_avg, 100,
            motion_x_avg, motion_y_avg, 100
        ))

    def update_localization(self) -> LocalizationData:
        if self.filtering_mode == FilteringMode.AVERAGE:
            self._get_localization_data_average()
        else:
            raise NotImplementedError(f"Filtering mode {self.filtering_mode} not implemented")
        
        self._update_position()
        
        return self.localization_data