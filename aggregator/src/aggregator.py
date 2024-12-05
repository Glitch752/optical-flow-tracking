from abc import ABC, abstractmethod
import time
from typing import Literal, Optional, Type
from sensor_data_filter import FilteringMode, SensorData, SensorDataLocalizer
import serial
import serial.tools.list_ports
import serial.tools.list_ports_common

import fastplotlib as fpl

def main():
    # Find all serial ports with an Arduino connected
    ports = serial.tools.list_ports.comports()
    sensor_ports: list[tuple[type[Sensor], serial.tools.list_ports_common.ListPortInfo]] = []
    for port in ports:
        # Can be adjusted if we add other board types, I guess?
        if "Arduino" in port.description:
            sensor_ports.append((Sensor_NanoEvery_3901L0X, port))
    
    if len(sensor_ports) == 0:
        print("No connected sensors found.")
        return
    
    print(f"Found {len(sensor_ports)} connected sensors:")
    for port in sensor_ports:
        print(f"  - {port[0].name} on {port[1].device} [{port[1].description}]")
    
    # Initialize all sensors
    sensors = []
    for sensor_type, port in sensor_ports:
        sensors.append(sensor_type(port.device))
    
    localizer = SensorDataLocalizer(FilteringMode.AVERAGE)
    
    PLOT_INDIVIDUAL_VALUES = True
    PLOT_FILTERED_VALUES = True
    PLOT_POSITION = True
    
    PLOT_MOTION = True
    PLOT_DISTANCE = True
    PLOT_QUALITY = True
    
    fig = fpl.Figure(shape=(3, 2), size=(700, 560))
    
    motion_figure = fig[0, 0] if PLOT_MOTION else None
    distance_figure = fig[1, 0] if PLOT_DISTANCE else None
    quality_figure = fig[2, 0] if PLOT_QUALITY else None
    position_figure = fig[0, 1] if PLOT_POSITION else None
    
    if PLOT_MOTION:
        motion_figure.set_title("Motion")
    if PLOT_DISTANCE:
        distance_figure.set_title("Distance")
    if PLOT_QUALITY:
        quality_figure.set_title("Quality")
    if PLOT_POSITION:
        position_figure.set_title("Position")
        position_figure.set_grid_visibility(True)
    
    while True:
        loop_start = time.time()
        
        sample_list: list[SensorData] = []
        for sensor in sensors:
            sensor.update()
            sample_list.append(SensorData(
                time.time(),
                sensor.distance_mm, sensor.distance_quality,
                sensor.motion_x, sensor.motion_y, sensor.motion_quality
            ))
        localizer.add_sample_list(sample_list)
        
        localization_data = localizer.update_localization()
        
        draw_start = time.time()
        
        fig.clear()
        
        colors = ["#FF0000", "#00FF00", "#0000FF", "#FFFF00", "#00FFFF", "#FF00FF"] * 10
        current_colors = [0] * 3
        text_y_offsets = [1.5] * 3
        
        def plot_values(figure_type: Literal['motion', 'distance', 'quality'], values, label):
            # Kind of hacky, but needed to get typing working since fastplotlib doesn't export Subplot
            match figure_type:
                case 'motion':
                    figure = motion_figure
                    figure_idx = 0
                case 'distance':
                    figure = distance_figure
                    figure_idx = 1
                case 'quality':
                    figure = quality_figure
                    figure_idx = 2

            figure.add_line(values, colors=colors[current_colors[figure_idx]])
            current_colors[figure_idx] += 1
            figure.add_text(
                label,
                face_color=colors[current_colors[figure_idx]],
                anchor="top-right",
                font_size=20,
                offset=(1, text_y_offsets[figure_idx], 0)
            )
            text_y_offsets[figure_idx] += 1.5
        
        if PLOT_INDIVIDUAL_VALUES:
            for i, sensor in enumerate(sensors):
                sensor_samples = [s[i] for s in localizer.samples]
                samples_for = lambda attribute: [(s.timestamp, getattr(s, attribute)) for s in sensor_samples]
                if PLOT_MOTION:
                    plot_values("motion", samples_for("motion_x"), f"Sensor {i+1} - Motion X")
                    plot_values("motion", samples_for("motion_y"), f"Sensor {i+1} - Motion Y")
                    if PLOT_QUALITY:
                        plot_values("quality", samples_for("motion_quality"), f"Sensor {i+1} - Motion Quality")
                if PLOT_DISTANCE:
                    plot_values("distance", samples_for("distance_mm"), f"Sensor {i+1} - Distance (mm)")
                    if PLOT_QUALITY:
                        plot_values("quality", samples_for("distance_quality"), f"Sensor {i+1} - Distance Quality")
        
        if PLOT_FILTERED_VALUES:
            filtered_values = localization_data.filtered_values
            samples_for = lambda attribute: [(s.timestamp, getattr(s, attribute)) for s in filtered_values]
            if PLOT_MOTION:
                plot_values("motion", samples_for("motion_x"), "Filtered - Motion X")
                plot_values("motion", samples_for("motion_y"), "Filtered - Motion Y")
            if PLOT_DISTANCE:
                plot_values("distance", samples_for("distance_mm"), "Filtered - Distance (mm)")
        
        if PLOT_POSITION:
            position_samples = localization_data.position_samples
            position_figure.add_line(position_samples, cmap="viridis")
        
        try:
            fig.show()
    
            loop_time = time.time() - loop_start
            draw_time_ratio = (time.time() - draw_start) / loop_time
            fig.canvas.set_title(f"Sensor Data Aggregator - {1/loop_time:.2f} FPS (Draw time: {draw_time_ratio:.2%})")
        except Exception as e:
            break
    

class Sensor(ABC):
    name = "Generic Sensor"
    port: serial.Serial
    
    # This could be made more generic, but it's fine for now
    distance_mm: int = 0 # The maximum distance in-spec is 2000mm. Negative values indicate an error.
    distance_quality: int = 0 # 0-255
    
    motion_x: int = 0 # Some unit..?
    motion_y: int = 0 # Some unit..?
    motion_quality: int = 0 # 0-255
    
    def __init__(self, port: str, baudrate: int):
        self.port = serial.Serial(port, baudrate)
        if not self.port.is_open:
            self.port.open()
    
    @abstractmethod
    def update(self):
        pass
            

class Sensor_NanoEvery_3901L0X(Sensor):
    name = "Nano Every + 3901-L0X"

    def __init__(self, port: str):
        super().__init__(port, 115200)
    
    def update(self):
        update_types_seen = set()
        while self.port.in_waiting > 0:
            data = self.port.readline()
            if not data.startswith(b"### "):
                # This is a message we should forward to the user
                print(data)
                return
            data = data[4:].split(b" | ")
            
            update_type = data[0]
            
            if update_type in update_types_seen:
                print("WARNING: Sampling too slowly to accurately read sensor data.")
            
            update_types_seen.add(update_type)
            
            if update_type == b"OPTICAL_FLOW":
                self.motion_x, self.motion_y, self.motion_quality = map(lambda s: int(s.split(b" ")[1]), data[1:])
            elif update_type == b"DISTANCE":
                self.distance_mm, self.distance_quality = map(lambda s: int(s.split(b" ")[1]), data[1:])
            else:
                print(f"Unknown update type: {update_type}")