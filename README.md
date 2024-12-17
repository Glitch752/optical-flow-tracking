# MSPv2 Optical Flow Odometry
A work-in-progress experiment to use the 3901-L0X Optical Flow/Lidar sensor for tracking a moving carrage's position.  
Currently primarily an adaptation of https://github.com/omrijsharon/mateksys_3901-L0X_esp32_reader to work with the Arduino Nano Every, but I plan to write a more robust system that supports multiple sensors with centralized data collection and filtering.  
As a temporary way to demonstrate the program, I made a simple dashboard web interface that communicates with the aggregator using WebSockets.