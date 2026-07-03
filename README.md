# Custom-Flight-Controller
A custom STM32F446RE-based Flight Controller System for UAVs featuring IMU, GPS, Barometer, Magnetometer, Sensor Fusion, PID Control, PWM Motor Control, and real-time telemetry. Designed as a modular embedded system for autonomous flight research and educational applications.

**Features**

- STM32F446RE based controller
- IMU (BNO085)
- GPS (NEO-M8N)
- Barometer (MS5611)
- Magnetometer (HMC5883L)
- UART Communication
- SPI Communication
- PWM Motor Control
- Sensor Fusion
- PID Controller
- Real-time Telemetry

  **Hardware Used**

- STM32F446RE
- BNO085 IMU
- MS5611 Barometer
- HMC5883L Magnetometer
- NEO-M8N GPS
- ESC
- Brushless Motors

 **Working**

1. Initialize STM32 peripherals.
2. Read data from sensors.
3. Apply filtering.
4. Calculate orientation.
5. Run PID controller.
6. Generate PWM signals.
7. Send telemetry.

   **Authors**

-  R. Saikumar
-  K. Saathvik Vardhan
-  P. Pranay Kumar
