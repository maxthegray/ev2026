# ev2026

Science Olympiad Electric Vehicle, 2025–26. **3rd place, NY States.**

Drive a target distance in a target time and stop on the line. Two motors and a gyro, with no encoders. Speed comes from timing and feedforward, and the gyro keeps it straight.

## Control

- **Motion profile:** trapezoidal; cruise speed solved from distance, time, and max acceleration, with a velocity cap
- **Speed:** open-loop per-wheel feedforward (measured deadband + duty per m/s; left and right calibrated separately)
- **Heading:** gyro bias averaged over 200 samples at startup, yaw rate integrated, PD heading hold with clamped correction
- **Stop:** 80 ms reverse pulse at 30% duty, then cut
- **Battery:** motor output capped by battery voltage
- **State machine:** idle → running → braking → stopped, with LED status (red calibrating, blue ready, green running, yellow braking)

## Use

Set `TARGET_DISTANCE_M` and `TARGET_TIME_S` at the top of `ev2026.ino`, flash, and press go. Built on the `TektiteRotEv` library.
