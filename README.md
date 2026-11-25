# EcoSmart Shuttle: energy-efficient autonomous robot

My BEng (Hons) Electronic Engineering final-year project at the University of Surrey (2024–25), supervised by Professor Zhili Sun.

It's a small autonomous shuttle robot that follows a line and stops for obstacles. I built it to compare how much power two control strategies, PID and fuzzy logic, use on real hardware, and whether ESP32 sleep modes save meaningful energy.

**Status:** built, run and measured on real hardware. All numbers below come from measurements on the physical robot.

## Hardware

| Part | Choice |
| --- | --- |
| Microcontroller | ESP32-S3 (Freenove ESP32-S3 DevKit C-1; ESP32-S3-WROOM CAM in the final build) |
| Line sensing | ITR9909 IR sensors, calibrated at 3.5 mm above the surface |
| Obstacle sensing | HC-SR04 ultrasonic sensors, validated to about 4 m |
| Camera | OV2640 onboard camera (replaced a Logitech C270 USB camera, which drew too much power) |
| Drive | DC motor through an L298N driver; A20 CLS steering servo (replaced an underpowered SG90) |
| Power | 7.4 V 2S LiPo (2200 mAh) with BMS and a 5 V buck converter |
| PCB | Custom board designed in KiCad and fabricated by JLCPCB (`CarPCB.zip`, revised in `NewCarPCB.zip`) |
| Chassis | 3D-printed PLA |

Early versions processed sensor data remotely over Wi-Fi (UDP). I moved all decision-making onto the ESP32-S3, which removed network latency and reduced power draw.

## Results

Each controller was run for repeated 5-minute trials at fixed load, measured in active, light-sleep and deep-sleep modes.

| Controller | Average active power |
| --- | --- |
| PID | 2664 mW |
| Fuzzy logic | 2794 mW |

- **PID averaged 4.6% less power than fuzzy logic** (about 130 mW), most likely because PID does less computation per control step.
- **What that 4.6% means:** the gap is smaller than the measurement uncertainty (about ±210 mW, half-range method), so PID used less power on average across these trials, but the trials do not prove it is always more efficient. A firm conclusion would need more runs or better power instrumentation. A 3σ estimate gave only ±0.3 mW, which ignored battery sag, sensor noise and transient spikes, so I used the more realistic half-range figure.
- **Sleep modes saved about 0.01% of total power** (around 0.3 mW against roughly 2.7 W in operation). At this scale, active-mode efficiency matters far more than sleep management.

## Repository contents

| File | What it is |
| --- | --- |
| `GitHubUpload2.zip` | Arduino sketches for the two controllers: `LocalPIDPWM` and `LocalFuzzyPWM` |
| `CarPCB.zip`, `NewCarPCB.zip` | KiCad PCB projects (first and revised board) |
| `PWRMeasure.zip`, `FinalPowerMeasurements.zip`, `PowerAverages.zip` | Power measurement code and raw data |
| `ErrorAnalysisPIDFuzzy.py`, `ErrorAnalysisSleepmodes.py` | Error and uncertainty analysis |
| `PWRAverageResults.txt`, `PowerErrorAnalysis.txt` | Summary results |
| `Power Calculations.docx` | Worked power calculations |
| `esp32s3_pinout.png` | ESP32-S3 pin mapping |

## Building the firmware

The sketches use the Arduino IDE with ESP32 board support. The steering servo and the DC motor (through the L298N) need the bundled ESP32Servo library: extract `ESP32Servo-master.zip` into your Arduino `libraries` folder before compiling.
