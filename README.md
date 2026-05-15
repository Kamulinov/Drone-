# MUGIN-3 3600MM H-Tail UAV Flight Control System

**Production-Ready Flight Control Software for MUGIN-3 Autonomous Aerial Vehicle**

## Overview

A complete, scalable flight control system designed specifically for the MUGIN-3 3600MM H-tail UAV. This system provides:

- **Real-time Flight Control**: 50 Hz servo control with PID stabilization
- **Sensor Fusion**: Complementary filter for accurate attitude estimation
- **H-Tail Support**: Proper dual elevator and dual aileron control
- **MAVLink Compatible**: Full compatibility with QGroundControl and MAVProxy
- **Production Deployment**: Docker containerization and systemd integration
- **Comprehensive Logging**: JSON and CSV flight data recording
- **Safety Systems**: Geofencing, failsafe modes, battery monitoring

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│         Flight Control Main (Async Event Loop)              │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────┐  ┌──────────────────┐  ┌────────────┐ │
│  │ Sensor Read     │  │ Control Update   │  │ Telemetry  │ │
│  │ Loop (100 Hz)   │  │ Loop (50 Hz)     │  │ Loop (10Hz)│ │
│  └────────┬���───────┘  └────────┬─────────┘  └────────┬───┘ │
│           │                    │                     │     │
│           └────────────────────┴─────────────────────┘     │
│                         │                                   │
├─────────────────────────┼───────────────────────────────────┤
│                         ▼                                   │
│             ┌──────────────────────────┐                   │
│             │ Flight Control System    │                   │
│             │  - PID Controllers       │                   │
│             │  - Attitude Estimation   │                   │
│             │  - Flight Mode Manager   │                   │
│             │  - Safety Limits         │                   │
│             └──────────────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
         ▲                                      ▼
    ┌────┴────────────────────────────────────┴───┐
    │                                              │
    ▼                                              ▼
┌──────────────────┐                      ┌───────────────────┐
│ Flight Controller│                      │ Telemetry Manager │
│     Board        │                      │   - MAVLink       │
│ - IMU (MPU-9250) │                      │   - Data Logging  │
│ - Barometer      │                      │   - Networking    │
│ - Compass        │                      └───────────────────┘
│ - GPS            │
│ - Servos (PWM)   │
└──────────────────┘
```

## Hardware Requirements

### Flight Controller
- **Pixhawk 4** or compatible autopilot
- **Raspberry Pi 4B** or **NVIDIA Jetson Nano** (onboard compute)

### Sensors
- **IMU**: MPU-9250 (9-axis: accel, gyro, compass)
- **Barometer**: BMP280 (altitude measurement)
- **Magnetometer**: HMC5883L (heading reference)
- **GPS**: u-blox M8N (GNSS positioning)

### Output Hardware
- **6-Channel PWM Servo Controller** for:
  - Elevator Left & Right (H-tail)
  - Aileron Left & Right
  - Rudder
  - Throttle (Motor ESC)

## Installation

### Option 1: Docker Deployment (Recommended)

```bash
# Clone repository
git clone https://github.com/Kamulinov/Drone-.git
cd Drone-

# Build and run with Docker Compose
docker-compose up -d

# View logs
docker-compose logs -f mugin3-fcs
```

### Option 2: Direct Installation

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install -y python3-pip i2c-tools supervisor

# Install Python dependencies
pip3 install -r requirements.txt

# Configure flight control system
sudo mkdir -p /etc/mugin3
sudo cp flight_control_config.json /etc/mugin3/

# Install supervisor config
sudo cp supervisor_fcs.conf /etc/supervisor/conf.d/
sudo supervisorctl reread
sudo supervisorctl update

# Start system
sudo supervisorctl start flight_control_system
```

## Configuration

Edit `flight_control_config.json` to customize:

```json
{
  "pid": {
    "roll": {"kp": 4.5, "ki": 0.1, "kd": 0.3},
    "pitch": {"kp": 4.5, "ki": 0.1, "kd": 0.3},
    "yaw": {"kp": 4.5, "ki": 0.05, "kd": 0.15},
    "altitude": {"kp": 0.5, "ki": 0.01, "kd": 0.2}
  },
  "safety": {
    "max_roll_angle": 45,
    "max_pitch_angle": 30,
    "max_altitude": 1000
  }
}
```

## Command Interface

### TCP Command Server (Port 5005)

Send JSON commands:

```python
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 5005))

# ARM
command = {'type': 'ARM'}
sock.send(json.dumps(command).encode())
response = json.loads(sock.recv(1024))

# SET MODE
command = {'type': 'SET_MODE', 'mode': 'STABILIZE'}
sock.send(json.dumps(command).encode())

# SET ALTITUDE
command = {'type': 'SET_ALTITUDE', 'altitude': 100.0}
sock.send(json.dumps(command).encode())

sock.close()
```

### Supported Commands

| Command | Parameters | Description |
|---------|------------|-------------|
| ARM | - | Arm aircraft |
| DISARM | - | Disarm aircraft |
| SET_MODE | mode | Set flight mode (STABILIZE, ALTITUDE_HOLD, etc.) |
| SET_ATTITUDE | roll, pitch, yaw | Set target attitude |
| SET_ALTITUDE | altitude | Set target altitude (meters) |
| SET_THROTTLE | throttle | Set throttle (0.0-1.0) |
| GET_STATUS | - | Get full telemetry |

## Telemetry

### MAVLink (Port 14550 UDP)

Compat with QGroundControl:

```bash
mavproxy.py --master=udpout:127.0.0.1:14550
```

### Data Logging

Flight data recorded in:
- **JSON**: `/var/log/mugin3/flight_YYYYMMDD_HHMMSS.json`
- **CSV**: `/var/log/mugin3/flight_YYYYMMDD_HHMMSS.csv`

### Telemetry Data

```json
{
  "timestamp": "2026-05-15T12:30:45.123456",
  "armed": true,
  "mode": "ALTITUDE_HOLD",
  "attitude": {
    "roll": 0.05,
    "pitch": -0.02,
    "yaw": 1.57,
    "roll_rate": 0.01,
    "pitch_rate": -0.005,
    "yaw_rate": 0.02
  },
  "altitude": 152.3,
  "pressure": 99847.5,
  "temperature": 22.5,
  "gps_satellites": 12,
  "control_output": {
    "elevator_left": 0.52,
    "elevator_right": 0.52,
    "aileron_left": 0.48,
    "aileron_right": 0.52,
    "rudder": 0.50,
    "throttle": 0.65
  }
}
```

## Flight Modes

- **DISARMED**: Safe state, no outputs
- **STABILIZE**: Manual control with automatic stabilization
- **ALTITUDE_HOLD**: Maintains altitude via throttle
- **GPS_GUIDED**: Autonomous waypoint navigation
- **RETURN_TO_HOME**: Auto-return to launch position
- **AUTO_MISSION**: Execute pre-programmed mission
- **MANUAL**: Direct servo control (no stabilization)

## Performance Specifications

- **Sensor Read Rate**: 100 Hz
- **Control Loop**: 50 Hz
- **Telemetry Update**: 10 Hz
- **Latency**: < 50 ms end-to-end
- **Attitude Estimation Accuracy**: ±2°
- **Altitude Accuracy**: ±2 meters
- **Processing Load**: ~45% on Raspberry Pi 4B

## Safety Features

✅ **Geofencing**: Circular boundary enforcement
✅ **Failsafe Actions**: Return-to-home, land, hover
✅ **Battery Monitoring**: Voltage warnings and cutoff
✅ **Attitude Limits**: Max roll/pitch angles
✅ **Altitude Limits**: Min/max altitude enforcement
✅ **Watchdog Timer**: Automatic disarm on signal loss
✅ **Graceful Shutdown**: Proper resource cleanup

## Troubleshooting

### I2C Sensor Not Detected

```bash
i2cdetect -y 1  # List I2C devices
sudo cat /var/log/mugin3/flight_control.log
```

### High CPU Usage

- Reduce sensor read rate in config
- Disable unnecessary telemetry streams
- Use hardware PWM instead of GPIO

### GPS No Fix

- Ensure clear sky view
- Check GPS baud rate (default: 38400)
- Verify u-blox configuration

### Servo Not Responding

- Check GPIO pin assignments in config
- Verify servo power supply (5V, 2A minimum)
- Test with dedicated PWM controller

## Development

### Running Tests

```bash
pytest tests/
pytest --cov=. tests/
```

### Code Quality

```bash
black *.py
pylint *.py
```

## License

Proprietary - MUGIN Flight Control Team

## Support

For issues and feature requests, contact the development team.

## Changelog

### v1.0.0 (2026-05-15)
- Initial production release
- Full H-tail support
- Docker deployment
- MAVLink integration
- Comprehensive logging
