#!/usr/bin/env python3
"""
Flight Control System for MUGIN-3 3600MM H-Tail UAV
Core flight dynamics, PID controllers, and flight mode management
"""

import asyncio
import logging
import json
from dataclasses import dataclass, asdict
from enum import Enum
from typing import Optional
from datetime import datetime
import math

logger = logging.getLogger(__name__)


class FlightMode(Enum):
    """Flight modes"""
    DISARMED = 0
    STABILIZE = 1
    ALTITUDE_HOLD = 2
    GPS_GUIDED = 3
    RETURN_TO_HOME = 4
    AUTO_MISSION = 5
    MANUAL = 6


@dataclass
class SensorData:
    """Sensor data container"""
    timestamp: float
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    altitude: float
    pressure: float
    temperature: float
    gps_satellites: int


@dataclass
class Attitude:
    """Aircraft attitude (Euler angles)"""
    roll: float = 0.0  # radians
    pitch: float = 0.0  # radians
    yaw: float = 0.0  # radians
    roll_rate: float = 0.0  # rad/s
    pitch_rate: float = 0.0  # rad/s
    yaw_rate: float = 0.0  # rad/s


@dataclass
class ControlOutput:
    """Control surface outputs (0-1 normalized)"""
    elevator_left: float = 0.5
    elevator_right: float = 0.5
    aileron_left: float = 0.5
    aileron_right: float = 0.5
    rudder: float = 0.5
    throttle: float = 0.0


class PIDController:
    """PID controller implementation"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1, 
                 output_min: float = -1.0, output_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def update(self, error: float, current_time: float) -> float:
        """Update PID controller"""
        if self.last_time is None:
            dt = 0.02
        else:
            dt = current_time - self.last_time
        
        self.last_time = current_time
        
        # Proportional
        p = self.kp * error
        
        # Integral
        self.integral += error * dt
        self.integral = max(self.output_min, min(self.output_max, self.integral))
        i = self.ki * self.integral
        
        # Derivative
        if dt > 0:
            d = self.kd * (error - self.last_error) / dt
        else:
            d = 0
        
        self.last_error = error
        
        # Output
        output = p + i + d
        output = max(self.output_min, min(self.output_max, output))
        
        return output
    
    def reset(self):
        """Reset controller"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None


class ComplementaryFilter:
    """Complementary filter for attitude estimation"""
    
    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha  # Filter coefficient (0-1)
        self.attitude = Attitude()
        self.last_time = None
    
    def update(self, accel: tuple, gyro: tuple, mag: tuple = None, dt: float = 0.02) -> Attitude:
        """Update attitude estimate"""
        ax, ay, az = accel
        gx, gy, gz = gyro
        
        # Accelerometer-based attitude
        accel_roll = math.atan2(ay, math.sqrt(ax**2 + az**2))
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        # Gyro integration
        self.attitude.roll += gx * dt
        self.attitude.pitch += gy * dt
        self.attitude.yaw += gz * dt
        
        # Complementary filter
        self.attitude.roll = self.alpha * self.attitude.roll + (1 - self.alpha) * accel_roll
        self.attitude.pitch = self.alpha * self.attitude.pitch + (1 - self.alpha) * accel_pitch
        
        # Gyro rates
        self.attitude.roll_rate = gx
        self.attitude.pitch_rate = gy
        self.attitude.yaw_rate = gz
        
        return self.attitude


class FlightControlSystem:
    """Main flight control system"""
    
    def __init__(self, config_file: str = "flight_control_config.json"):
        self.config = self._load_config(config_file)
        
        # State
        self.armed = False
        self.mode = FlightMode.DISARMED
        self.attitude = Attitude()
        self.altitude = 0.0
        self.pressure = 101325.0
        self.temperature = 25.0
        self.gps_satellites = 0
        
        # Control targets
        self.target_attitude = Attitude()
        self.target_altitude = 0.0
        self.target_throttle = 0.0
        
        # Current control output
        self.current_control_output = ControlOutput()
        
        # Initialize controllers
        self._init_controllers()
        
        # Sensor fusion
        self.complementary_filter = ComplementaryFilter(alpha=0.98)
        self.last_sensor_data = None
        
        # Safety limits
        self.max_roll = math.radians(45)
        self.max_pitch = math.radians(30)
        self.max_altitude = 1000.0  # meters
        self.min_altitude = 0.0
        self.battery_voltage = 12.0
        self.battery_threshold = 10.5  # Volts
        
        logger.info("Flight Control System initialized")
    
    def _load_config(self, config_file: str) -> dict:
        """Load configuration"""
        try:
            with open(config_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            logger.warning(f"Config file not found: {config_file}, using defaults")
            return self._get_default_config()
    
    @staticmethod
    def _get_default_config() -> dict:
        """Get default configuration"""
        return {
            "pid": {
                "roll": {"kp": 4.5, "ki": 0.1, "kd": 0.3},
                "pitch": {"kp": 4.5, "ki": 0.1, "kd": 0.3},
                "yaw": {"kp": 4.5, "ki": 0.05, "kd": 0.15},
                "altitude": {"kp": 0.5, "ki": 0.01, "kd": 0.2}
            },
            "control": {
                "h_tail": True,
                "max_elevator": 0.25,
                "max_aileron": 0.25,
                "max_rudder": 0.25
            }
        }
    
    def _init_controllers(self):
        """Initialize PID controllers"""
        pid_config = self.config.get("pid", self._get_default_config()["pid"])
        
        self.roll_controller = PIDController(**pid_config["roll"])
        self.pitch_controller = PIDController(**pid_config["pitch"])
        self.yaw_controller = PIDController(**pid_config["yaw"])
        self.altitude_controller = PIDController(**pid_config["altitude"])
    
    def update_sensor_data(self, sensor_data: SensorData):
        """Update with sensor data"""
        self.last_sensor_data = sensor_data
        
        # Update altitude and environmental data
        self.altitude = sensor_data.altitude
        self.pressure = sensor_data.pressure
        self.temperature = sensor_data.temperature
        self.gps_satellites = sensor_data.gps_satellites
        
        # Update attitude via complementary filter
        accel = (sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z)
        gyro = (sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z)
        self.attitude = self.complementary_filter.update(accel, gyro)
    
    def arm(self):
        """Arm the aircraft"""
        if self.mode == FlightMode.DISARMED:
            self.armed = True
            self.mode = FlightMode.STABILIZE
            logger.info("Aircraft armed")
        else:
            logger.warning("Cannot arm: not in DISARMED mode")
    
    def disarm(self):
        """Disarm the aircraft"""
        self.armed = False
        self.mode = FlightMode.DISARMED
        self.current_control_output = ControlOutput()
        logger.info("Aircraft disarmed")
    
    def set_mode(self, mode: FlightMode):
        """Set flight mode"""
        if self.armed or mode == FlightMode.DISARMED:
            self.mode = mode
            logger.info(f"Flight mode changed to {mode.name}")
        else:
            logger.warning("Cannot change mode: aircraft not armed")
    
    def set_target_attitude(self, roll: float, pitch: float, yaw: float):
        """Set target attitude"""
        # Limit targets
        self.target_attitude.roll = max(-self.max_roll, min(self.max_roll, roll))
        self.target_attitude.pitch = max(-self.max_pitch, min(self.max_pitch, pitch))
        self.target_attitude.yaw = yaw % (2 * math.pi)
    
    def set_target_altitude(self, altitude: float):
        """Set target altitude"""
        self.target_altitude = max(self.min_altitude, min(self.max_altitude, altitude))
    
    def set_throttle(self, throttle: float):
        """Set throttle (0-1)"""
        self.target_throttle = max(0.0, min(1.0, throttle))
    
    def update_control(self, current_time: float):
        """Update control outputs"""
        if not self.armed:
            self.current_control_output = ControlOutput()
            return
        
        # Roll control (using ailerons)
        roll_error = self.target_attitude.roll - self.attitude.roll
        aileron_output = self.roll_controller.update(roll_error, current_time)
        
        # Pitch control (using elevators)
        pitch_error = self.target_attitude.pitch - self.attitude.pitch
        elevator_output = self.pitch_controller.update(pitch_error, current_time)
        
        # Yaw control (using rudder)
        yaw_error = self.target_attitude.yaw - self.attitude.yaw
        rudder_output = self.yaw_controller.update(yaw_error, current_time)
        
        # Altitude control (throttle)
        altitude_error = self.target_altitude - self.altitude
        throttle_output = self.target_throttle + self.altitude_controller.update(altitude_error, current_time)
        throttle_output = max(0.0, min(1.0, throttle_output))
        
        # Map to control surfaces (H-tail configuration)
        control_config = self.config.get("control", {})
        max_elevator = control_config.get("max_elevator", 0.25)
        max_aileron = control_config.get("max_aileron", 0.25)
        max_rudder = control_config.get("max_rudder", 0.25)
        
        # Normalize to servo range (0.0-1.0 where 0.5 is neutral)
        self.current_control_output.elevator_left = 0.5 + elevator_output * max_elevator
        self.current_control_output.elevator_right = 0.5 + elevator_output * max_elevator
        self.current_control_output.aileron_left = 0.5 + aileron_output * max_aileron
        self.current_control_output.aileron_right = 0.5 - aileron_output * max_aileron
        self.current_control_output.rudder = 0.5 + rudder_output * max_rudder
        self.current_control_output.throttle = throttle_output
        
        # Clamp all outputs
        for attr in ['elevator_left', 'elevator_right', 'aileron_left', 'aileron_right', 'rudder', 'throttle']:
            value = getattr(self.current_control_output, attr)
            setattr(self.current_control_output, attr, max(0.0, min(1.0, value)))
    
    def get_telemetry(self) -> dict:
        """Get telemetry data"""
        return {
            'timestamp': datetime.now().isoformat(),
            'armed': self.armed,
            'mode': self.mode.name,
            'attitude': asdict(self.attitude),
            'altitude': self.altitude,
            'pressure': self.pressure,
            'temperature': self.temperature,
            'gps_satellites': self.gps_satellites,
            'target': asdict(self.target_attitude),
            'target_altitude': self.target_altitude,
            'control_output': asdict(self.current_control_output),
            'battery_voltage': self.battery_voltage
        }


class FlightControlServer:
    """Command server for FCS"""
    
    def __init__(self, fcs: FlightControlSystem, host: str = "0.0.0.0", port: int = 5005):
        self.fcs = fcs
        self.host = host
        self.port = port
        self.server = None
        self.running = False
    
    async def handle_command(self, reader, writer):
        """Handle incoming command"""
        try:
            data = await reader.read(1024)
            command = json.loads(data.decode())
            
            response = await self._process_command(command)
            
            writer.write(json.dumps(response).encode())
            await writer.drain()
            writer.close()
        
        except Exception as e:
            logger.error(f"Command handler error: {e}")
    
    async def _process_command(self, command: dict) -> dict:
        """Process command"""
        cmd_type = command.get('type')
        
        if cmd_type == 'ARM':
            self.fcs.arm()
            return {'status': 'success', 'message': 'Aircraft armed'}
        
        elif cmd_type == 'DISARM':
            self.fcs.disarm()
            return {'status': 'success', 'message': 'Aircraft disarmed'}
        
        elif cmd_type == 'SET_MODE':
            mode_name = command.get('mode')
            try:
                mode = FlightMode[mode_name]
                self.fcs.set_mode(mode)
                return {'status': 'success', 'message': f'Mode set to {mode_name}'}
            except KeyError:
                return {'status': 'error', 'message': f'Unknown mode: {mode_name}'}
        
        elif cmd_type == 'SET_ATTITUDE':
            roll = command.get('roll', 0)
            pitch = command.get('pitch', 0)
            yaw = command.get('yaw', 0)
            self.fcs.set_target_attitude(roll, pitch, yaw)
            return {'status': 'success', 'message': 'Attitude target set'}
        
        elif cmd_type == 'SET_ALTITUDE':
            altitude = command.get('altitude', 0)
            self.fcs.set_target_altitude(altitude)
            return {'status': 'success', 'message': f'Altitude target set to {altitude}m'}
        
        elif cmd_type == 'SET_THROTTLE':
            throttle = command.get('throttle', 0)
            self.fcs.set_throttle(throttle)
            return {'status': 'success', 'message': f'Throttle set to {throttle}'}
        
        elif cmd_type == 'GET_STATUS':
            return self.fcs.get_telemetry()
        
        else:
            return {'status': 'error', 'message': f'Unknown command: {cmd_type}'}
    
    async def start(self):
        """Start command server"""
        logger.info(f"Starting FCS command server on {self.host}:{self.port}")
        self.server = await asyncio.start_server(self.handle_command, self.host, self.port)
        self.running = True
        logger.info("FCS command server started")
        
        async with self.server:
            await self.server.serve_forever()
    
    def stop(self):
        """Stop command server"""
        if self.server:
            self.server.close()
        self.running = False
        logger.info("FCS command server stopped")
