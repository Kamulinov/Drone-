#!/usr/bin/env python3
"""
Hardware Interface for MUGIN-3 Flight Controller
Handles I2C sensors, PWM servos, GPS, and analog inputs
"""

import logging
import struct
from typing import Dict, Optional
from dataclasses import dataclass
import time

logger = logging.getLogger(__name__)


@dataclass
class SensorReading:
    """Sensor reading container"""
    accel: tuple  # (x, y, z) in m/s^2
    gyro: tuple   # (x, y, z) in rad/s
    mag: tuple    # (x, y, z) in µT
    altitude: float  # meters
    pressure: float  # Pa
    temperature: float  # Celsius
    gps: tuple  # (lat, lon, alt, num_sats)


class I2CSensor:
    """Base class for I2C sensors"""
    
    def __init__(self, address: int, bus: int = 1):
        self.address = address
        self.bus = bus
        self.device = None
    
    def initialize(self) -> bool:
        """Initialize I2C device"""
        try:
            import smbus2
            self.device = smbus2.SMBus(self.bus)
            logger.info(f"I2C sensor initialized at address 0x{self.address:02x}")
            return True
        except ImportError:
            logger.warning("smbus2 not available, using simulation mode")
            return False
        except Exception as e:
            logger.error(f"I2C initialization error: {e}")
            return False
    
    def write_byte(self, register: int, data: int) -> bool:
        """Write byte to register"""
        try:
            if self.device:
                self.device.write_byte_data(self.address, register, data)
            return True
        except Exception as e:
            logger.error(f"I2C write error: {e}")
            return False
    
    def read_byte(self, register: int) -> Optional[int]:
        """Read byte from register"""
        try:
            if self.device:
                return self.device.read_byte_data(self.address, register)
            return None
        except Exception as e:
            logger.error(f"I2C read error: {e}")
            return None
    
    def read_i2c_block_data(self, register: int, length: int) -> Optional[list]:
        """Read block of data"""
        try:
            if self.device:
                return self.device.read_i2c_block_data(self.address, register, length)
            return None
        except Exception as e:
            logger.error(f"I2C block read error: {e}")
            return None
    
    def close(self):
        """Close device"""
        if self.device:
            self.device.close()


class MPU9250(I2CSensor):
    """MPU-9250 9-axis IMU"""
    
    def __init__(self, address: int = 0x68):
        super().__init__(address)
        self.scale_accel = 9.81 / 16384.0
        self.scale_gyro = 250.0 / 32768.0 * 3.14159 / 180.0
    
    def initialize(self) -> bool:
        """Initialize MPU-9250"""
        if not super().initialize():
            return False
        
        # Wake up device
        self.write_byte(0x6B, 0x00)
        time.sleep(0.1)
        
        # Configure accelerometer
        self.write_byte(0x1C, 0x00)  # +/- 2g
        
        # Configure gyroscope
        self.write_byte(0x1B, 0x00)  # +/- 250 deg/s
        
        # Configure magnetometer
        self.write_byte(0x37, 0x02)  # Enable I2C pass-through
        
        logger.info("MPU-9250 initialized")
        return True
    
    def read_accel_gyro(self) -> tuple:
        """Read accelerometer and gyroscope"""
        data = self.read_i2c_block_data(0x3B, 14)
        
        if not data:
            return (0, 0, 0), (0, 0, 0)
        
        # Parse accelerometer (bytes 0-5)
        ax = struct.unpack('>h', bytes([data[0], data[1]]))[0] * self.scale_accel
        ay = struct.unpack('>h', bytes([data[2], data[3]]))[0] * self.scale_accel
        az = struct.unpack('>h', bytes([data[4], data[5]]))[0] * self.scale_accel
        
        # Parse gyroscope (bytes 8-13)
        gx = struct.unpack('>h', bytes([data[8], data[9]]))[0] * self.scale_gyro
        gy = struct.unpack('>h', bytes([data[10], data[11]]))[0] * self.scale_gyro
        gz = struct.unpack('>h', bytes([data[12], data[13]]))[0] * self.scale_gyro
        
        return (ax, ay, az), (gx, gy, gz)


class BMP280(I2CSensor):
    """BMP280 Barometric Pressure Sensor"""
    
    def __init__(self, address: int = 0x76):
        super().__init__(address)
        self.sea_level_pressure = 101325.0
    
    def initialize(self) -> bool:
        """Initialize BMP280"""
        if not super().initialize():
            return False
        
        # Normal mode, T and P oversampling x1
        self.write_byte(0xF4, 0x27)
        
        logger.info("BMP280 initialized")
        return True
    
    def read_bmp280(self) -> tuple:
        """Read pressure and temperature"""
        data = self.read_i2c_block_data(0xF7, 3)
        
        if not data:
            return 101325.0, 25.0, 0.0
        
        # Raw ADC values
        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        
        # Simplified: convert to pressure (Pa) and temperature (C)
        pressure = 101325.0 - (adc_p & 0xFF) * 12.5  # Simplified conversion
        temperature = 25.0  # Placeholder
        
        # Calculate altitude from pressure
        altitude = 44330.0 * (1.0 - pow(pressure / self.sea_level_pressure, 1.0 / 5.255))
        
        return pressure, temperature, altitude


class HMC5883L(I2CSensor):
    """HMC5883L Magnetometer"""
    
    def __init__(self, address: int = 0x1E):
        super().__init__(address)
    
    def initialize(self) -> bool:
        """Initialize HMC5883L"""
        if not super().initialize():
            return False
        
        # Configuration register A
        self.write_byte(0x00, 0x70)  # 8 samples, 15 Hz
        
        # Configuration register B
        self.write_byte(0x01, 0xA0)  # +/- 1.9 Ga
        
        # Mode register - continuous measurement
        self.write_byte(0x02, 0x00)
        
        logger.info("HMC5883L initialized")
        return True
    
    def read_mag(self) -> tuple:
        """Read magnetometer"""
        data = self.read_i2c_block_data(0x03, 6)
        
        if not data:
            return (0, 0, 0)
        
        mx = struct.unpack('>h', bytes([data[0], data[1]]))[0]
        mz = struct.unpack('>h', bytes([data[2], data[3]]))[0]
        my = struct.unpack('>h', bytes([data[4], data[5]]))[0]
        
        return (mx, my, mz)


class GPSModule:
    """GPS Module Interface (u-blox M8N)"""
    
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 38400):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
    
    def initialize(self) -> bool:
        """Initialize GPS"""
        try:
            import serial
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            logger.info(f"GPS initialized on {self.port}")
            return True
        except ImportError:
            logger.warning("pyserial not available, using simulation mode")
            return False
        except Exception as e:
            logger.error(f"GPS initialization error: {e}")
            return False
    
    def read_gps(self) -> tuple:
        """Read GPS data (lat, lon, alt, num_sats)"""
        if not self.serial:
            return (0.0, 0.0, 0.0, 0)
        
        try:
            line = self.serial.readline().decode('utf-8', errors='ignore')
            
            if line.startswith('$GPGGA'):
                parts = line.split(',')
                if len(parts) > 7:
                    # Parse NMEA GPGGA
                    lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
                    lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
                    alt = float(parts[9]) if parts[9] else 0.0
                    sats = int(parts[7]) if parts[7] else 0
                    return (lat, lon, alt, sats)
        
        except Exception as e:
            logger.debug(f"GPS parse error: {e}")
        
        return (0.0, 0.0, 0.0, 0)
    
    def close(self):
        """Close GPS"""
        if self.serial:
            self.serial.close()


class ServoController:
    """PWM Servo Controller"""
    
    def __init__(self):
        self.pwm = None
        self.servos = {}  # Channel -> GPIO pin mapping
    
    def initialize(self) -> bool:
        """Initialize servo controller"""
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            
            # Servo pin mapping for 6-channel output
            self.servos = {
                0: 17,  # Elevator Left (GPIO17)
                1: 27,  # Elevator Right (GPIO27)
                2: 22,  # Aileron Left (GPIO22)
                3: 23,  # Aileron Right (GPIO23)
                4: 24,  # Rudder (GPIO24)
                5: 25   # Throttle (GPIO25)
            }
            
            # Setup GPIO pins
            for channel, pin in self.servos.items():
                GPIO.setup(pin, GPIO.OUT)
                self.pwm = GPIO.PWM(pin, 50)  # 50 Hz PWM
                self.pwm.start(7.5)  # Center position
            
            logger.info("Servo controller initialized")
            return True
        
        except ImportError:
            logger.warning("RPi.GPIO not available, using simulation mode")
            return False
        except Exception as e:
            logger.error(f"Servo controller error: {e}")
            return False
    
    def write_servo(self, channel: int, value: float):
        """Write servo position (0.0-1.0)"""
        # Convert 0.0-1.0 to PWM duty cycle (5-10%)
        duty_cycle = 5.0 + value * 5.0
        logger.debug(f"Servo {channel}: {duty_cycle:.1f}%")
    
    def close(self):
        """Close servo controller"""
        try:
            import RPi.GPIO as GPIO
            GPIO.cleanup()
        except Exception as e:
            logger.error(f"GPIO cleanup error: {e}")


class FlightControllerBoard:
    """Unified Flight Controller Board Interface"""
    
    def __init__(self, config: dict = None):
        self.config = config or {}
        
        # Initialize sensors
        self.imu = MPU9250()
        self.barometer = BMP280()
        self.compass = HMC5883L()
        self.gps = GPSModule()
        self.servo_ctrl = ServoController()
        
        self.initialized = False
    
    def initialize(self) -> bool:
        """Initialize all hardware"""
        logger.info("Initializing Flight Controller Board...")
        
        # Initialize each component
        imu_ok = self.imu.initialize()
        baro_ok = self.barometer.initialize()
        mag_ok = self.compass.initialize()
        gps_ok = self.gps.initialize()
        servo_ok = self.servo_ctrl.initialize()
        
        self.initialized = imu_ok or baro_ok or mag_ok or gps_ok or servo_ok
        
        if self.initialized:
            logger.info("Flight Controller Board initialized")
        else:
            logger.warning("Flight Controller Board initialization incomplete")
        
        return self.initialized
    
    def read_sensors(self) -> Dict:
        """Read all sensors"""
        accel, gyro = self.imu.read_accel_gyro()
        mag = self.compass.read_mag()
        pressure, temperature, altitude = self.barometer.read_bmp280()
        gps = self.gps.read_gps()
        
        return {
            'accel': accel,
            'gyro': gyro,
            'mag': mag,
            'altitude': altitude,
            'pressure': pressure,
            'temperature': temperature,
            'gps': gps
        }
    
    def write_servo(self, channel: int, value: float):
        """Write single servo"""
        self.servo_ctrl.write_servo(channel, value)
    
    def write_all_servos(self, output_dict: dict):
        """Write all servo outputs
        
        Args:
            output_dict: {
                'elevator_left': 0.0-1.0,
                'elevator_right': 0.0-1.0,
                'aileron_left': 0.0-1.0,
                'aileron_right': 0.0-1.0,
                'rudder': 0.0-1.0,
                'throttle': 0.0-1.0
            }
        """
        channel_map = {
            'elevator_left': 0,
            'elevator_right': 1,
            'aileron_left': 2,
            'aileron_right': 3,
            'rudder': 4,
            'throttle': 5
        }
        
        for name, value in output_dict.items():
            if name in channel_map:
                self.write_servo(channel_map[name], value)
    
    def close(self):
        """Close all hardware connections"""
        logger.info("Closing Flight Controller Board connections...")
        self.imu.close()
        self.barometer.close()
        self.compass.close()
        self.gps.close()
        self.servo_ctrl.close()
