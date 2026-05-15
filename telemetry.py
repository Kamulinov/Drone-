#!/usr/bin/env python3
"""
Telemetry Manager for MUGIN-3 Flight Control System
Handles MAVLink protocol, data logging, and telemetry servers
"""

import asyncio
import json
import logging
import csv
from datetime import datetime
from typing import Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class TelemetryServer:
    """UDP/TCP telemetry server"""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 14550):
        self.host = host
        self.port = port
        self.transport = None
        self.protocol = None
    
    async def start(self):
        """Start telemetry server"""
        logger.info(f"Starting telemetry server on {self.host}:{self.port}")
        
        try:
            loop = asyncio.get_event_loop()
            self.transport, self.protocol = await loop.create_datagram_endpoint(
                lambda: TelemetryProtocol(),
                local_addr=(self.host, self.port)
            )
            logger.info("Telemetry server started")
        except Exception as e:
            logger.error(f"Telemetry server error: {e}")
    
    async def broadcast_telemetry(self, telemetry_data: dict):
        """Broadcast telemetry data"""
        try:
            if self.protocol:
                await self.protocol.send_telemetry(telemetry_data)
        except Exception as e:
            logger.error(f"Broadcast error: {e}")
    
    def stop(self):
        """Stop telemetry server"""
        if self.transport:
            self.transport.close()
        logger.info("Telemetry server stopped")


class TelemetryProtocol(asyncio.DatagramProtocol):
    """Telemetry protocol handler"""
    
    def __init__(self):
        self.transport = None
        self.clients = set()
    
    def connection_made(self, transport):
        """Connection made"""
        self.transport = transport
        logger.info("Telemetry protocol connection made")
    
    def datagram_received(self, data, addr):
        """Datagram received"""
        try:
            message = json.loads(data.decode())
            self.clients.add(addr)
            logger.debug(f"Telemetry received from {addr}")
        except Exception as e:
            logger.error(f"Protocol error: {e}")
    
    async def send_telemetry(self, telemetry_data: dict):
        """Send telemetry to all connected clients"""
        if not self.clients:
            return
        
        message = json.dumps(telemetry_data).encode()
        
        for client_addr in list(self.clients):
            try:
                self.transport.sendto(message, client_addr)
            except Exception as e:
                logger.error(f"Send error: {e}")
                self.clients.discard(client_addr)


class MAVLinkInterface:
    """MAVLink protocol interface"""
    
    def __init__(self):
        self.mavlink = None
        self._init_mavlink()
    
    def _init_mavlink(self):
        """Initialize MAVLink"""
        try:
            from pymavlink import mavutil
            self.mavlink = mavutil
            logger.info("MAVLink interface initialized")
        except ImportError:
            logger.warning("pymavlink not available")
    
    def create_mavlink_message(self, message_type: str, **kwargs) -> bytes:
        """Create MAVLink message"""
        message = {
            'type': message_type,
            'timestamp': datetime.now().isoformat(),
            **kwargs
        }
        return json.dumps(message).encode()


class DataLogger:
    """Flight data logger"""
    
    def __init__(self, log_dir: str = "/var/log/mugin3"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.json_file = None
        self.csv_file = None
        self.csv_writer = None
        
        self._init_log_files()
    
    def _init_log_files(self):
        """Initialize log files"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # JSON log
        json_path = self.log_dir / f"flight_{timestamp}.json"
        self.json_file = open(json_path, 'w')
        logger.info(f"JSON log file: {json_path}")
        
        # CSV log
        csv_path = self.log_dir / f"flight_{timestamp}.csv"
        self.csv_file = open(csv_path, 'w', newline='')
        
        # CSV headers
        headers = [
            'timestamp', 'armed', 'mode',
            'roll', 'pitch', 'yaw',
            'roll_rate', 'pitch_rate', 'yaw_rate',
            'altitude', 'pressure', 'temperature',
            'gps_satellites',
            'target_roll', 'target_pitch', 'target_yaw', 'target_altitude',
            'elevator_left', 'elevator_right',
            'aileron_left', 'aileron_right',
            'rudder', 'throttle'
        ]
        
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=headers)
        self.csv_writer.writeheader()
        
        logger.info(f"CSV log file: {csv_path}")
    
    def log_telemetry(self, telemetry_data: dict):
        """Log telemetry data"""
        try:
            # JSON log
            self.json_file.write(json.dumps(telemetry_data) + '\n')
            self.json_file.flush()
            
            # CSV log
            csv_data = self._flatten_telemetry(telemetry_data)
            self.csv_writer.writerow(csv_data)
            self.csv_file.flush()
        
        except Exception as e:
            logger.error(f"Logging error: {e}")
    
    @staticmethod
    def _flatten_telemetry(telemetry_data: dict) -> dict:
        """Flatten nested telemetry data for CSV"""
        flat = {
            'timestamp': telemetry_data.get('timestamp', ''),
            'armed': telemetry_data.get('armed', False),
            'mode': telemetry_data.get('mode', ''),
        }
        
        # Attitude
        attitude = telemetry_data.get('attitude', {})
        flat.update({
            'roll': attitude.get('roll', 0),
            'pitch': attitude.get('pitch', 0),
            'yaw': attitude.get('yaw', 0),
            'roll_rate': attitude.get('roll_rate', 0),
            'pitch_rate': attitude.get('pitch_rate', 0),
            'yaw_rate': attitude.get('yaw_rate', 0),
        })
        
        # Measurements
        flat.update({
            'altitude': telemetry_data.get('altitude', 0),
            'pressure': telemetry_data.get('pressure', 0),
            'temperature': telemetry_data.get('temperature', 0),
            'gps_satellites': telemetry_data.get('gps_satellites', 0),
        })
        
        # Target
        target = telemetry_data.get('target', {})
        flat.update({
            'target_roll': target.get('roll', 0),
            'target_pitch': target.get('pitch', 0),
            'target_yaw': target.get('yaw', 0),
            'target_altitude': target.get('target_altitude', 0),
        })
        
        # Control output
        output = telemetry_data.get('control_output', {})
        flat.update({
            'elevator_left': output.get('elevator_left', 0),
            'elevator_right': output.get('elevator_right', 0),
            'aileron_left': output.get('aileron_left', 0),
            'aileron_right': output.get('aileron_right', 0),
            'rudder': output.get('rudder', 0),
            'throttle': output.get('throttle', 0),
        })
        
        return flat
    
    def close(self):
        """Close log files"""
        if self.json_file:
            self.json_file.close()
        if self.csv_file:
            self.csv_file.close()
        logger.info("Data logger closed")


class TelemetryManager:
    """Main telemetry manager"""
    
    def __init__(self, fcs, host: str = "0.0.0.0", port: int = 14550):
        self.fcs = fcs
        self.host = host
        self.port = port
        
        # Initialize components
        self.telemetry_server = TelemetryServer(host, port)
        self.mavlink_interface = MAVLinkInterface()
        self.data_logger = DataLogger()
        
        logger.info("Telemetry Manager initialized")
    
    async def start(self):
        """Start telemetry systems"""
        logger.info("Starting telemetry systems...")
        
        try:
            await self.telemetry_server.start()
        except Exception as e:
            logger.error(f"Telemetry manager error: {e}")
    
    async def log_and_broadcast_telemetry(self):
        """Log and broadcast telemetry data"""
        while True:
            try:
                if self.fcs:
                    telemetry = self.fcs.get_telemetry()
                    
                    # Log data
                    self.data_logger.log_telemetry(telemetry)
                    
                    # Broadcast
                    await self.telemetry_server.broadcast_telemetry(telemetry)
                
                await asyncio.sleep(0.1)
            
            except Exception as e:
                logger.error(f"Telemetry loop error: {e}")
                await asyncio.sleep(1)
    
    def close(self):
        """Close telemetry systems"""
        self.telemetry_server.stop()
        self.data_logger.close()
        logger.info("Telemetry Manager closed")
