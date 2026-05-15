#!/usr/bin/env python3
"""
Main entry point for MUGIN-3 Flight Control System
Integrates flight control, hardware, and telemetry systems
"""

import asyncio
import logging
import signal
import sys
import json
from pathlib import Path

# Import flight control modules
from flight_control_system import FlightControlSystem, FlightControlServer
from hardware_interface import FlightControllerBoard
from telemetry import TelemetryManager

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/mugin3/flight_control_main.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class MuginFlightControlApplication:
    """Main application class"""
    
    def __init__(self, config_file: str = "/etc/mugin3/flight_control.json"):
        self.config_file = config_file
        self.config = self._load_config()
        
        # Initialize systems
        self.fcs = None
        self.fc_board = None
        self.server = None
        self.telemetry_manager = None
        
        self.running = False
        
        logger.info("MUGIN-3 Flight Control Application initialized")
    
    def _load_config(self) -> dict:
        """Load configuration"""
        try:
            with open(self.config_file, 'r') as f:
                config = json.load(f)
            logger.info(f"Configuration loaded from {self.config_file}")
            return config
        except FileNotFoundError:
            logger.warning(f"Config file not found: {self.config_file}")
            return {}
    
    def initialize(self) -> bool:
        """Initialize all systems"""
        try:
            logger.info("Initializing Flight Control Systems...")
            
            # Initialize flight control system
            self.fcs = FlightControlSystem(self.config_file)
            logger.info("Flight Control System initialized")
            
            # Initialize flight controller board (hardware)
            self.fc_board = FlightControllerBoard(self.config)
            if not self.fc_board.initialize():
                logger.warning("Flight controller board initialization had issues")
            logger.info("Flight Controller Board initialized")
            
            # Initialize flight control server
            self.server = FlightControlServer(self.fcs)
            logger.info("Flight Control Server initialized")
            
            # Initialize telemetry manager
            self.telemetry_manager = TelemetryManager(self.fcs)
            logger.info("Telemetry Manager initialized")
            
            logger.info("All systems initialized successfully")
            return True
        
        except Exception as e:
            logger.error(f"Initialization error: {e}", exc_info=True)
            return False
    
    def setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown"""
        def signal_handler(sig, frame):
            logger.info(f"Signal {sig} received, shutting down...")
            asyncio.create_task(self.shutdown())
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    
    async def shutdown(self):
        """Graceful shutdown"""
        logger.info("Shutting down Flight Control System...")
        self.running = False
        
        if self.fcs and self.fcs.armed:
            self.fcs.disarm()
        
        if self.fc_board:
            self.fc_board.close()
        
        if self.server:
            self.server.stop()
        
        if self.telemetry_manager:
            self.telemetry_manager.close()
        
        logger.info("Flight Control System shutdown complete")
    
    async def sensor_read_loop(self):
        """Continuously read sensors and update FCS"""
        logger.info("Sensor read loop started")
        
        while self.running:
            try:
                # Read sensor data from flight controller board
                if self.fc_board:
                    sensor_data = self.fc_board.read_sensors()
                    
                    if sensor_data:
                        # Update FCS with sensor data
                        from flight_control_system import SensorData
                        
                        fcs_sensor_data = SensorData(
                            timestamp=asyncio.get_event_loop().time(),
                            accel_x=sensor_data.get('accel', [0, 0, 0])[0],
                            accel_y=sensor_data.get('accel', [0, 0, 0])[1],
                            accel_z=sensor_data.get('accel', [0, 0, 0])[2],
                            gyro_x=sensor_data.get('gyro', [0, 0, 0])[0],
                            gyro_y=sensor_data.get('gyro', [0, 0, 0])[1],
                            gyro_z=sensor_data.get('gyro', [0, 0, 0])[2],
                            altitude=sensor_data.get('altitude', 0),
                            pressure=sensor_data.get('pressure', 101325),
                            temperature=sensor_data.get('temperature', 25),
                            gps_satellites=sensor_data.get('gps', [0, 0, 0, 0])[3]
                        )
                        
                        self.fcs.update_sensor_data(fcs_sensor_data)
                
                await asyncio.sleep(0.01)  # 100 Hz sensor read rate
            
            except Exception as e:
                logger.error(f"Sensor read loop error: {e}")
                await asyncio.sleep(0.1)
    
    async def control_output_loop(self):
        """Write control outputs to servos"""
        logger.info("Control output loop started")
        
        while self.running:
            try:
                if self.fcs and self.fcs.armed and self.fc_board:
                    # Get control outputs from FCS
                    control_output = self.fcs.current_control_output
                    
                    # Write to servos
                    output_dict = {
                        'elevator_left': control_output.elevator_left,
                        'elevator_right': control_output.elevator_right,
                        'aileron_left': control_output.aileron_left,
                        'aileron_right': control_output.aileron_right,
                        'rudder': control_output.rudder,
                        'throttle': control_output.throttle
                    }
                    
                    self.fc_board.write_all_servos(output_dict)
                
                await asyncio.sleep(0.02)  # 50 Hz control output rate
            
            except Exception as e:
                logger.error(f"Control output loop error: {e}")
                await asyncio.sleep(0.1)
    
    async def telemetry_loop(self):
        """Broadcast telemetry data"""
        logger.info("Telemetry loop started")
        
        while self.running:
            try:
                if self.fcs and self.telemetry_manager:
                    telemetry = self.fcs.get_telemetry()
                    await self.telemetry_manager.telemetry_server.broadcast_telemetry(telemetry)
                
                await asyncio.sleep(0.1)  # 10 Hz telemetry rate
            
            except Exception as e:
                logger.error(f"Telemetry loop error: {e}")
                await asyncio.sleep(0.1)
    
    async def run(self):
        """Main run loop"""
        if not self.initialize():
            logger.error("Failed to initialize systems")
            return False
        
        self.running = True
        self.setup_signal_handlers()
        
        logger.info("Starting MUGIN-3 Flight Control System")
        
        try:
            # Create async tasks for all loops
            tasks = [
                asyncio.create_task(self.sensor_read_loop()),
                asyncio.create_task(self.server.start()),
                asyncio.create_task(self.control_output_loop()),
                asyncio.create_task(self.telemetry_loop()),
                asyncio.create_task(self.telemetry_manager.start())
            ]
            
            # Run until shutdown
            await asyncio.gather(*tasks)
        
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received")
        except asyncio.CancelledError:
            logger.info("Tasks cancelled")
        except Exception as e:
            logger.error(f"Application error: {e}", exc_info=True)
        
        finally:
            await self.shutdown()


async def main():
    """Application entry point"""
    logger.info("=" * 60)
    logger.info("MUGIN-3 3600MM H-Tail UAV Flight Control System")
    logger.info("Version: 1.0.0")
    logger.info("=" * 60)
    
    app = MuginFlightControlApplication()
    await app.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Application terminated by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)
