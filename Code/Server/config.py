# -*- coding: utf-8 -*-
"""
Centralized configuration system for Freenove Hexapod Robot Server.
Provides type-safe configuration using dataclasses with JSON serialization support.
"""

import json
import os
from dataclasses import dataclass, field, asdict
from typing import List, Tuple, Optional
from pathlib import Path


@dataclass
class NetworkConfig:
    """Network communication configuration."""
    video_port: int = 8002
    command_port: int = 5002
    interface: str = 'wlan0'
    buffer_size: int = 1024
    enable_tcp_nodelay: bool = True
    connection_timeout: float = 10.0
    robot_name: str = "HexapodRobot"  # Robot name for network discovery


@dataclass
class I2CConfig:
    """I2C bus configuration."""
    bus_number: int = 1
    clock_speed_hz: int = 400000  # 400kHz for Pi 4
    pwm_addresses: List[int] = field(default_factory=lambda: [0x40, 0x41])
    adc_address: int = 0x48
    imu_address: int = 0x68


@dataclass
class ServoConfig:
    """Servo and PWM configuration."""
    pwm_frequency: int = 50  # Hz
    angle_min: int = 0
    angle_max: int = 180
    duty_cycle_min: int = 500
    duty_cycle_max: int = 2500
    pwm_resolution: int = 4095
    enable_debug: bool = False  # Disable for production
    power_disable_pin: int = 4


@dataclass
class CameraConfig:
    """Camera and video streaming configuration."""
    resolution: Tuple[int, int] = (640, 480)  # Width x Height (Pi 4 optimized)
    framerate: int = 30
    encoder: str = 'h264'  # 'h264' or 'jpeg'
    bitrate: int = 10000000  # 10 Mbps for H264
    intra_period: int = 30  # I-frame interval
    gpu_memory_mb: int = 256
    frame_buffer_size: int = 3


@dataclass
class IMUConfig:
    """IMU sensor configuration."""
    enable_fifo: bool = True
    calibration_samples: int = 50  # Reduced from 100
    sampling_rate_hz: int = 50
    kalman_q: float = 0.001
    kalman_r: float = 0.1


@dataclass
class LEDConfig:
    """LED control configuration."""
    pixel_count: int = 8
    gpio_pin: int = 18
    frequency_hz: int = 1200000  # 1.2 MHz for Pi 4
    dma_channel: int = 10
    brightness: int = 255
    spi_speed_hz: int = 12000000  # 12 MHz for SPI-based LEDs


@dataclass
class SensorConfig:
    """Sensor pins and configuration."""
    ultrasonic_trigger_pin: int = 27
    ultrasonic_echo_pin: int = 22
    ultrasonic_max_distance: float = 3.0
    buzzer_pin: int = 17
    adc_voltage_coefficient: float = 3.0


@dataclass
class RobotGeometry:
    """Robot physical dimensions and parameters."""
    body_height: float = -25.0
    body_points: List[List[float]] = field(default_factory=lambda: [
        [137.1, 189.4, -25.0],
        [225.0, 0.0, -25.0],
        [137.1, -189.4, -25.0],
        [-137.1, -189.4, -25.0],
        [-225.0, 0.0, -25.0],
        [-137.1, 189.4, -25.0]
    ])
    leg_link_lengths: Tuple[float, float, float] = (33.0, 90.0, 110.0)  # l1, l2, l3
    default_leg_position: Tuple[float, float, float] = (140.0, 0.0, 0.0)


@dataclass
class MovementConfig:
    """Movement and gait configuration."""
    position_limit_x: Tuple[int, int] = (-40, 40)
    position_limit_y: Tuple[int, int] = (-40, 40)
    position_limit_z: Tuple[int, int] = (-20, 20)
    speed_min: int = 2
    speed_max: int = 10
    default_speed: int = 8
    gait_step_height: int = 40
    gait_frame_delay: float = 0.01  # seconds
    enable_non_blocking_gait: bool = True


@dataclass
class PerformanceConfig:
    """Performance tuning parameters."""
    condition_monitor_timeout: float = 0.1  # seconds
    thread_join_timeout: float = 5.0
    servo_batch_updates: bool = True
    enable_coordinate_caching: bool = True
    command_timeout: float = 10.0


@dataclass
class LoggingConfig:
    """Logging configuration."""
    log_level: str = 'INFO'  # DEBUG, INFO, WARNING, ERROR
    log_to_file: bool = True
    log_to_console: bool = True
    log_file_path: str = '/tmp/hexapod_server.log'
    log_max_bytes: int = 10485760  # 10 MB
    log_backup_count: int = 3


@dataclass
class RobotConfig:
    """Complete robot configuration."""
    network: NetworkConfig = field(default_factory=NetworkConfig)
    i2c: I2CConfig = field(default_factory=I2CConfig)
    servo: ServoConfig = field(default_factory=ServoConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    led: LEDConfig = field(default_factory=LEDConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    geometry: RobotGeometry = field(default_factory=RobotGeometry)
    movement: MovementConfig = field(default_factory=MovementConfig)
    performance: PerformanceConfig = field(default_factory=PerformanceConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)

    @classmethod
    def load_from_file(cls, config_file: str = 'config.json') -> 'RobotConfig':
        """
        Load configuration from JSON file.

        Args:
            config_file: Path to JSON configuration file

        Returns:
            RobotConfig instance with loaded values

        Raises:
            FileNotFoundError: If config file doesn't exist
            json.JSONDecodeError: If config file is invalid JSON
        """
        config_path = Path(config_file)
        if not config_path.exists():
            # Return default configuration if file doesn't exist
            print(f"Config file {config_file} not found, using defaults")
            return cls()

        with open(config_path, 'r') as f:
            config_dict = json.load(f)

        # Reconstruct nested dataclasses
        network = NetworkConfig(**config_dict.get('network', {}))
        i2c = I2CConfig(**config_dict.get('i2c', {}))
        servo = ServoConfig(**config_dict.get('servo', {}))
        camera = CameraConfig(**config_dict.get('camera', {}))
        imu = IMUConfig(**config_dict.get('imu', {}))
        led = LEDConfig(**config_dict.get('led', {}))
        sensor = SensorConfig(**config_dict.get('sensor', {}))
        geometry = RobotGeometry(**config_dict.get('geometry', {}))
        movement = MovementConfig(**config_dict.get('movement', {}))
        performance = PerformanceConfig(**config_dict.get('performance', {}))
        logging_cfg = LoggingConfig(**config_dict.get('logging', {}))

        return cls(
            network=network,
            i2c=i2c,
            servo=servo,
            camera=camera,
            imu=imu,
            led=led,
            sensor=sensor,
            geometry=geometry,
            movement=movement,
            performance=performance,
            logging=logging_cfg
        )

    def save_to_file(self, config_file: str = 'config.json') -> None:
        """
        Save configuration to JSON file.

        Args:
            config_file: Path to JSON configuration file
        """
        config_dict = asdict(self)
        with open(config_file, 'w') as f:
            json.dump(config_dict, f, indent=2)
        print(f"Configuration saved to {config_file}")

    def validate(self) -> bool:
        """
        Validate configuration values.

        Returns:
            True if configuration is valid

        Raises:
            ValueError: If configuration contains invalid values
        """
        # Network validation
        if not (1024 <= self.network.video_port <= 65535):
            raise ValueError(f"Invalid video port: {self.network.video_port}")
        if not (1024 <= self.network.command_port <= 65535):
            raise ValueError(f"Invalid command port: {self.network.command_port}")

        # I2C validation
        if self.i2c.clock_speed_hz not in [100000, 400000, 1000000]:
            raise ValueError(f"Invalid I2C speed: {self.i2c.clock_speed_hz} Hz")

        # Servo validation
        if not (0 < self.servo.pwm_frequency <= 1000):
            raise ValueError(f"Invalid PWM frequency: {self.servo.pwm_frequency} Hz")

        # Camera validation
        if self.camera.encoder not in ['h264', 'jpeg', 'mjpeg']:
            raise ValueError(f"Invalid encoder: {self.camera.encoder}")

        # Logging validation
        if self.logging.log_level not in ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']:
            raise ValueError(f"Invalid log level: {self.logging.log_level}")

        return True


# Global configuration instance
_config: Optional[RobotConfig] = None


def get_config() -> RobotConfig:
    """
    Get the global configuration instance.
    Loads from config.json if exists, otherwise uses defaults.

    Returns:
        RobotConfig instance
    """
    global _config
    if _config is None:
        _config = RobotConfig.load_from_file()
        _config.validate()
    return _config


def reload_config(config_file: str = 'config.json') -> RobotConfig:
    """
    Reload configuration from file.

    Args:
        config_file: Path to configuration file

    Returns:
        Reloaded RobotConfig instance
    """
    global _config
    _config = RobotConfig.load_from_file(config_file)
    _config.validate()
    return _config


if __name__ == '__main__':
    # Example: Generate default configuration file
    config = RobotConfig()
    config.save_to_file('config_default.json')
    print("Default configuration saved to config_default.json")

    # Validate
    try:
        config.validate()
        print("Configuration validation passed")
    except ValueError as e:
        print(f"Configuration validation failed: {e}")
