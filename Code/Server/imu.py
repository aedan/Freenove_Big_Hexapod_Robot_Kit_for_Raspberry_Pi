# coding:utf-8
"""
IMU sensor fusion using MPU6050 with FIFO mode.
Optimized for Pi 4 with batch sensor reads and vectorized Kalman filtering.
"""

import time
import math
import os
import numpy as np
from typing import Tuple, Optional

from kalman import Kalman_filter
from mpu6050 import mpu6050
from config import get_config
from logger import get_logger

logger = get_logger()


class IMU:
    """
    IMU sensor fusion using Madgwick filter algorithm.
    Supports FIFO mode for batch sensor reads with reduced I2C overhead.
    """

    # MPU6050 FIFO registers
    FIFO_EN = 0x23
    USER_CTRL = 0x6A
    FIFO_COUNT_H = 0x72
    FIFO_COUNT_L = 0x73
    FIFO_R_W = 0x74
    INT_ENABLE = 0x38

    def __init__(self, enable_fifo: bool = True):
        """
        Initialize IMU sensor with optional FIFO mode.

        Args:
            enable_fifo: Enable FIFO buffered reads (default True)
        """
        self.config = get_config()

        # Madgwick filter parameters
        self.proportional_gain = 100
        self.integral_gain = 0.002
        self.half_time_step = 0.001

        # Quaternion state
        self.quaternion_w = 1.0
        self.quaternion_x = 0.0
        self.quaternion_y = 0.0
        self.quaternion_z = 0.0

        # Integral error accumulation
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_z = 0.0

        # Euler angles (degrees)
        self.pitch_angle = 0.0
        self.roll_angle = 0.0
        self.yaw_angle = 0.0

        # Initialize MPU6050
        try:
            self.sensor = mpu6050(
                address=self.config.i2c.imu_address,
                bus=self.config.i2c.bus_number
            )
            self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
            self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)
            logger.info("MPU6050 initialized")
        except Exception as e:
            logger.error(f"Failed to initialize MPU6050: {e}")
            raise

        # Enable FIFO if requested
        self.fifo_enabled = False
        if enable_fifo and self.config.imu.enable_fifo:
            self._enable_fifo()

        # Initialize Kalman filters
        q = self.config.imu.kalman_q
        r = self.config.imu.kalman_r

        self.kalman_filter_AX = Kalman_filter(q, r)
        self.kalman_filter_AY = Kalman_filter(q, r)
        self.kalman_filter_AZ = Kalman_filter(q, r)
        self.kalman_filter_GX = Kalman_filter(q, r)
        self.kalman_filter_GY = Kalman_filter(q, r)
        self.kalman_filter_GZ = Kalman_filter(q, r)

        # Calibrate sensor
        self.error_accel_data, self.error_gyro_data = self._calibrate_sensor()
        logger.info("IMU calibration complete")

    def _enable_fifo(self) -> None:
        """Enable FIFO mode for accelerometer and gyroscope data."""
        try:
            bus = self.sensor.bus

            # Disable FIFO first
            bus.write_byte_data(self.sensor.address, self.USER_CTRL, 0x00)
            time.sleep(0.01)

            # Reset FIFO
            bus.write_byte_data(self.sensor.address, self.USER_CTRL, 0x04)
            time.sleep(0.01)

            # Enable FIFO for accel and gyro
            # Bit 3: ACCEL_FIFO_EN, Bit 6-4: XG_FIFO_EN, YG_FIFO_EN, ZG_FIFO_EN
            bus.write_byte_data(self.sensor.address, self.FIFO_EN, 0x78)

            # Enable FIFO
            bus.write_byte_data(self.sensor.address, self.USER_CTRL, 0x40)

            self.fifo_enabled = True
            logger.info("MPU6050 FIFO mode enabled")

        except Exception as e:
            logger.warning(f"Failed to enable FIFO mode: {e}")
            self.fifo_enabled = False

    def _read_fifo_count(self) -> int:
        """
        Read number of bytes available in FIFO.

        Returns:
            Number of bytes in FIFO buffer
        """
        try:
            bus = self.sensor.bus
            high = bus.read_byte_data(self.sensor.address, self.FIFO_COUNT_H)
            low = bus.read_byte_data(self.sensor.address, self.FIFO_COUNT_L)
            count = (high << 8) | low
            return count
        except Exception as e:
            logger.error(f"Error reading FIFO count: {e}")
            return 0

    def _read_fifo_batch(self) -> Tuple[Optional[dict], Optional[dict]]:
        """
        Read sensor data from FIFO buffer.
        Each FIFO packet is 12 bytes: 6 for accel, 6 for gyro.

        Returns:
            Tuple of (accel_data, gyro_data) or (None, None) on error
        """
        if not self.fifo_enabled:
            # Fallback to normal reads
            return self.sensor.get_accel_data(), self.sensor.get_gyro_data()

        try:
            fifo_count = self._read_fifo_count()

            # Each packet is 12 bytes (6 accel + 6 gyro)
            packet_size = 12
            if fifo_count < packet_size:
                # Not enough data, use normal read
                return self.sensor.get_accel_data(), self.sensor.get_gyro_data()

            # Read one packet from FIFO
            bus = self.sensor.bus
            fifo_data = []
            for _ in range(packet_size):
                fifo_data.append(bus.read_byte_data(self.sensor.address, self.FIFO_R_W))

            # Parse accelerometer data (bytes 0-5)
            accel_x_raw = (fifo_data[0] << 8) | fifo_data[1]
            accel_y_raw = (fifo_data[2] << 8) | fifo_data[3]
            accel_z_raw = (fifo_data[4] << 8) | fifo_data[5]

            # Parse gyroscope data (bytes 6-11)
            gyro_x_raw = (fifo_data[6] << 8) | fifo_data[7]
            gyro_y_raw = (fifo_data[8] << 8) | fifo_data[9]
            gyro_z_raw = (fifo_data[10] << 8) | fifo_data[11]

            # Convert to signed 16-bit
            def to_signed(val):
                return val if val < 32768 else val - 65536

            accel_x_raw = to_signed(accel_x_raw)
            accel_y_raw = to_signed(accel_y_raw)
            accel_z_raw = to_signed(accel_z_raw)
            gyro_x_raw = to_signed(gyro_x_raw)
            gyro_y_raw = to_signed(gyro_y_raw)
            gyro_z_raw = to_signed(gyro_z_raw)

            # Apply scale modifiers
            accel_scale = self.sensor.ACCEL_SCALE_MODIFIER_2G
            gyro_scale = self.sensor.GYRO_SCALE_MODIFIER_250DEG

            accel_x = (accel_x_raw / accel_scale) * self.sensor.GRAVITIY_MS2
            accel_y = (accel_y_raw / accel_scale) * self.sensor.GRAVITIY_MS2
            accel_z = (accel_z_raw / accel_scale) * self.sensor.GRAVITIY_MS2

            gyro_x = gyro_x_raw / gyro_scale
            gyro_y = gyro_y_raw / gyro_scale
            gyro_z = gyro_z_raw / gyro_scale

            accel_data = {'x': accel_x, 'y': accel_y, 'z': accel_z}
            gyro_data = {'x': gyro_x, 'y': gyro_y, 'z': gyro_z}

            return accel_data, gyro_data

        except Exception as e:
            logger.error(f"FIFO read error: {e}")
            # Fallback to normal read
            return self.sensor.get_accel_data(), self.sensor.get_gyro_data()

    def _calibrate_sensor(self) -> Tuple[dict, dict]:
        """
        Calibrate sensor by averaging multiple samples.
        Uses adaptive sample count from config.

        Returns:
            Tuple of (accel_offset, gyro_offset)
        """
        sample_count = self.config.imu.calibration_samples
        logger.info(f"Calibrating IMU with {sample_count} samples...")

        # Use NumPy arrays for efficient accumulation
        accel_sum = np.zeros(3)
        gyro_sum = np.zeros(3)

        for i in range(sample_count):
            if self.fifo_enabled:
                accel_data, gyro_data = self._read_fifo_batch()
            else:
                accel_data = self.sensor.get_accel_data()
                gyro_data = self.sensor.get_gyro_data()

            accel_sum[0] += accel_data['x']
            accel_sum[1] += accel_data['y']
            accel_sum[2] += accel_data['z']

            gyro_sum[0] += gyro_data['x']
            gyro_sum[1] += gyro_data['y']
            gyro_sum[2] += gyro_data['z']

            time.sleep(0.01)

        # Calculate averages
        accel_avg = accel_sum / sample_count
        gyro_avg = gyro_sum / sample_count

        # Subtract gravity from Z axis (9.8 m/s^2)
        accel_avg[2] -= 9.8

        accel_offset = {'x': accel_avg[0], 'y': accel_avg[1], 'z': accel_avg[2]}
        gyro_offset = {'x': gyro_avg[0], 'y': gyro_avg[1], 'z': gyro_avg[2]}

        logger.info(f"Accel offset: {accel_offset}")
        logger.info(f"Gyro offset: {gyro_offset}")

        return accel_offset, gyro_offset

    def update_imu_state(self) -> Tuple[float, float, float]:
        """
        Update IMU state using Madgwick filter algorithm.
        Reads sensor data and computes orientation.

        Returns:
            Tuple of (pitch, roll, yaw) in degrees
        """
        # Read sensor data (from FIFO if enabled)
        if self.fifo_enabled:
            accel_data, gyro_data = self._read_fifo_batch()
        else:
            accel_data = self.sensor.get_accel_data()
            gyro_data = self.sensor.get_gyro_data()

        # Apply calibration offsets and Kalman filtering
        accel_x = self.kalman_filter_AX.kalman(
            accel_data['x'] - self.error_accel_data['x']
        )
        accel_y = self.kalman_filter_AY.kalman(
            accel_data['y'] - self.error_accel_data['y']
        )
        accel_z = self.kalman_filter_AZ.kalman(
            accel_data['z'] - self.error_accel_data['z']
        )

        gyro_x = self.kalman_filter_GX.kalman(
            gyro_data['x'] - self.error_gyro_data['x']
        )
        gyro_y = self.kalman_filter_GY.kalman(
            gyro_data['y'] - self.error_gyro_data['y']
        )
        gyro_z = self.kalman_filter_GZ.kalman(
            gyro_data['z'] - self.error_gyro_data['z']
        )

        # Normalize accelerometer measurements
        accel_norm = math.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z)

        if accel_norm == 0:
            return self.pitch_angle, self.roll_angle, self.yaw_angle

        accel_x /= accel_norm
        accel_y /= accel_norm
        accel_z /= accel_norm

        # Estimated direction of gravity from quaternion
        quat_vx = 2 * (self.quaternion_x * self.quaternion_z - self.quaternion_w * self.quaternion_y)
        quat_vy = 2 * (self.quaternion_w * self.quaternion_x + self.quaternion_y * self.quaternion_z)
        quat_vz = (self.quaternion_w * self.quaternion_w - self.quaternion_x * self.quaternion_x -
                   self.quaternion_y * self.quaternion_y + self.quaternion_z * self.quaternion_z)

        # Error is sum of cross product between estimated direction and measured direction
        error_x = (accel_y * quat_vz - accel_z * quat_vy)
        error_y = (accel_z * quat_vx - accel_x * quat_vz)
        error_z = (accel_x * quat_vy - accel_y * quat_vx)

        # Accumulate integral error
        self.integral_error_x += error_x * self.integral_gain
        self.integral_error_y += error_y * self.integral_gain
        self.integral_error_z += error_z * self.integral_gain

        # Apply feedback to gyro
        gyro_x += self.proportional_gain * error_x + self.integral_error_x
        gyro_y += self.proportional_gain * error_y + self.integral_error_y
        gyro_z += self.proportional_gain * error_z + self.integral_error_z

        # Integrate rate of change of quaternion
        self.quaternion_w += (-self.quaternion_x * gyro_x - self.quaternion_y * gyro_y -
                             self.quaternion_z * gyro_z) * self.half_time_step
        self.quaternion_x += (self.quaternion_w * gyro_x + self.quaternion_y * gyro_z -
                             self.quaternion_z * gyro_y) * self.half_time_step
        self.quaternion_y += (self.quaternion_w * gyro_y - self.quaternion_x * gyro_z +
                             self.quaternion_z * gyro_x) * self.half_time_step
        self.quaternion_z += (self.quaternion_w * gyro_z + self.quaternion_x * gyro_y -
                             self.quaternion_y * self.quaternion_x) * self.half_time_step

        # Normalize quaternion
        norm = math.sqrt(
            self.quaternion_w * self.quaternion_w +
            self.quaternion_x * self.quaternion_x +
            self.quaternion_y * self.quaternion_y +
            self.quaternion_z * self.quaternion_z
        )

        self.quaternion_w /= norm
        self.quaternion_x /= norm
        self.quaternion_y /= norm
        self.quaternion_z /= norm

        # Convert quaternion to Euler angles
        current_pitch = math.asin(
            -2 * self.quaternion_x * self.quaternion_z + 2 * self.quaternion_w * self.quaternion_y
        ) * 57.3

        current_roll = math.atan2(
            2 * self.quaternion_y * self.quaternion_z + 2 * self.quaternion_w * self.quaternion_x,
            -2 * self.quaternion_x * self.quaternion_x - 2 * self.quaternion_y * self.quaternion_y + 1
        ) * 57.3

        current_yaw = math.atan2(
            2 * (self.quaternion_x * self.quaternion_y + self.quaternion_w * self.quaternion_z),
            self.quaternion_w * self.quaternion_w + self.quaternion_x * self.quaternion_x -
            self.quaternion_y * self.quaternion_y - self.quaternion_z * self.quaternion_z
        ) * 57.3

        self.pitch_angle = current_pitch
        self.roll_angle = current_roll
        self.yaw_angle = current_yaw

        return self.pitch_angle, self.roll_angle, self.yaw_angle

    def handle_exception(self, exception: Exception) -> None:
        """
        Handle IMU exceptions and run I2C diagnostics.

        Args:
            exception: Exception to handle
        """
        logger.error(f"IMU error: {exception}")
        logger.info("Running I2C diagnostics...")
        os.system("i2cdetect -y 1")
        raise exception

    def reset_fifo(self) -> None:
        """Reset FIFO buffer (useful if overflow detected)."""
        if not self.fifo_enabled:
            return

        try:
            bus = self.sensor.bus
            # Reset FIFO
            bus.write_byte_data(self.sensor.address, self.USER_CTRL, 0x04)
            time.sleep(0.01)
            # Re-enable FIFO
            bus.write_byte_data(self.sensor.address, self.USER_CTRL, 0x40)
            logger.debug("FIFO reset")
        except Exception as e:
            logger.error(f"FIFO reset error: {e}")

    def close(self) -> None:
        """Cleanup IMU resources."""
        if self.fifo_enabled:
            try:
                # Disable FIFO
                bus = self.sensor.bus
                bus.write_byte_data(self.sensor.address, self.USER_CTRL, 0x00)
                logger.info("IMU closed")
            except Exception as e:
                logger.warning(f"Error closing IMU: {e}")


if __name__ == '__main__':
    from logger import setup_logger

    setup_logger(log_level='INFO')

    logger.info("Starting IMU test")
    imu_sensor = IMU(enable_fifo=True)

    try:
        start_time = time.time()
        frame_count = 0

        while True:
            time.sleep(0.01)
            pitch, roll, yaw = imu_sensor.update_imu_state()

            frame_count += 1
            if frame_count % 100 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                logger.info(f"Pitch: {pitch:.2f}, Roll: {roll:.2f}, Yaw: {yaw:.2f} | {fps:.1f} Hz")

    except KeyboardInterrupt:
        logger.info("IMU test stopped")
    except Exception as e:
        imu_sensor.handle_exception(e)
    finally:
        imu_sensor.close()
