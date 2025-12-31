# coding:utf-8
"""
Servo control module for Freenove Hexapod Robot.
Optimized for Pi 4 with batch PWM operations and proper resource management.
"""

from pca9685 import PCA9685
from typing import Dict, Tuple
from config import get_config
from logger import get_logger

logger = get_logger()


def angle_to_duty_cycle(angle: float) -> int:
    """
    Convert servo angle to PWM duty cycle value.
    Combines the two-step mapping into a single efficient calculation.

    Args:
        angle: Angle in degrees (0-180)

    Returns:
        Duty cycle value for PCA9685 (0-4095)
    """
    config = get_config()
    # Map angle (0-180) -> pulse width (500-2500 us) -> duty cycle (0-4095)
    pulse_us = (
        config.servo.duty_cycle_min +
        (config.servo.duty_cycle_max - config.servo.duty_cycle_min) *
        (angle - config.servo.angle_min) / (config.servo.angle_max - config.servo.angle_min)
    )
    duty_cycle = pulse_us * config.servo.pwm_resolution / 20000.0
    return int(duty_cycle)


class Servo:
    """
    Servo controller managing two PCA9685 PWM drivers.
    Supports batch updates for improved performance.
    """

    def __init__(self):
        """Initialize servo controllers and configuration."""
        self.config = get_config()

        # Initialize PCA9685 controllers (debug mode removed for production)
        logger.info("Initializing servo controllers")
        self.pwm_40 = PCA9685(self.config.i2c.pwm_addresses[0])  # Servos 16-31
        self.pwm_41 = PCA9685(self.config.i2c.pwm_addresses[1])  # Servos 0-15

        # Set PWM frequency (no sleep needed - handled in PCA9685)
        self.pwm_40.set_pwm_freq(self.config.servo.pwm_frequency)
        self.pwm_41.set_pwm_freq(self.config.servo.pwm_frequency)

        # Cache for pending batch updates
        self._batch_pwm_40: Dict[int, Tuple[int, int]] = {}
        self._batch_pwm_41: Dict[int, Tuple[int, int]] = {}

        logger.info("Servo controllers initialized successfully")

    def set_servo_angle(self, channel: int, angle: float) -> None:
        """
        Set a single servo to a specific angle.

        Args:
            channel: Servo channel (0-31)
            angle: Target angle in degrees (0-180)
        """
        # Clamp angle to valid range
        angle = max(self.config.servo.angle_min,
                    min(self.config.servo.angle_max, angle))

        duty_cycle = angle_to_duty_cycle(angle)

        if channel < 16:
            # Channels 0-15 on pwm_41
            self.pwm_41.set_pwm(channel, 0, duty_cycle)
        elif channel < 32:
            # Channels 16-31 on pwm_40
            self.pwm_40.set_pwm(channel - 16, 0, duty_cycle)
        else:
            logger.warning(f"Invalid servo channel: {channel}")

    def queue_servo_angle(self, channel: int, angle: float) -> None:
        """
        Queue a servo angle update for batch processing.
        Call flush_batch() to apply all queued updates.

        Args:
            channel: Servo channel (0-31)
            angle: Target angle in degrees (0-180)
        """
        # Clamp angle to valid range
        angle = max(self.config.servo.angle_min,
                    min(self.config.servo.angle_max, angle))

        duty_cycle = angle_to_duty_cycle(angle)

        if channel < 16:
            # Channels 0-15 on pwm_41
            self._batch_pwm_41[channel] = (0, duty_cycle)
        elif channel < 32:
            # Channels 16-31 on pwm_40
            self._batch_pwm_40[channel - 16] = (0, duty_cycle)
        else:
            logger.warning(f"Invalid servo channel: {channel}")

    def flush_batch(self) -> None:
        """
        Apply all queued servo angle updates in batch.
        Significantly faster than individual updates (2-4ms vs 18-36ms).
        """
        if self._batch_pwm_41:
            self.pwm_41.set_pwm_batch(self._batch_pwm_41)
            self._batch_pwm_41.clear()

        if self._batch_pwm_40:
            self.pwm_40.set_pwm_batch(self._batch_pwm_40)
            self._batch_pwm_40.clear()

    def set_servo_angles_batch(self, angles: Dict[int, float]) -> None:
        """
        Set multiple servo angles in a single batch operation.
        This is the preferred method for updating multiple servos.

        Args:
            angles: Dictionary mapping channel -> angle
                   e.g., {0: 90, 1: 45, 16: 120}
        """
        for channel, angle in angles.items():
            self.queue_servo_angle(channel, angle)
        self.flush_batch()

    def relax(self) -> None:
        """
        Relax all servos by disabling PWM output.
        Servos will enter a relaxed state and can be moved manually.
        """
        logger.info("Relaxing all servos")

        # Build batch update to relax all servos
        # PWM value 4096 disables the output
        relax_value = (4096, 4096)

        # Relax all 16 channels on each controller
        batch_40 = {i: relax_value for i in range(16)}
        batch_41 = {i: relax_value for i in range(16)}

        self.pwm_40.set_pwm_batch(batch_40)
        self.pwm_41.set_pwm_batch(batch_41)

    def close(self) -> None:
        """Close I2C connections and cleanup resources."""
        logger.info("Closing servo controllers")
        self.pwm_40.close()
        self.pwm_41.close()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup resources."""
        self.close()
        return False


# Main program logic follows:
if __name__ == '__main__':
    from logger import setup_logger
    import time

    setup_logger(log_level='INFO')
    logger = get_logger()

    logger.info("Servo calibration mode")
    logger.info("Servos will rotate to calibration angles")
    logger.info("Keep the program running when installing servos")
    logger.info("Press Ctrl-C to end the program")

    with Servo() as servo:
        try:
            # Build calibration angles
            calibration_angles = {}
            for i in range(32):
                if i in [10, 13, 31]:
                    calibration_angles[i] = 10
                elif i in [18, 21, 27]:
                    calibration_angles[i] = 170
                else:
                    calibration_angles[i] = 90

            # Apply calibration angles using batch operation
            logger.info(f"Setting {len(calibration_angles)} servos to calibration positions")
            servo.set_servo_angles_batch(calibration_angles)

            # Hold position
            while True:
                time.sleep(1)

        except KeyboardInterrupt:
            logger.info("Keyboard interrupt - relaxing servos")
            servo.relax()
            logger.info("Program ended")
