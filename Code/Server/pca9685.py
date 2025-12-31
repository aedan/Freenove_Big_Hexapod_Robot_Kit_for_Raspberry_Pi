#!/usr/bin/python
"""
Raspberry Pi PCA9685 16-Channel PWM Servo Driver
Optimized for Pi 4 with batch I2C operations and proper resource management.
"""

import time
import math
import smbus
from typing import List, Tuple, Dict
from logger import get_logger

logger = get_logger()

# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PCA9685:
    """PCA9685 PWM driver with batch operations support."""

    # Registers/etc.
    __SUBADR1            = 0x02
    __SUBADR2            = 0x03
    __SUBADR3            = 0x04
    __MODE1              = 0x00
    __PRESCALE           = 0xFE
    __LED0_ON_L          = 0x06
    __LED0_ON_H          = 0x07
    __LED0_OFF_L         = 0x08
    __LED0_OFF_H         = 0x09
    __ALLLED_ON_L        = 0xFA
    __ALLLED_ON_H        = 0xFB
    __ALLLED_OFF_L       = 0xFC
    __ALLLED_OFF_H       = 0xFD

    def __init__(self, address: int = 0x40, bus_number: int = 1):
        """
        Initialize PCA9685.

        Args:
            address: I2C address of the PCA9685
            bus_number: I2C bus number (default 1 for Pi 4)
        """
        try:
            self.bus = smbus.SMBus(bus_number)
            self.address = address
            self.write(self.__MODE1, 0x00)
            logger.debug(f"PCA9685 initialized at address 0x{address:02x}")
        except Exception as e:
            logger.error(f"Failed to initialize PCA9685 at 0x{address:02x}: {e}")
            raise

    def write(self, reg: int, value: int) -> None:
        """Writes an 8-bit value to the specified register/address."""
        try:
            self.bus.write_byte_data(self.address, reg, value)
        except OSError as e:
            logger.error(f"I2C write error at 0x{self.address:02x} reg 0x{reg:02x}: {e}")
            raise

    def read(self, reg: int) -> int:
        """Read an unsigned byte from the I2C device."""
        try:
            result = self.bus.read_byte_data(self.address, reg)
            return result
        except OSError as e:
            logger.error(f"I2C read error at 0x{self.address:02x} reg 0x{reg:02x}: {e}")
            raise

    def set_pwm_freq(self, freq: float) -> None:
        """
        Sets the PWM frequency.

        Args:
            freq: Frequency in Hz (typically 50 for servos)
        """
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10        # sleep
        self.write(self.__MODE1, newmode)        # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)
        logger.debug(f"PWM frequency set to {freq} Hz")

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        """
        Sets a single PWM channel.

        Args:
            channel: PWM channel (0-15)
            on: ON time (0-4095)
            off: OFF time (0-4095)
        """
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)

    def set_pwm_batch(self, channels_values: Dict[int, Tuple[int, int]]) -> None:
        """
        Set multiple PWM channels efficiently using batch I2C writes.
        Significantly faster than individual set_pwm() calls.

        Args:
            channels_values: Dictionary mapping channel -> (on, off) values
                           e.g., {0: (0, 2048), 1: (0, 3000)}
        """
        if not channels_values:
            return

        try:
            # Group consecutive channels for block writes
            # For now, write each channel's 4 bytes as a block
            for channel, (on, off) in sorted(channels_values.items()):
                if not (0 <= channel <= 15):
                    logger.warning(f"Invalid channel {channel}, skipping")
                    continue

                # Write all 4 registers for this channel as a block
                reg_start = self.__LED0_ON_L + 4 * channel
                data = [
                    on & 0xFF,      # LED_ON_L
                    on >> 8,        # LED_ON_H
                    off & 0xFF,     # LED_OFF_L
                    off >> 8        # LED_OFF_H
                ]
                self.bus.write_i2c_block_data(self.address, reg_start, data)

            logger.debug(f"Batch updated {len(channels_values)} PWM channels")

        except OSError as e:
            logger.error(f"Batch I2C write error: {e}")
            raise

    def set_all_pwm(self, on: int, off: int) -> None:
        """
        Set all PWM channels to the same value.

        Args:
            on: ON time for all channels
            off: OFF time for all channels
        """
        self.write(self.__ALLLED_ON_L, on & 0xFF)
        self.write(self.__ALLLED_ON_H, on >> 8)
        self.write(self.__ALLLED_OFF_L, off & 0xFF)
        self.write(self.__ALLLED_OFF_H, off >> 8)

    def set_motor_pwm(self, channel: int, duty: int) -> None:
        """
        Sets the PWM duty cycle for a motor.

        Args:
            channel: Motor channel
            duty: Duty cycle value (0-4095)
        """
        self.set_pwm(channel, 0, duty)

    def set_servo_pulse(self, channel: int, pulse: float) -> None:
        """
        Sets the Servo Pulse. The PWM frequency must be 50Hz.

        Args:
            channel: Servo channel
            pulse: Pulse width in microseconds
        """
        pulse = pulse * 4096 / 20000  # PWM frequency is 50Hz, period is 20000us
        self.set_pwm(channel, 0, int(pulse))

    def close(self) -> None:
        """Close the I2C bus."""
        try:
            self.bus.close()
            logger.debug(f"I2C bus closed for PCA9685 at 0x{self.address:02x}")
        except Exception as e:
            logger.warning(f"Error closing I2C bus: {e}")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup resources."""
        self.close()
        return False


if __name__ == '__main__':
    pass
