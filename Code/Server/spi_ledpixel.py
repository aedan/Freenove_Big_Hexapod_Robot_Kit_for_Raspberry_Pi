"""
SPI LED Pixel driver for WS2812B RGB LEDs.
Optimized for Pi 4 with 12MHz SPI speed for smoother animations.
"""

import spidev
import numpy
from typing import List, Optional

from config import get_config
from logger import get_logger

logger = get_logger()


class Freenove_SPI_LedPixel(object):
    """
    WS2812B LED controller using SPI interface.
    Supports RGB color control and various animation effects.
    """

    def __init__(
        self,
        count: int = 8,
        bright: int = 255,
        sequence: str = 'GRB',
        bus: int = 0,
        device: int = 0
    ):
        """
        Initialize LED pixel controller.

        Args:
            count: Number of LEDs in strip
            bright: Default brightness (0-255)
            sequence: Color sequence (RGB, GRB, etc.)
            bus: SPI bus number
            device: SPI device number
        """
        self.config = get_config()

        # Initialize LED parameters
        self.set_led_type(sequence)
        self.set_led_count(count)
        self.set_led_brightness(bright)

        # Initialize SPI connection
        self.led_begin(bus, device)

        # Set all LEDs to off
        self.set_all_led_color(0, 0, 0)

    def led_begin(self, bus: int = 0, device: int = 0) -> None:
        """
        Initialize SPI connection.

        Args:
            bus: SPI bus number
            device: SPI device number
        """
        self.bus = bus
        self.device = device

        try:
            # Initialize SPI device
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.device)
            self.spi.mode = 0

            # Set SPI speed - optimized for Pi 4
            # Using 12 MHz for faster LED updates
            self.spi.max_speed_hz = self.config.led.spi_speed_hz
            logger.info(
                f"SPI LED initialized on bus {bus}, device {device} "
                f"at {self.config.led.spi_speed_hz / 1e6:.1f} MHz"
            )

            self.led_init_state = 1

        except OSError as e:
            logger.error(f"SPI initialization failed: {e}")
            logger.error("Check /boot/firmware/config.txt configuration")

            if self.bus == 0:
                logger.error(
                    "Enable SPI in 'sudo raspi-config' -> Interface Options, "
                    "or ensure 'dtparam=spi=on' in config.txt"
                )
            else:
                logger.error(
                    f"Add 'dtoverlay=spi{self.bus}-2cs' to /boot/firmware/config.txt "
                    f"to enable SPI{self.bus}"
                )

            self.led_init_state = 0

    def check_spi_state(self) -> int:
        """Return the current SPI initialization state."""
        return self.led_init_state

    def spi_gpio_info(self) -> None:
        """Print GPIO pin information for the specified SPI bus."""
        gpio_info = {
            0: "SPI0-MOSI: GPIO10(WS2812-PIN)  SPI0-MISO: GPIO9  SPI0-SCLK: GPIO11  SPI0-CE0: GPIO8  SPI0-CE1: GPIO7",
            1: "SPI1-MOSI: GPIO20(WS2812-PIN)  SPI1-MISO: GPIO19  SPI1-SCLK: GPIO21  SPI1-CE0: GPIO18  SPI1-CE1: GPIO17  SPI0-CE1: GPIO16",
            2: "SPI2-MOSI: GPIO41(WS2812-PIN)  SPI2-MISO: GPIO40  SPI2-SCLK: GPIO42  SPI2-CE0: GPIO43  SPI2-CE1: GPIO44  SPI2-CE1: GPIO45",
            3: "SPI3-MOSI: GPIO2(WS2812-PIN)  SPI3-MISO: GPIO1  SPI3-SCLK: GPIO3  SPI3-CE0: GPIO0  SPI3-CE1: GPIO24",
            4: "SPI4-MOSI: GPIO6(WS2812-PIN)  SPI4-MISO: GPIO5  SPI4-SCLK: GPIO7  SPI4-CE0: GPIO4  SPI4-CE1: GPIO25",
            5: "SPI5-MOSI: GPIO14(WS2812-PIN)  SPI5-MISO: GPIO13  SPI5-SCLK: GPIO15  SPI5-CE0: GPIO12  SPI5-CE1: GPIO26",
            6: "SPI6-MOSI: GPIO20(WS2812-PIN)  SPI6-MISO: GPIO19  SPI6-SCLK: GPIO21  SPI6-CE0: GPIO18  SPI6-CE1: GPIO27"
        }
        if self.bus in gpio_info:
            logger.info(gpio_info[self.bus])
        else:
            logger.warning(f"Unknown SPI bus: {self.bus}")

    def led_close(self) -> None:
        """Turn off all LEDs and close SPI connection."""
        try:
            self.set_all_led_rgb([0, 0, 0])
            self.spi.close()
            logger.info("SPI LED closed")
        except Exception as e:
            logger.warning(f"Error closing SPI: {e}")

    def set_led_count(self, count: int) -> None:
        """
        Set the number of LEDs.

        Args:
            count: Number of LEDs
        """
        self.led_count = count
        # Initialize color arrays
        self.led_color = [0, 0, 0] * self.led_count
        self.led_original_color = [0, 0, 0] * self.led_count

    def get_led_count(self) -> int:
        """Return the number of LEDs."""
        return self.led_count

    def set_led_type(self, rgb_type: str) -> int:
        """
        Set the LED color sequence (RGB, GRB, etc.).

        Args:
            rgb_type: Color sequence string

        Returns:
            Index of sequence or -1 if invalid
        """
        try:
            led_type = ['RGB', 'RBG', 'GRB', 'GBR', 'BRG', 'BGR']
            led_type_offset = [0x06, 0x09, 0x12, 0x21, 0x18, 0x24]
            index = led_type.index(rgb_type)
            self.led_red_offset = (led_type_offset[index] >> 4) & 0x03
            self.led_green_offset = (led_type_offset[index] >> 2) & 0x03
            self.led_blue_offset = (led_type_offset[index] >> 0) & 0x03
            return index
        except ValueError:
            # Default to GRB
            self.led_red_offset = 1
            self.led_green_offset = 0
            self.led_blue_offset = 2
            logger.warning(f"Invalid LED type '{rgb_type}', using GRB")
            return -1

    def set_led_brightness(self, brightness: int) -> None:
        """
        Set the brightness of all LEDs.

        Args:
            brightness: Brightness value (0-255)
        """
        self.led_brightness = max(0, min(255, brightness))
        for i in range(self.get_led_count()):
            self.set_led_rgb_data(i, self.led_original_color)

    def set_ledpixel(self, index: int, r: int, g: int, b: int) -> None:
        """
        Set the color of a specific LED.

        Args:
            index: LED index
            r: Red value (0-255)
            g: Green value (0-255)
            b: Blue value (0-255)
        """
        p = [0, 0, 0]
        p[self.led_red_offset] = round(r * self.led_brightness / 255)
        p[self.led_green_offset] = round(g * self.led_brightness / 255)
        p[self.led_blue_offset] = round(b * self.led_brightness / 255)

        self.led_original_color[index * 3 + self.led_red_offset] = r
        self.led_original_color[index * 3 + self.led_green_offset] = g
        self.led_original_color[index * 3 + self.led_blue_offset] = b

        for i in range(3):
            self.led_color[index * 3 + i] = p[i]

    def set_led_color_data(self, index: int, r: int, g: int, b: int) -> None:
        """Set the color data of a specific LED."""
        self.set_ledpixel(index, r, g, b)

    def set_led_rgb_data(self, index: int, color: List[int]) -> None:
        """Set the RGB data of a specific LED."""
        self.set_ledpixel(index, color[0], color[1], color[2])

    def set_led_color(self, index: int, r: int, g: int, b: int) -> None:
        """Set the color of a specific LED and update the display."""
        self.set_ledpixel(index, r, g, b)
        self.show()

    def set_led_rgb(self, index: int, color: List[int]) -> None:
        """Set the RGB color of a specific LED and update the display."""
        self.set_led_rgb_data(index, color)
        self.show()

    def set_all_led_color_data(self, r: int, g: int, b: int) -> None:
        """Set the color data of all LEDs."""
        for i in range(self.get_led_count()):
            self.set_led_color_data(i, r, g, b)

    def set_all_led_rgb_data(self, color: List[int]) -> None:
        """Set the RGB data of all LEDs."""
        for i in range(self.get_led_count()):
            self.set_led_rgb_data(i, color)

    def set_all_led_color(self, r: int, g: int, b: int) -> None:
        """Set the color of all LEDs and update the display."""
        for i in range(self.get_led_count()):
            self.set_led_color_data(i, r, g, b)
        self.show()

    def set_all_led_rgb(self, color: List[int]) -> None:
        """Set the RGB color of all LEDs and update the display."""
        for i in range(self.get_led_count()):
            self.set_led_rgb_data(i, color)
        self.show()

    def write_ws2812_numpy8(self) -> None:
        """
        Convert color data to WS2812 format and send via SPI.
        Uses 8-bit encoding for accurate timing.
        """
        # Convert data into one-dimensional array
        d = numpy.array(self.led_color).ravel()

        # Each RGB color has 8 bits, each represented by a uint8 type data
        tx = numpy.zeros(len(d) * 8, dtype=numpy.uint8)

        # Convert each bit to SPI data
        # T0H=1,T0L=7, T1H=5,T1L=3
        # 0b11111000 = T1 (0.78125us), 0b10000000 = T0 (0.15625us)
        for ibit in range(8):
            tx[7 - ibit::8] = ((d >> ibit) & 1) * 0x78 + 0x80

        if self.led_init_state != 0:
            try:
                # Send at configured SPI speed (12 MHz for Pi 4)
                # Speed is now set via max_speed_hz in led_begin()
                self.spi.xfer(tx.tolist())
            except OSError as e:
                logger.error(f"SPI transfer error: {e}")

    def write_ws2812_numpy4(self) -> None:
        """
        Convert color data to WS2812 format using 4-bit encoding.
        Faster but less precise than 8-bit mode.
        """
        d = numpy.array(self.led_color).ravel()
        tx = numpy.zeros(len(d) * 4, dtype=numpy.uint8)

        for ibit in range(4):
            tx[3 - ibit::4] = (
                ((d >> (2 * ibit + 1)) & 1) * 0x60 +
                ((d >> (2 * ibit + 0)) & 1) * 0x06 + 0x88
            )

        if self.led_init_state != 0:
            try:
                self.spi.xfer(tx.tolist())
            except OSError as e:
                logger.error(f"SPI transfer error: {e}")

    def show(self, mode: int = 1) -> None:
        """
        Update the display with the current color data.

        Args:
            mode: 1 for 8-bit mode, 0 for 4-bit mode
        """
        if mode == 1:
            write_ws2812 = self.write_ws2812_numpy8
        else:
            write_ws2812 = self.write_ws2812_numpy4
        write_ws2812()

    def wheel(self, pos: int) -> List[int]:
        """
        Generate a color based on position in color wheel.

        Args:
            pos: Position (0-255)

        Returns:
            [R, G, B] color values
        """
        if pos < 85:
            return [(255 - pos * 3), (pos * 3), 0]
        elif pos < 170:
            pos = pos - 85
            return [0, (255 - pos * 3), (pos * 3)]
        else:
            pos = pos - 170
            return [(pos * 3), 0, (255 - pos * 3)]

    def hsv2rgb(self, h: int, s: int, v: int) -> List[int]:
        """
        Convert HSV to RGB.

        Args:
            h: Hue (0-360)
            s: Saturation (0-100)
            v: Value (0-100)

        Returns:
            [R, G, B] color values
        """
        h = h % 360
        rgb_max = round(v * 2.55)
        rgb_min = round(rgb_max * (100 - s) / 100)
        i = round(h / 60)
        diff = round(h % 60)
        rgb_adj = round((rgb_max - rgb_min) * diff / 60)

        if i == 0:
            r, g, b = rgb_max, rgb_min + rgb_adj, rgb_min
        elif i == 1:
            r, g, b = rgb_max - rgb_adj, rgb_max, rgb_min
        elif i == 2:
            r, g, b = rgb_min, rgb_max, rgb_min + rgb_adj
        elif i == 3:
            r, g, b = rgb_min, rgb_max - rgb_adj, rgb_max
        elif i == 4:
            r, g, b = rgb_min + rgb_adj, rgb_min, rgb_max
        else:
            r, g, b = rgb_max, rgb_min, rgb_max - rgb_adj

        return [r, g, b]

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup resources."""
        self.led_close()
        return False


if __name__ == '__main__':
    import time
    import os
    from logger import setup_logger

    setup_logger(log_level='INFO')

    logger.info(f"spidev version: {spidev.__version__}")
    logger.info("Available SPI devices:")
    os.system("ls /dev/spi*")

    # Create LED controller instance
    with Freenove_SPI_LedPixel(8, 255) as led:
        try:
            if led.check_spi_state() != 0:
                # Set number of LEDs
                led.set_led_count(8)

                # Test basic colors
                logger.info("Testing red...")
                led.set_all_led_color_data(255, 0, 0)
                led.show()
                time.sleep(0.5)

                logger.info("Testing green...")
                led.set_all_led_rgb_data([0, 255, 0])
                led.show()
                time.sleep(0.5)

                logger.info("Testing blue...")
                led.set_all_led_color(0, 0, 255)
                time.sleep(0.5)

                logger.info("Testing cyan...")
                led.set_all_led_rgb([0, 255, 255])
                time.sleep(0.5)

                # Brightness fade test
                logger.info("Testing brightness fade...")
                led.set_led_count(12)
                led.set_all_led_color_data(255, 255, 0)

                # Fade in
                for i in range(255):
                    led.set_led_brightness(i)
                    led.show()
                    time.sleep(0.005)

                # Fade out
                for i in range(255):
                    led.set_led_brightness(255 - i)
                    led.show()
                    time.sleep(0.005)

                # Color wheel animation
                logger.info("Running color wheel animation (Ctrl+C to stop)...")
                led.set_led_brightness(20)

                while True:
                    for j in range(255):
                        for i in range(led.led_count):
                            color = led.wheel(
                                (round(i * 255 / led.led_count) + j) % 256
                            )
                            led.set_led_rgb_data(i, color)
                        led.show()
                        time.sleep(0.002)
            else:
                logger.error("SPI initialization failed")

        except KeyboardInterrupt:
            logger.info("Animation stopped by user")
