# -*- coding: utf-8 -*-
"""
Hexapod Robot Server - Refactored for Pi 4 with proper resource management.
Handles video streaming and command processing with safe threading.
"""

import io
import time
import fcntl
import socket
import struct
import threading
from threading import Condition
from typing import Optional
from contextlib import contextmanager

from config import get_config
from logger import get_logger
from Thread import ManagedThread
from led import Led
from servo import Servo
from buzzer import Buzzer
from control import Control
from adc import ADC
from ultrasonic import Ultrasonic
from command import COMMAND as cmd
from camera import Camera

logger = get_logger()


class StreamingOutput(io.BufferedIOBase):
    """Thread-safe video frame buffer."""

    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class Server:
    """
    Main server class handling video streaming and command processing.
    Supports context manager for automatic cleanup.
    """

    def __init__(self):
        """Initialize server components and state."""
        self.config = get_config()

        # Initialize server state
        self.tcp_flag = False
        self.is_servo_relaxed = False

        # Initialize hardware components
        logger.info("Initializing hardware components")
        self.led_controller = Led()
        self.adc_sensor = ADC()
        self.servo_controller = Servo()
        self.buzzer_controller = Buzzer()
        self.control_system = Control()
        self.ultrasonic_sensor = Ultrasonic()
        self.camera_device = Camera()

        # Thread management
        self.led_thread: Optional[ManagedThread] = None
        self.ultrasonic_thread: Optional[ManagedThread] = None
        self.video_thread: Optional[ManagedThread] = None
        self.command_thread: Optional[ManagedThread] = None

        # Socket initialization (will be set in start_server)
        self.video_socket: Optional[socket.socket] = None
        self.command_socket: Optional[socket.socket] = None
        self.video_connection: Optional[socket.socket] = None
        self.command_connection: Optional[socket.socket] = None
        self.video_file: Optional[io.BufferedWriter] = None

        # Start control thread
        self.control_system.condition_thread.start()
        logger.info("Server initialized successfully")

    @contextmanager
    def _socket_context(self, sock_type: str = 'DGRAM'):
        """Context manager for temporary sockets."""
        if sock_type == 'DGRAM':
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            yield sock
        finally:
            try:
                sock.close()
            except Exception as e:
                logger.warning(f"Error closing socket: {e}")

    def get_interface_ip(self, interface: str = None) -> str:
        """
        Get IP address of network interface.

        Args:
            interface: Network interface name (default from config)

        Returns:
            IP address as string

        Raises:
            OSError: If interface not found or no IP assigned
        """
        if interface is None:
            interface = self.config.network.interface

        with self._socket_context('DGRAM') as sock:
            try:
                ip = socket.inet_ntoa(
                    fcntl.ioctl(
                        sock.fileno(),
                        0x8915,  # SIOCGIFADDR
                        struct.pack('256s', interface.encode()[:15])
                    )[20:24]
                )
                logger.info(f"Interface {interface} IP: {ip}")
                return ip
            except OSError as e:
                logger.error(f"Failed to get IP for interface {interface}: {e}")
                raise

    def start_server(self):
        """
        Start video and command server sockets.
        Creates listening sockets on configured ports.
        """
        try:
            host_ip = self.get_interface_ip()

            # Video socket
            self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.video_socket.bind((host_ip, self.config.network.video_port))
            self.video_socket.listen(1)
            logger.info(f"Video server listening on {host_ip}:{self.config.network.video_port}")

            # Command socket
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.command_socket.bind((host_ip, self.config.network.command_port))
            self.command_socket.listen(1)
            logger.info(f"Command server listening on {host_ip}:{self.config.network.command_port}")

        except Exception as e:
            logger.error(f"Failed to start server: {e}")
            self._cleanup_sockets()
            raise

    def _cleanup_sockets(self):
        """Clean up all socket resources."""
        sockets_to_close = [
            ('video_connection', self.video_connection),
            ('command_connection', self.command_connection),
            ('video_socket', self.video_socket),
            ('command_socket', self.command_socket)
        ]

        for name, sock in sockets_to_close:
            if sock is not None:
                try:
                    sock.close()
                    logger.debug(f"Closed {name}")
                except Exception as e:
                    logger.warning(f"Error closing {name}: {e}")

        # Close video file wrapper
        if self.video_file is not None:
            try:
                self.video_file.close()
                logger.debug("Closed video file wrapper")
            except Exception as e:
                logger.warning(f"Error closing video file: {e}")

    def stop_server(self):
        """Stop server and clean up resources."""
        logger.info("Stopping server")
        self.tcp_flag = False
        self._cleanup_sockets()

    def reset_server(self):
        """Reset server by stopping and restarting."""
        logger.info("Resetting server")
        self.stop_server()
        time.sleep(0.5)  # Brief pause
        self.start_server()

        # Restart threads
        self.video_thread = ManagedThread(
            target=self.transmit_video,
            name="VideoTransmit"
        ).start()

        self.command_thread = ManagedThread(
            target=self.receive_commands,
            name="CommandReceive"
        ).start()

    def send_data(self, connection: socket.socket, data: str):
        """
        Send data over socket connection.

        Args:
            connection: Socket to send data on
            data: String data to send
        """
        try:
            connection.send(data.encode('utf-8'))
        except Exception as e:
            logger.error(f"Failed to send data: {e}")

    def transmit_video(self, shutdown_event: Optional[threading.Event] = None):
        """
        Transmit video frames to connected client.
        Runs in dedicated thread.

        Args:
            shutdown_event: Event to signal shutdown
        """
        try:
            logger.info("Waiting for video client connection")
            self.video_connection, video_client_addr = self.video_socket.accept()
            logger.info(f"Video client connected: {video_client_addr}")

            # Enable TCP_NODELAY for low latency
            if self.config.network.enable_tcp_nodelay:
                self.video_connection.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1
                )
                logger.debug("TCP_NODELAY enabled for video")

            # Wrap socket in file-like object
            self.video_file = self.video_connection.makefile('wb')

        except Exception as e:
            logger.error(f"Video connection failed: {e}")
            return
        finally:
            # Close listening socket after accepting connection
            if self.video_socket:
                try:
                    self.video_socket.close()
                    self.video_socket = None
                except:
                    pass

        # Start camera stream
        try:
            self.camera_device.start_stream()
            logger.info("Camera stream started")
        except Exception as e:
            logger.error(f"Failed to start camera: {e}")
            return

        # Transmit frames
        frame_count = 0
        try:
            while shutdown_event is None or not shutdown_event.is_set():
                if not self.tcp_flag:
                    break

                try:
                    frame = self.camera_device.get_frame()
                    frame_length = len(frame)

                    # Send frame length then frame data
                    length_binary = struct.pack('<I', frame_length)
                    self.video_file.write(length_binary)
                    self.video_file.write(frame)
                    self.video_file.flush()

                    frame_count += 1
                    if frame_count % 100 == 0:
                        logger.debug(f"Transmitted {frame_count} frames")

                except Exception as e:
                    logger.error(f"Frame transmission error: {e}")
                    break

        finally:
            self.camera_device.stop_stream()
            logger.info(f"Video transmission ended. Total frames: {frame_count}")
            if self.video_file:
                self.video_file.close()

    def receive_commands(self, shutdown_event: Optional[threading.Event] = None):
        """
        Receive and process commands from connected client.
        Runs in dedicated thread.

        Args:
            shutdown_event: Event to signal shutdown
        """
        try:
            logger.info("Waiting for command client connection")
            self.command_connection, command_client_addr = self.command_socket.accept()
            logger.info(f"Command client connected: {command_client_addr}")

            # Enable TCP_NODELAY
            if self.config.network.enable_tcp_nodelay:
                self.command_connection.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1
                )
                logger.debug("TCP_NODELAY enabled for commands")

        except Exception as e:
            logger.error(f"Command connection failed: {e}")
            return
        finally:
            # Close listening socket
            if self.command_socket:
                try:
                    self.command_socket.close()
                    self.command_socket = None
                except:
                    pass

        # Command processing loop
        try:
            while shutdown_event is None or not shutdown_event.is_set():
                if not self.tcp_flag:
                    break

                try:
                    received_data = self.command_connection.recv(
                        self.config.network.buffer_size
                    ).decode('utf-8')
                except socket.timeout:
                    continue
                except (ConnectionResetError, BrokenPipeError) as e:
                    logger.warning(f"Connection lost: {e}")
                    if self.tcp_flag:
                        self.reset_server()
                    break
                except Exception as e:
                    logger.error(f"Receive error: {e}")
                    break

                # Check for disconnection
                if received_data == "":
                    logger.info("Client disconnected")
                    if self.tcp_flag:
                        self.reset_server()
                    break

                # Process commands
                self._process_commands(received_data)

        finally:
            # Cleanup threads
            self._stop_worker_threads()
            logger.info("Command processing ended")

    def _process_commands(self, received_data: str):
        """
        Parse and execute commands from received data.

        Args:
            received_data: Raw command string from client
        """
        command_array = received_data.split('\n')

        # Remove incomplete last command if present
        if command_array[-1] != "":
            command_array = command_array[:-1]

        for single_command in command_array:
            command_parts = single_command.split("#")

            if not command_parts or command_parts[0] == '':
                continue

            try:
                self._execute_command(command_parts)
            except Exception as e:
                logger.error(f"Command execution error: {e}", exc_info=True)

    def _execute_command(self, command_parts: list):
        """Execute a single parsed command."""
        cmd_type = command_parts[0]

        if cmd.CMD_DISCOVER == cmd_type:
            # Network discovery - respond with robot information
            robot_name = self.config.network.robot_name if hasattr(self.config.network, 'robot_name') else "HexapodRobot"
            version = "2.0"
            response = f"HEXAPOD:{robot_name}:{version}\n"
            self.send_data(self.command_connection, response)
            logger.debug(f"Responded to discovery request: {robot_name}")

        elif cmd.CMD_BUZZER == cmd_type:
            self.buzzer_controller.set_state(command_parts[1] == "1")

        elif cmd.CMD_POWER == cmd_type:
            try:
                battery_voltage = self.adc_sensor.read_battery_voltage()
                response = f"{cmd.CMD_POWER}#{battery_voltage[0]}#{battery_voltage[1]}\n"
                self.send_data(self.command_connection, response)

                # Low battery warning
                if battery_voltage[0] < 5.5 or battery_voltage[1] < 6:
                    logger.warning(f"Low battery: {battery_voltage}")
                    for _ in range(3):
                        self.buzzer_controller.set_state(True)
                        time.sleep(0.15)
                        self.buzzer_controller.set_state(False)
                        time.sleep(0.1)
            except Exception as e:
                logger.error(f"Battery check failed: {e}")

        elif cmd.CMD_LED == cmd_type or cmd.CMD_LED_MOD == cmd_type:
            # Stop existing LED thread if running
            if self.led_thread is not None and self.led_thread.is_alive():
                self.led_thread.stop(timeout=1.0)

            self.led_thread = ManagedThread(
                target=self.led_controller.process_light_command,
                args=(command_parts,),
                name="LEDControl"
            ).start()

        elif cmd.CMD_SONIC == cmd_type:
            distance = self.ultrasonic_sensor.get_distance()
            response = f"{cmd.CMD_SONIC}#{distance}\n"
            self.send_data(self.command_connection, response)

        elif cmd.CMD_HEAD == cmd_type:
            if len(command_parts) == 3:
                servo_id = int(command_parts[1])
                angle = int(command_parts[2])
                self.servo_controller.set_servo_angle(servo_id, angle)

        elif cmd.CMD_CAMERA == cmd_type:
            if len(command_parts) == 3:
                x = self.control_system.restrict_value(int(command_parts[1]), 50, 180)
                y = self.control_system.restrict_value(int(command_parts[2]), 0, 180)
                self.servo_controller.set_servo_angle(0, x)
                self.servo_controller.set_servo_angle(1, y)

        elif cmd.CMD_RELAX == cmd_type:
            self.is_servo_relaxed = not self.is_servo_relaxed
            self.control_system.relax(self.is_servo_relaxed)
            logger.info(f"Servos {'relaxed' if self.is_servo_relaxed else 'active'}")

        elif cmd.CMD_SERVOPOWER == cmd_type:
            if command_parts[1] == "0":
                self.control_system.servo_power_disable.on()
            else:
                self.control_system.servo_power_disable.off()

        else:
            # Pass to control system
            self.control_system.command_queue = command_parts
            self.control_system.timeout = time.time()

    def _stop_worker_threads(self):
        """Stop LED and ultrasonic worker threads."""
        if self.led_thread is not None:
            self.led_thread.stop(timeout=2.0)

        if self.ultrasonic_thread is not None:
            self.ultrasonic_thread.stop(timeout=2.0)

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup resources."""
        logger.info("Server context exit - cleaning up")
        self.stop_server()
        self._stop_worker_threads()
        return False


if __name__ == '__main__':
    # Example usage
    from logger import setup_from_config

    config = get_config()
    setup_from_config(config)

    logger.info("Starting hexapod server")

    with Server() as server:
        server.start_server()
        server.tcp_flag = True

        # Start threads
        video_thread = ManagedThread(
            target=server.transmit_video,
            name="VideoTransmit"
        ).start()

        command_thread = ManagedThread(
            target=server.receive_commands,
            name="CommandReceive"
        ).start()

        try:
            # Keep alive
            video_thread._thread.join()
            command_thread._thread.join()
        except KeyboardInterrupt:
            logger.info("Keyboard interrupt received")
        finally:
            video_thread.stop()
            command_thread.stop()

    logger.info("Server shutdown complete")
