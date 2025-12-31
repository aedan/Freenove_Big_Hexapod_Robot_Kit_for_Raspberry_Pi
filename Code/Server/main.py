# -*- coding: utf-8 -*-
"""
Main entry point for Freenove Hexapod Robot Server.
Supports both GUI (PyQt5) and headless operation with proper signal handling.
"""

import os
import sys
import getopt
import signal
import threading
from typing import Optional

from Thread import ManagedThread
from ui_server import Ui_server
from PyQt5.QtCore import QCoreApplication
from PyQt5.QtWidgets import QApplication, QMainWindow
from server import Server
from config import get_config
from logger import get_logger, setup_from_config

# Initialize configuration and logging
config = get_config()
setup_from_config(config)
logger = get_logger()


class MyWindow(QMainWindow, Ui_server):
    """Main window handling GUI and server lifecycle."""

    def __init__(self):
        """Initialize window with optional GUI and server."""
        self.user_ui = True
        self.start_tcp = False
        self.server = Server()

        # Thread management
        self.video_thread: Optional[ManagedThread] = None
        self.instruction_thread: Optional[ManagedThread] = None
        self.shutdown_event = threading.Event()

        # Parse command line options
        self.parseOpt()

        # Setup GUI if enabled
        if self.user_ui:
            self.app = QApplication(sys.argv)
            super(MyWindow, self).__init__()
            self.setupUi(self)
            self.pushButton_On_And_Off.clicked.connect(self.on_and_off_server)
            self.on_and_off_server()

        # Start TCP server if requested
        if self.start_tcp:
            self._start_tcp_server()

    def parseOpt(self):
        """Parse command line options."""
        try:
            self.opts, self.args = getopt.getopt(sys.argv[1:], "tn")
            for o, a in self.opts:
                if o in ('-t'):
                    logger.info("TCP server mode enabled via command line")
                    self.start_tcp = True
                elif o in ('-n'):
                    logger.info("Headless mode enabled (no GUI)")
                    self.user_ui = False
        except getopt.GetoptError as e:
            logger.error(f"Invalid command line option: {e}")
            sys.exit(2)

    def _start_tcp_server(self):
        """Start TCP server and communication threads."""
        try:
            self.server.start_server()
            self.server.tcp_flag = True

            # Create managed threads
            self.video_thread = ManagedThread(
                target=self.server.transmit_video,
                name="VideoTransmit"
            ).start()

            self.instruction_thread = ManagedThread(
                target=self.server.receive_commands,
                name="CommandReceive"
            ).start()

            logger.info("TCP server started successfully")

            # Update GUI if present
            if self.user_ui:
                self.pushButton_On_And_Off.setText('Off')
                self.states.setText('On')

        except Exception as e:
            logger.error(f"Failed to start TCP server: {e}", exc_info=True)
            raise

    def _stop_tcp_server(self):
        """Stop TCP server and communication threads."""
        logger.info("Stopping TCP server")
        self.server.tcp_flag = False

        # Stop threads gracefully
        if self.video_thread is not None:
            if not self.video_thread.stop(timeout=5.0):
                logger.warning("Video thread did not stop within timeout")

        if self.instruction_thread is not None:
            if not self.instruction_thread.stop(timeout=5.0):
                logger.warning("Instruction thread did not stop within timeout")

        # Stop server
        try:
            self.server.stop_server()
            logger.info("TCP server stopped successfully")
        except Exception as e:
            logger.error(f"Error stopping server: {e}")

    def on_and_off_server(self):
        """Toggle server on/off from GUI button."""
        if self.pushButton_On_And_Off.text() == 'On':
            # Start server
            self.pushButton_On_And_Off.setText('Off')
            self.states.setText('On')
            self._start_tcp_server()
        else:
            # Stop server
            self.pushButton_On_And_Off.setText('On')
            self.states.setText('Off')
            self._stop_tcp_server()

    def closeEvent(self, event):
        """Handle window close event with proper cleanup."""
        logger.info("Close event received - cleaning up")
        self.close_application()

        if self.user_ui:
            QCoreApplication.instance().quit()

        event.accept()

    def close_application(self):
        """Gracefully shutdown all components."""
        logger.info("Shutting down application")
        self.shutdown_event.set()

        # Stop TCP server if running
        if self.server.tcp_flag:
            self._stop_tcp_server()

        # Cleanup server resources
        try:
            self.server.__exit__(None, None, None)
        except Exception as e:
            logger.error(f"Error during server cleanup: {e}")

    def run(self):
        """Main run loop for headless mode."""
        if self.user_ui:
            # GUI mode
            self.show()
            return self.app.exec_()
        else:
            # Headless mode - wait for shutdown signal
            logger.info("Headless mode - waiting for shutdown signal (Ctrl+C)")
            try:
                self.shutdown_event.wait()
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received")
            finally:
                self.close_application()
            return 0


# Global reference for signal handlers
_window_instance: Optional[MyWindow] = None


def signal_handler(signum, frame):
    """
    Handle shutdown signals (SIGINT, SIGTERM) gracefully.

    Args:
        signum: Signal number
        frame: Current stack frame
    """
    signal_name = signal.Signals(signum).name
    logger.info(f"Received signal {signal_name} - initiating graceful shutdown")

    if _window_instance is not None:
        _window_instance.shutdown_event.set()
        _window_instance.close_application()

    sys.exit(0)


def setup_signal_handlers():
    """Register signal handlers for graceful shutdown."""
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    logger.info("Signal handlers registered (SIGINT, SIGTERM)")


if __name__ == '__main__':
    try:
        # Setup signal handlers
        setup_signal_handlers()

        # Create and run application
        logger.info("Starting Hexapod Robot Server")
        _window_instance = MyWindow()
        exit_code = _window_instance.run()

        logger.info("Application exited normally")
        sys.exit(exit_code)

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt in main - shutting down")
        if _window_instance is not None:
            _window_instance.close_application()
        sys.exit(0)

    except Exception as e:
        logger.exception(f"Fatal error in main: {e}")
        sys.exit(1)
