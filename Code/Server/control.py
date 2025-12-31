# -*- coding: utf-8 -*-
"""
Hexapod Robot Control System - Refactored for Pi 4.
Handles kinematics, gait generation, and movement coordination.
"""

import time
import math
import threading
import queue
import numpy as np
from gpiozero import OutputDevice
from typing import List, Tuple, Optional

from config import get_config
from logger import get_logger
from pid import Incremental_PID
from command import COMMAND as cmd
from imu import IMU
from servo import Servo
from camera import Camera
from ultrasonic import Ultrasonic
from face_recognition import FaceRecognition
from autonomous import AutonomousController

logger = get_logger()


class Control:
    """Main control system for hexapod robot movement and balance."""

    def __init__(self):
        """Initialize control system and hardware."""
        self.config = get_config()
        logger.info("Initializing control system")

        # Hardware initialization
        self.imu = IMU()
        self.servo = Servo()
        self.servo_power_disable = OutputDevice(
            self.config.servo.power_disable_pin
        )
        self.servo_power_disable.off()

        # Control state
        self.movement_flag = 0x01
        self.relaxation_flag = False
        self.status_flag = 0x00

        # PID controller for balance
        self.pid_controller = Incremental_PID(0.500, 0.00, 0.0025)

        # Thread-safe command queue (FIXES RACE CONDITION)
        self.command_queue: queue.Queue = queue.Queue(maxsize=10)
        self.shutdown_event = threading.Event()

        # Thread-safe timeout tracking (FIXES RACE CONDITION)
        self._timeout = 0.0
        self._timeout_lock = threading.Lock()

        # Robot geometry from config
        self.body_height = self.config.geometry.body_height
        self.body_points = self.config.geometry.body_points.copy()

        # Calibration data
        self.calibration_leg_positions = self.read_from_txt('point')
        self.leg_positions = [
            list(self.config.geometry.default_leg_position) for _ in range(6)
        ]
        self.calibration_angles = [[0, 0, 0] for _ in range(6)]
        self.current_angles = [[90, 0, 0] for _ in range(6)]

        # Pre-allocated numpy arrays for coordinate calculations (OPTIMIZATION)
        self._rotation_x = np.zeros((3, 3))
        self._rotation_y = np.zeros((3, 3))
        self._rotation_z = np.zeros((3, 3))

        # Non-blocking gait support (OPTIMIZATION - Phase 2)
        self.gait_thread: Optional[threading.Thread] = None
        self.gait_cancel_event = threading.Event()
        self.gait_lock = threading.Lock()

        # Autonomous mode components (optional, initialized on demand)
        self.camera: Optional[Camera] = None
        self.ultrasonic: Optional[Ultrasonic] = None
        self.face_recognition: Optional[FaceRecognition] = None
        self.autonomous_controller: Optional[AutonomousController] = None

        # Initialize robot position
        self.calibrate()
        self.set_leg_angles()

        # Start condition monitor thread
        self.condition_thread = threading.Thread(
            target=self.condition_monitor,
            name="ConditionMonitor",
            daemon=True
        )

        logger.info("Control system initialized")

    @property
    def timeout(self) -> float:
        """Thread-safe timeout getter."""
        with self._timeout_lock:
            return self._timeout

    @timeout.setter
    def timeout(self, value: float):
        """Thread-safe timeout setter."""
        with self._timeout_lock:
            self._timeout = value

    def command_input(self, command: str) -> bool:
        """
        Queue a command for processing.
        Used by autonomous controller and external interfaces.

        Args:
            command: Command string (e.g., "CMD_MOVE#30#0#0#0#tripod")

        Returns:
            True if command queued successfully, False if queue full
        """
        try:
            command_parts = command.split('#')
            self.command_queue.put(command_parts, block=False)
            logger.debug(f"Queued command: {command_parts[0]}")
            return True
        except queue.Full:
            logger.warning(f"Command queue full, dropping command: {command}")
            return False
        except Exception as e:
            logger.error(f"Error queuing command: {e}")
            return False

    def read_from_txt(self, filename: str) -> List[List[int]]:
        """Read calibration data from file."""
        try:
            with open(filename + ".txt", "r") as file:
                lines = file.readlines()
                data = [list(map(int, line.strip().split("\t"))) for line in lines]
            logger.debug(f"Loaded calibration from {filename}.txt")
            return data
        except FileNotFoundError:
            logger.warning(f"Calibration file {filename}.txt not found, using defaults")
            return [[0, 0, 0] for _ in range(6)]

    def save_to_txt(self, data: List[List[int]], filename: str):
        """Save calibration data to file."""
        with open(filename + '.txt', 'w') as file:
            for row in data:
                file.write('\t'.join(map(str, row)) + '\n')
        logger.info(f"Saved calibration to {filename}.txt")

    def coordinate_to_angle(
        self,
        x: float,
        y: float,
        z: float,
        l1: float = 33,
        l2: float = 90,
        l3: float = 110
    ) -> Tuple[int, int, int]:
        """
        Inverse kinematics: Convert 3D coordinates to servo angles.

        Args:
            x, y, z: Target leg endpoint coordinates
            l1, l2, l3: Link lengths

        Returns:
            Tuple of (angle_a, angle_b, angle_c) in degrees
        """
        a = math.pi / 2 - math.atan2(z, y)
        x_3 = 0
        x_4 = l1 * math.sin(a)
        x_5 = l1 * math.cos(a)
        l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + (x - x_3) ** 2)
        w = self.restrict_value((x - x_3) / l23, -1, 1)
        v = self.restrict_value((l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23), -1, 1)
        u = self.restrict_value((l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2), -1, 1)
        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round(u, 2))
        return round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))

    def angle_to_coordinate(
        self,
        a: float,
        b: float,
        c: float,
        l1: float = 33,
        l2: float = 90,
        l3: float = 110
    ) -> Tuple[int, int, int]:
        """
        Forward kinematics: Convert servo angles to 3D coordinates.

        Args:
            a, b, c: Servo angles in degrees
            l1, l2, l3: Link lengths

        Returns:
            Tuple of (x, y, z) coordinates
        """
        a = math.pi / 180 * a
        b = math.pi / 180 * b
        c = math.pi / 180 * c
        x = round(l3 * math.sin(b + c) + l2 * math.sin(b))
        y = round(l3 * math.sin(a) * math.cos(b + c) + l2 * math.sin(a) * math.cos(b) + l1 * math.sin(a))
        z = round(l3 * math.cos(a) * math.cos(b + c) + l2 * math.cos(a) * math.cos(b) + l1 * math.cos(a))
        return x, y, z

    def calibrate(self):
        """Calculate calibration angle offsets for all legs."""
        self.leg_positions = [
            list(self.config.geometry.default_leg_position) for _ in range(6)
        ]

        for i in range(6):
            self.calibration_angles[i][0], self.calibration_angles[i][1], self.calibration_angles[i][2] = \
                self.coordinate_to_angle(
                    -self.calibration_leg_positions[i][2],
                    self.calibration_leg_positions[i][0],
                    self.calibration_leg_positions[i][1]
                )

        for i in range(6):
            self.current_angles[i][0], self.current_angles[i][1], self.current_angles[i][2] = \
                self.coordinate_to_angle(
                    -self.leg_positions[i][2],
                    self.leg_positions[i][0],
                    self.leg_positions[i][1]
                )

        for i in range(6):
            self.calibration_angles[i][0] -= self.current_angles[i][0]
            self.calibration_angles[i][1] -= self.current_angles[i][1]
            self.calibration_angles[i][2] -= self.current_angles[i][2]

        logger.debug("Calibration complete")

    def auto_calibrate_with_imu(
        self,
        target_height_mm: float = 80.0,
        max_iterations: int = 10,
        tolerance_degrees: float = 2.0
    ) -> bool:
        """
        Auto-calibrate robot stance using IMU feedback.
        Adjusts individual leg heights to achieve level stance, then lifts body.

        Args:
            target_height_mm: Target body height in mm (default 80mm = 3.15 inches)
            max_iterations: Maximum adjustment iterations
            tolerance_degrees: Acceptable pitch/roll tolerance in degrees

        Returns:
            True if calibration successful, False otherwise
        """
        logger.info("Starting IMU auto-calibration")
        logger.info(f"Target body height: {target_height_mm}mm ({target_height_mm/25.4:.2f} inches)")

        # Step 1: Lower robot to ground with legs extended
        logger.info("Step 1: Lowering to ground contact...")
        self.body_height = -10.0  # Start very low to ensure contact
        self.leg_positions = [
            [140.0, 0.0, 0.0] for _ in range(6)  # Extended position
        ]
        self.transform_coordinates(self.body_points)
        self.set_leg_angles()
        time.sleep(2.0)  # Allow robot to settle

        # Step 2: Calibrate IMU to get baseline
        logger.info("Step 2: Calibrating IMU baseline...")
        self.imu.error_accel_data, self.imu.error_gyro_data = \
            self.imu._calibrate_sensor()
        time.sleep(1.0)

        # Step 3: Iteratively adjust leg heights to level the robot
        logger.info("Step 3: Leveling robot using IMU feedback...")
        leg_height_offsets = [0.0] * 6  # Individual height adjustments per leg

        for iteration in range(max_iterations):
            # Read current orientation
            pitch, roll, yaw = self.imu.update_imu_state()
            logger.info(f"Iteration {iteration + 1}: Pitch={pitch:.2f}°, Roll={roll:.2f}°")

            # Check if level enough
            if abs(pitch) < tolerance_degrees and abs(roll) < tolerance_degrees:
                logger.info(f"✓ Robot leveled! Pitch={pitch:.2f}°, Roll={roll:.2f}°")
                break

            # Calculate corrections (proportional control)
            # Pitch affects legs 0,1,2 vs 3,4,5 (front vs back)
            # Roll affects legs 0,5 vs 2,3 (right vs left)

            pitch_correction = pitch * 0.3  # Scale factor
            roll_correction = roll * 0.3

            # Apply corrections to individual legs
            # Leg numbering: 0=FR, 1=MR, 2=RR, 3=RL, 4=ML, 5=FL
            # Front legs (0, 1): adjust for pitch
            leg_height_offsets[0] -= pitch_correction + roll_correction
            leg_height_offsets[1] -= pitch_correction

            # Right rear (2): adjust for pitch and roll
            leg_height_offsets[2] -= pitch_correction - roll_correction

            # Left rear (3): adjust for pitch and roll
            leg_height_offsets[3] += pitch_correction - roll_correction

            # Middle left (4): adjust for pitch
            leg_height_offsets[4] += pitch_correction

            # Front left (5): adjust for pitch and roll
            leg_height_offsets[5] += pitch_correction + roll_correction

            # Clamp offsets to reasonable range
            for i in range(6):
                leg_height_offsets[i] = self.restrict_value(
                    leg_height_offsets[i], -20.0, 20.0
                )

            # Apply offsets to leg positions
            for i in range(6):
                self.leg_positions[i][2] = leg_height_offsets[i]

            # Update servo angles
            self.transform_coordinates(self.body_points)
            self.set_leg_angles()
            time.sleep(0.5)  # Allow robot to settle

        else:
            logger.warning(f"Failed to level robot within {max_iterations} iterations")
            return False

        # Step 4: Store calibration offsets
        logger.info("Step 4: Storing leg calibration offsets...")
        for i in range(6):
            logger.debug(f"Leg {i}: Height offset = {leg_height_offsets[i]:.2f}mm")

        # Step 5: Lift body to target height
        logger.info(f"Step 5: Lifting body to {target_height_mm}mm...")

        # Gradually lift to avoid sudden movements
        current_height = self.body_height
        steps = 20
        height_increment = (target_height_mm - current_height) / steps

        for step in range(steps):
            self.body_height = current_height + (height_increment * (step + 1))

            # Maintain leg offsets while lifting
            for i in range(6):
                self.leg_positions[i][2] = leg_height_offsets[i]

            self.transform_coordinates(self.body_points)
            self.set_leg_angles()
            time.sleep(0.05)  # Smooth lifting motion

        # Final position
        self.body_height = target_height_mm
        for i in range(6):
            self.leg_positions[i][2] = leg_height_offsets[i]

        self.transform_coordinates(self.body_points)
        self.set_leg_angles()
        time.sleep(1.0)

        # Verify final stance
        pitch, roll, yaw = self.imu.update_imu_state()
        logger.info(f"✓ Auto-calibration complete!")
        logger.info(f"  Body height: {self.body_height}mm ({self.body_height/25.4:.2f} inches)")
        logger.info(f"  Final orientation: Pitch={pitch:.2f}°, Roll={roll:.2f}°")
        logger.info(f"  Robot is ready to move!")

        return True

    def set_standing_height(self, height_inches: float = 3.0):
        """
        Set robot standing height in inches.
        Convenience method for user-friendly height adjustment.

        Args:
            height_inches: Desired height in inches (default 3.0)
        """
        height_mm = height_inches * 25.4
        logger.info(f"Setting standing height to {height_inches} inches ({height_mm:.1f}mm)")

        # Gradually adjust height
        current_height = self.body_height
        steps = 20
        height_increment = (height_mm - current_height) / steps

        for step in range(steps):
            self.body_height = current_height + (height_increment * (step + 1))
            self.transform_coordinates(self.body_points)
            self.set_leg_angles()
            time.sleep(0.05)

        self.body_height = height_mm
        self.transform_coordinates(self.body_points)
        self.set_leg_angles()

        logger.info(f"✓ Standing height set to {height_inches} inches")

    def set_leg_angles(self):
        """
        Calculate and set servo angles for all legs based on current positions.
        Applies calibration offsets.
        """
        if not self.check_point_validity():
            logger.warning("Coordinate point out of active range")
            return

        # Calculate angles for all legs
        for i in range(6):
            self.current_angles[i][0], self.current_angles[i][1], self.current_angles[i][2] = \
                self.coordinate_to_angle(
                    -self.leg_positions[i][2],
                    self.leg_positions[i][0],
                    self.leg_positions[i][1]
                )

        # Apply calibration and constraints
        for i in range(3):
            self.current_angles[i][0] = self.restrict_value(
                self.current_angles[i][0] + self.calibration_angles[i][0], 0, 180
            )
            self.current_angles[i][1] = self.restrict_value(
                90 - (self.current_angles[i][1] + self.calibration_angles[i][1]), 0, 180
            )
            self.current_angles[i][2] = self.restrict_value(
                self.current_angles[i][2] + self.calibration_angles[i][2], 0, 180
            )
            self.current_angles[i + 3][0] = self.restrict_value(
                self.current_angles[i + 3][0] + self.calibration_angles[i + 3][0], 0, 180
            )
            self.current_angles[i + 3][1] = self.restrict_value(
                90 + self.current_angles[i + 3][1] + self.calibration_angles[i + 3][1], 0, 180
            )
            self.current_angles[i + 3][2] = self.restrict_value(
                180 - (self.current_angles[i + 3][2] + self.calibration_angles[i + 3][2]), 0, 180
            )

        # Set servo angles (18 servos total)
        # TODO: Batch these into 2 I2C transactions in Phase 2
        self.servo.set_servo_angle(15, self.current_angles[0][0])  # Leg 1
        self.servo.set_servo_angle(14, self.current_angles[0][1])
        self.servo.set_servo_angle(13, self.current_angles[0][2])
        self.servo.set_servo_angle(12, self.current_angles[1][0])  # Leg 2
        self.servo.set_servo_angle(11, self.current_angles[1][1])
        self.servo.set_servo_angle(10, self.current_angles[1][2])
        self.servo.set_servo_angle(9, self.current_angles[2][0])   # Leg 3
        self.servo.set_servo_angle(8, self.current_angles[2][1])
        self.servo.set_servo_angle(31, self.current_angles[2][2])
        self.servo.set_servo_angle(16, self.current_angles[5][0])  # Leg 6
        self.servo.set_servo_angle(17, self.current_angles[5][1])
        self.servo.set_servo_angle(18, self.current_angles[5][2])
        self.servo.set_servo_angle(19, self.current_angles[4][0])  # Leg 5
        self.servo.set_servo_angle(20, self.current_angles[4][1])
        self.servo.set_servo_angle(21, self.current_angles[4][2])
        self.servo.set_servo_angle(22, self.current_angles[3][0])  # Leg 4
        self.servo.set_servo_angle(23, self.current_angles[3][1])
        self.servo.set_servo_angle(27, self.current_angles[3][2])

    def check_point_validity(self) -> bool:
        """Check if leg positions are within valid range."""
        for i in range(6):
            leg_length = math.sqrt(
                self.leg_positions[i][0] ** 2 +
                self.leg_positions[i][1] ** 2 +
                self.leg_positions[i][2] ** 2
            )
            if leg_length > 248 or leg_length < 90:
                return False
        return True

    def condition_monitor(self):
        """
        Main control loop - processes commands from queue.
        CRITICAL FIX: No longer spins at 100% CPU!
        """
        logger.info("Condition monitor started")

        while not self.shutdown_event.is_set():
            # Check for timeout (auto-relax after 10s inactivity)
            current_time = time.time()
            if (current_time - self.timeout) > self.config.performance.command_timeout:
                if self.timeout != 0:
                    logger.debug("Command timeout - relaxing servos")
                    self.timeout = current_time
                    self.relax(True)
                    self.status_flag = 0x00

            # Wait for command with timeout (FIXES 100% CPU SPIN!)
            try:
                command_parts = self.command_queue.get(
                    timeout=self.config.performance.condition_monitor_timeout
                )
            except queue.Empty:
                # No command received, loop continues (uses minimal CPU)
                continue

            # Process command
            try:
                self._process_command(command_parts)
            except Exception as e:
                logger.error(f"Command processing error: {e}", exc_info=True)
            finally:
                self.command_queue.task_done()

        logger.info("Condition monitor stopped")

    def _process_command(self, command_parts: List[str]):
        """Process a single command from the queue."""
        if not command_parts or command_parts[0] == '':
            return

        cmd_type = command_parts[0]

        if cmd_type == cmd.CMD_POSITION and len(command_parts) == 4:
            if self.status_flag != 0x01:
                self.relax(False)
            x = self.restrict_value(int(command_parts[1]), *self.config.movement.position_limit_x)
            y = self.restrict_value(int(command_parts[2]), *self.config.movement.position_limit_y)
            z = self.restrict_value(int(command_parts[3]), *self.config.movement.position_limit_z)
            self.move_position(x, y, z)
            self.status_flag = 0x01

        elif cmd_type == cmd.CMD_ATTITUDE and len(command_parts) == 4:
            if self.status_flag != 0x02:
                self.relax(False)
            roll = self.restrict_value(int(command_parts[1]), -15, 15)
            pitch = self.restrict_value(int(command_parts[2]), -15, 15)
            yaw = self.restrict_value(int(command_parts[3]), -15, 15)
            points = self.calculate_posture_balance(roll, pitch, yaw)
            self.transform_coordinates(points)
            self.set_leg_angles()
            self.status_flag = 0x02

        elif cmd_type == cmd.CMD_MOVE and len(command_parts) == 6:
            if command_parts[2] == "0" and command_parts[3] == "0":
                self.run_gait(command_parts)
            else:
                if self.status_flag != 0x03:
                    self.relax(False)
                self.run_gait(command_parts)
                self.status_flag = 0x03

        elif cmd_type == cmd.CMD_BALANCE and len(command_parts) == 2:
            if command_parts[1] == "1":
                if self.status_flag != 0x04:
                    self.relax(False)
                self.status_flag = 0x04
                self.imu6050()

        elif cmd_type == cmd.CMD_CALIBRATION:
            self.timeout = 0
            self._handle_calibration_command(command_parts)

        elif cmd_type == cmd.CMD_AUTO_CALIBRATE:
            # Auto-calibrate with IMU
            # Format: CMD_AUTO_CALIBRATE#height_mm (optional)
            target_height = 80.0  # Default 80mm = 3.15 inches
            if len(command_parts) >= 2:
                try:
                    target_height = float(command_parts[1])
                except ValueError:
                    logger.warning(f"Invalid height value: {command_parts[1]}, using default")

            logger.info(f"Starting auto-calibration with target height: {target_height}mm")
            success = self.auto_calibrate_with_imu(target_height_mm=target_height)
            if success:
                logger.info("Auto-calibration completed successfully")
                self.status_flag = 0x01
            else:
                logger.error("Auto-calibration failed")

        elif cmd_type == cmd.CMD_SET_HEIGHT:
            # Set standing height
            # Format: CMD_SET_HEIGHT#height_inches
            if len(command_parts) >= 2:
                try:
                    height_inches = float(command_parts[1])
                    self.set_standing_height(height_inches)
                    logger.info(f"Height set to {height_inches} inches")
                except ValueError:
                    logger.warning(f"Invalid height value: {command_parts[1]}")

        # Autonomous mode commands
        elif cmd_type == cmd.CMD_START_EXPLORING:
            self._ensure_autonomous_initialized()
            if self.autonomous_controller:
                from autonomous import AutonomousState
                self.autonomous_controller.start(AutonomousState.EXPLORING)
                logger.info("Started exploration mode")

        elif cmd_type == cmd.CMD_START_PATROL:
            self._ensure_autonomous_initialized()
            if self.autonomous_controller:
                from autonomous import AutonomousState
                self.autonomous_controller.start(AutonomousState.PATROL)
                logger.info("Started patrol mode")

        elif cmd_type == cmd.CMD_STOP_AUTONOMOUS:
            if self.autonomous_controller:
                self.autonomous_controller.stop()
                logger.info("Stopped autonomous mode")

        elif cmd_type == cmd.CMD_ADD_PATROL_WAYPOINT:
            # Format: CMD_ADD_PATROL_WAYPOINT#command
            if len(command_parts) >= 2:
                waypoint_cmd = command_parts[1]
                self._ensure_autonomous_initialized()
                if self.autonomous_controller:
                    self.autonomous_controller.add_patrol_waypoint(waypoint_cmd)
                    logger.info(f"Added patrol waypoint: {waypoint_cmd}")

        elif cmd_type == cmd.CMD_CAPTURE_FACE_SAMPLE:
            # Format: CMD_CAPTURE_FACE_SAMPLE#name
            if len(command_parts) >= 2:
                name = command_parts[1]
                self._ensure_autonomous_initialized()
                if self.camera and self.face_recognition:
                    frame = self.camera.capture_frame()
                    if frame is not None:
                        face_id = self.face_recognition.add_face_sample(frame, name)
                        if face_id:
                            logger.info(f"Captured face sample for {name} (ID: {face_id})")
                        else:
                            logger.warning("No face detected in frame")

        elif cmd_type == cmd.CMD_TRAIN_FACES:
            self._ensure_autonomous_initialized()
            if self.face_recognition:
                success = self.face_recognition.train_recognizer()
                if success:
                    logger.info("Face recognizer trained successfully")
                else:
                    logger.warning("Face training failed")

        elif cmd_type == cmd.CMD_LIST_KNOWN_FACES:
            self._ensure_autonomous_initialized()
            if self.face_recognition:
                known_faces = self.face_recognition.get_known_faces()
                logger.info(f"Known faces: {len(known_faces)}")
                for face_id, info in known_faces.items():
                    logger.info(f"  ID {face_id}: {info['name']} - {info['sample_count']} samples")

        elif cmd_type == cmd.CMD_DELETE_FACE:
            # Format: CMD_DELETE_FACE#face_id
            if len(command_parts) >= 2:
                try:
                    face_id = int(command_parts[1])
                    self._ensure_autonomous_initialized()
                    if self.face_recognition:
                        success = self.face_recognition.delete_face(face_id)
                        if success:
                            logger.info(f"Deleted face ID {face_id}")
                        else:
                            logger.warning(f"Face ID {face_id} not found")
                except ValueError:
                    logger.warning(f"Invalid face ID: {command_parts[1]}")

    def _ensure_autonomous_initialized(self):
        """Initialize autonomous components if not already done."""
        if self.camera is None:
            try:
                logger.info("Initializing camera for autonomous mode")
                self.camera = Camera()
                self.camera.start_image()
            except Exception as e:
                logger.error(f"Failed to initialize camera: {e}")

        if self.ultrasonic is None:
            try:
                logger.info("Initializing ultrasonic sensor for autonomous mode")
                self.ultrasonic = Ultrasonic()
            except Exception as e:
                logger.error(f"Failed to initialize ultrasonic sensor: {e}")

        if self.face_recognition is None:
            try:
                logger.info("Initializing face recognition system")
                self.face_recognition = FaceRecognition()
            except Exception as e:
                logger.error(f"Failed to initialize face recognition: {e}")

        if self.autonomous_controller is None:
            if self.camera and self.ultrasonic and self.face_recognition:
                try:
                    logger.info("Initializing autonomous controller")
                    self.autonomous_controller = AutonomousController(
                        control=self,
                        face_recognition=self.face_recognition,
                        camera=self.camera,
                        sonic_sensor=self.ultrasonic
                    )

                    # Set up event callbacks
                    def on_unknown_face(face):
                        logger.warning(f"Unknown face detected! Confidence: {face['confidence']:.1f}")

                    def on_known_face(face):
                        logger.info(f"Known face detected: {face['name']}")

                    self.autonomous_controller.on_unknown_face_callback = on_unknown_face
                    self.autonomous_controller.on_known_face_callback = on_known_face

                except Exception as e:
                    logger.error(f"Failed to initialize autonomous controller: {e}")

    def _handle_calibration_command(self, command_parts: List[str]):
        """Handle calibration-specific commands."""
        self.calibrate()
        self.set_leg_angles()

        if len(command_parts) < 2:
            return

        leg_map = {
            "one": 0, "two": 1, "three": 2,
            "four": 3, "five": 4, "six": 5
        }

        leg_name = command_parts[1]
        if leg_name == "save":
            self.save_to_txt(self.calibration_leg_positions, 'point')
        elif leg_name in leg_map and len(command_parts) >= 5:
            leg_idx = leg_map[leg_name]
            self.calibration_leg_positions[leg_idx][0] = int(command_parts[2])
            self.calibration_leg_positions[leg_idx][1] = int(command_parts[3])
            self.calibration_leg_positions[leg_idx][2] = int(command_parts[4])
            self.calibrate()
            self.set_leg_angles()
            logger.info(f"Calibrated leg {leg_name}")

    def relax(self, flag: bool):
        """Set servos to relaxed or active state."""
        if flag:
            self.servo.relax()
            logger.debug("Servos relaxed")
        else:
            self.set_leg_angles()
            logger.debug("Servos active")

    def transform_coordinates(self, points: List[List[float]]):
        """
        Transform foot positions to leg coordinate frames.
        Applies rotation and offset for each leg.
        """
        # Leg angles in radians (pre-calculated for efficiency)
        angles = [54, 0, -54, -126, 180, 126]
        offsets_x = [-94, -85, -94, -94, -85, -94]

        for i in range(6):
            angle_rad = math.radians(angles[i])
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)

            self.leg_positions[i][0] = points[i][0] * cos_a + points[i][1] * sin_a + offsets_x[i]
            self.leg_positions[i][1] = -points[i][0] * sin_a + points[i][1] * cos_a
            self.leg_positions[i][2] = points[i][2] - 14

    def restrict_value(self, value: float, min_value: float, max_value: float) -> float:
        """Clamp value to range."""
        return max(min_value, min(max_value, value))

    def map_value(
        self,
        value: float,
        from_low: float,
        from_high: float,
        to_low: float,
        to_high: float
    ) -> float:
        """Map value from one range to another."""
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

    def move_position(self, x: float, y: float, z: float):
        """
        Move robot body to specified position offset.
        Uses shallow copy for efficiency (OPTIMIZATION).
        """
        # Shallow copy sufficient here - we're modifying elements
        points = [row[:] for row in self.body_points]

        for i in range(6):
            points[i][0] = self.body_points[i][0] - x
            points[i][1] = self.body_points[i][1] - y
            points[i][2] = -30 - z
            self.body_height = points[i][2]
            self.body_points[i][2] = points[i][2]

        self.transform_coordinates(points)
        self.set_leg_angles()

    def calculate_posture_balance(
        self,
        roll: float,
        pitch: float,
        yaw: float
    ) -> List[List[float]]:
        """
        Calculate foot positions for given body attitude.
        Uses pre-allocated arrays for efficiency (OPTIMIZATION).
        """
        position = np.array([0.0, 0.0, self.body_height])
        rpy = np.array([roll, pitch, yaw]) * math.pi / 180
        roll_angle, pitch_angle, yaw_angle = rpy[0], rpy[1], rpy[2]

        # Use pre-allocated rotation matrices
        cos_p, sin_p = math.cos(pitch_angle), math.sin(pitch_angle)
        self._rotation_x[0, :] = [1, 0, 0]
        self._rotation_x[1, :] = [0, cos_p, -sin_p]
        self._rotation_x[2, :] = [0, sin_p, cos_p]

        cos_r, sin_r = math.cos(roll_angle), math.sin(roll_angle)
        self._rotation_y[0, :] = [cos_r, 0, -sin_r]
        self._rotation_y[1, :] = [0, 1, 0]
        self._rotation_y[2, :] = [sin_r, 0, cos_r]

        cos_y, sin_y = math.cos(yaw_angle), math.sin(yaw_angle)
        self._rotation_z[0, :] = [cos_y, -sin_y, 0]
        self._rotation_z[1, :] = [sin_y, cos_y, 0]
        self._rotation_z[2, :] = [0, 0, 1]

        rotation_matrix = self._rotation_x @ self._rotation_y @ self._rotation_z

        footpoint_structure = np.array([
            [137.1, 189.4, 0],
            [225, 0, 0],
            [137.1, -189.4, 0],
            [-137.1, -189.4, 0],
            [-225, 0, 0],
            [-137.1, 189.4, 0]
        ]).T

        foot_positions = []
        for i in range(6):
            ab = position + rotation_matrix @ footpoint_structure[:, i]
            foot_positions.append(ab.tolist())

        return foot_positions

    def imu6050(self):
        """
        IMU-based balance control loop.
        Note: Still blocks - making this non-blocking is Phase 2 optimization.
        """
        logger.info("Starting IMU balance mode")
        points = self.calculate_posture_balance(0, 0, 0)
        self.transform_coordinates(points)
        self.set_leg_angles()

        time.sleep(2)
        self.imu.Error_value_accel_data, self.imu.Error_value_gyro_data = \
            self.imu.calculate_average_sensor_data()
        time.sleep(1)

        while not self.shutdown_event.is_set():
            # Check if new command received
            try:
                # Non-blocking check
                new_cmd = self.command_queue.get_nowait()
                if new_cmd and new_cmd[0] != "":
                    # Put it back for main loop to process
                    self.command_queue.put(new_cmd)
                    break
            except queue.Empty:
                pass

            time.sleep(0.02)
            roll, pitch, yaw = self.imu.update_imu_state()
            roll = self.pid_controller.pid_calculate(roll)
            pitch = self.pid_controller.pid_calculate(pitch)
            points = self.calculate_posture_balance(roll, pitch, 0)
            self.transform_coordinates(points)
            self.set_leg_angles()

        logger.info("IMU balance mode ended")

    def cancel_gait(self) -> None:
        """
        Cancel any ongoing gait animation.
        Signals gait thread to stop and waits for completion.
        """
        with self.gait_lock:
            if self.gait_thread is not None and self.gait_thread.is_alive():
                logger.debug("Cancelling ongoing gait")
                self.gait_cancel_event.set()

                # Wait for gait thread to finish (with timeout)
                self.gait_thread.join(timeout=0.5)

                if self.gait_thread.is_alive():
                    logger.warning("Gait thread did not stop within timeout")

                self.gait_thread = None
                self.gait_cancel_event.clear()

    def run_gait(self, data: List[str], Z: int = 40, F: int = 64):
        """
        Execute walking gait pattern in non-blocking mode.
        Cancels any ongoing gait and starts new one in separate thread.

        Args:
            data: Command array ['CMD_MOVE', gait, x, y, speed, angle]
            Z: Step height
            F: Frame count (speed)
        """
        # Check if non-blocking gaits are enabled
        if not self.config.movement.enable_non_blocking_gait:
            # Fallback to blocking execution
            self._run_gait_blocking(data, Z, F)
            return

        # Cancel any ongoing gait
        self.cancel_gait()

        # Start new gait in separate thread
        with self.gait_lock:
            self.gait_thread = threading.Thread(
                target=self._run_gait_blocking,
                args=(data, Z, F),
                name="GaitAnimation",
                daemon=True
            )
            self.gait_thread.start()
            logger.debug("Started non-blocking gait animation")

    def _run_gait_blocking(self, data: List[str], Z: int = 40, F: int = 64):
        """
        Internal method: Execute gait pattern (blocking).
        Called either directly (blocking mode) or in separate thread (non-blocking).

        Args:
            data: Command array ['CMD_MOVE', gait, x, y, speed, angle]
            Z: Step height
            F: Frame count (speed)
        """
        gait = data[1]
        x = self.restrict_value(int(data[2]), -35, 35)
        y = self.restrict_value(int(data[3]), -35, 35)

        if gait == "1":
            F = round(self.map_value(int(data[4]), 2, 10, 126, 22))
        else:
            F = round(self.map_value(int(data[4]), 2, 10, 171, 45))

        angle = int(data[5])
        z = Z / F
        delay = self.config.movement.gait_frame_delay

        # Shallow copy for efficiency
        points = [row[:] for row in self.body_points]
        xy = [[0, 0] for _ in range(6)]

        angle_rad = math.radians(angle)
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)

        for i in range(6):
            xy[i][0] = ((points[i][0] * cos_a + points[i][1] * sin_a - points[i][0]) + x) / F
            xy[i][1] = ((-points[i][0] * sin_a + points[i][1] * cos_a - points[i][1]) + y) / F

        if x == 0 and y == 0 and angle == 0:
            self.transform_coordinates(points)
            self.set_leg_angles()
            return

        # Gait execution (tripod or wave pattern)
        if gait == "1":
            self._execute_tripod_gait(points, xy, F, Z, z, delay)
        elif gait == "2":
            self._execute_wave_gait(points, xy, F, z, delay)

    def _execute_tripod_gait(self, points, xy, F, Z, z, delay):
        """Execute tripod gait pattern (legs 1,3,5 alternate with 2,4,6)."""
        for j in range(F):
            # Check for shutdown or gait cancellation
            if self.shutdown_event.is_set() or self.gait_cancel_event.is_set():
                logger.debug("Tripod gait interrupted")
                break

            for i in range(3):
                if j < (F / 8):
                    points[2 * i][0] -= 4 * xy[2 * i][0]
                    points[2 * i][1] -= 4 * xy[2 * i][1]
                    points[2 * i + 1][0] += 8 * xy[2 * i + 1][0]
                    points[2 * i + 1][1] += 8 * xy[2 * i + 1][1]
                    points[2 * i + 1][2] = Z + self.body_height
                elif j < (F / 4):
                    points[2 * i][0] -= 4 * xy[2 * i][0]
                    points[2 * i][1] -= 4 * xy[2 * i][1]
                    points[2 * i + 1][2] -= z * 8
                elif j < (3 * F / 8):
                    points[2 * i][2] += z * 8
                    points[2 * i + 1][0] -= 4 * xy[2 * i + 1][0]
                    points[2 * i + 1][1] -= 4 * xy[2 * i + 1][1]
                elif j < (5 * F / 8):
                    points[2 * i][0] += 8 * xy[2 * i][0]
                    points[2 * i][1] += 8 * xy[2 * i][1]
                    points[2 * i + 1][0] -= 4 * xy[2 * i + 1][0]
                    points[2 * i + 1][1] -= 4 * xy[2 * i + 1][1]
                elif j < (3 * F / 4):
                    points[2 * i][2] -= z * 8
                    points[2 * i + 1][0] -= 4 * xy[2 * i + 1][0]
                    points[2 * i + 1][1] -= 4 * xy[2 * i + 1][1]
                elif j < (7 * F / 8):
                    points[2 * i][0] -= 4 * xy[2 * i][0]
                    points[2 * i][1] -= 4 * xy[2 * i][1]
                    points[2 * i + 1][2] += z * 8
                else:  # j < F
                    points[2 * i][0] -= 4 * xy[2 * i][0]
                    points[2 * i][1] -= 4 * xy[2 * i][1]
                    points[2 * i + 1][0] += 8 * xy[2 * i + 1][0]
                    points[2 * i + 1][1] += 8 * xy[2 * i + 1][1]

            self.transform_coordinates(points)
            self.set_leg_angles()
            time.sleep(delay)

    def _execute_wave_gait(self, points, xy, F, z, delay):
        """Execute wave gait pattern (legs move sequentially)."""
        number = [5, 2, 1, 0, 3, 4]

        for i in range(6):
            # Check for shutdown or gait cancellation
            if self.shutdown_event.is_set() or self.gait_cancel_event.is_set():
                logger.debug("Wave gait interrupted")
                break

            for j in range(int(F / 6)):
                for k in range(6):
                    if number[i] == k:
                        if j < int(F / 18):
                            points[k][2] += 18 * z
                        elif j < int(F / 9):
                            points[k][0] += 30 * xy[k][0]
                            points[k][1] += 30 * xy[k][1]
                        elif j < int(F / 6):
                            points[k][2] -= 18 * z
                    else:
                        points[k][0] -= 2 * xy[k][0]
                        points[k][1] -= 2 * xy[k][1]

                self.transform_coordinates(points)
                self.set_leg_angles()
                time.sleep(delay)


if __name__ == '__main__':
    from logger import setup_from_config

    config = get_config()
    setup_from_config(config)

    logger.info("Testing control system")
    control = Control()
    control.condition_thread.start()

    try:
        # Test command
        control.command_queue.put([cmd.CMD_POSITION, "0", "0", "0"])
        time.sleep(2)
    except KeyboardInterrupt:
        logger.info("Test interrupted")
    finally:
        control.shutdown_event.set()
        control.condition_thread.join(timeout=5)

    logger.info("Control system test complete")
