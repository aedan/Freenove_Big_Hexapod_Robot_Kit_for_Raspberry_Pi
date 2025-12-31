"""
Autonomous behavior system for hexapod robot.
Implements exploration, patrol, and intruder detection modes.
"""

import threading
import time
import random
import numpy as np
from enum import Enum
from typing import Optional, List, Tuple, Dict
from datetime import datetime, timedelta

from logger import get_logger

logger = get_logger()


class AutonomousState(Enum):
    """Robot autonomous behavior states."""
    IDLE = "idle"
    EXPLORING = "exploring"
    PATROL = "patrol"
    INVESTIGATING = "investigating"
    ALERT = "alert"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"


class AutonomousController:
    """
    Controls autonomous behaviors including exploration, patrol, and intruder detection.
    Acts as a "kid exploring and learning" - curious, cautious, and protective.
    """

    def __init__(self, control, face_recognition, camera, sonic_sensor):
        """
        Initialize autonomous controller.

        Args:
            control: Main control system (for movement commands)
            face_recognition: Face recognition system
            camera: Camera system for capturing frames
            sonic_sensor: Ultrasonic distance sensor
        """
        self.control = control
        self.face_rec = face_recognition
        self.camera = camera
        self.sonic = sonic_sensor

        # State management
        self.state = AutonomousState.IDLE
        self.previous_state = AutonomousState.IDLE

        # Control thread
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.shutdown_event = threading.Event()

        # Behavior parameters
        self.obstacle_distance_threshold = 30.0  # cm
        self.safe_distance = 50.0  # cm
        self.investigation_distance = 20.0  # cm - move this close to unknown faces

        # Exploration parameters
        self.exploration_move_duration = 2.0  # seconds per move
        self.exploration_turn_probability = 0.3  # chance to turn randomly
        self.exploration_pause_duration = 1.0  # seconds to pause and scan

        # Patrol parameters
        self.patrol_waypoints: List[str] = []  # Movement commands for patrol route
        self.current_waypoint_index = 0
        self.patrol_pause_duration = 2.0  # seconds to pause at each waypoint

        # Face detection parameters
        self.face_check_interval = 1.0  # seconds between face checks
        self.last_face_check = time.time()
        self.unknown_face_alert_duration = 5.0  # seconds to alert for unknown
        self.alert_start_time: Optional[float] = None

        # Spatial memory (simple occupancy grid)
        self.spatial_memory = SpatialMemory(grid_size=20, cell_size=50)  # 20x20 grid, 50cm cells

        # Event callbacks
        self.on_unknown_face_callback = None
        self.on_known_face_callback = None
        self.on_obstacle_callback = None

        logger.info("Autonomous controller initialized")

    def start(self, mode: AutonomousState = AutonomousState.EXPLORING) -> bool:
        """
        Start autonomous operation.

        Args:
            mode: Initial autonomous state

        Returns:
            True if started successfully
        """
        if self.running:
            logger.warning("Autonomous mode already running")
            return False

        logger.info(f"Starting autonomous mode: {mode.value}")
        self.state = mode
        self.running = True
        self.shutdown_event.clear()

        # Start control thread
        self.thread = threading.Thread(
            target=self._autonomous_loop,
            name="AutonomousController",
            daemon=True
        )
        self.thread.start()

        return True

    def stop(self) -> None:
        """Stop autonomous operation."""
        if not self.running:
            return

        logger.info("Stopping autonomous mode")
        self.running = False
        self.shutdown_event.set()

        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None

        self.state = AutonomousState.IDLE
        logger.info("Autonomous mode stopped")

    def set_state(self, new_state: AutonomousState) -> None:
        """Change autonomous state."""
        if new_state != self.state:
            logger.info(f"State transition: {self.state.value} -> {new_state.value}")
            self.previous_state = self.state
            self.state = new_state

    def _autonomous_loop(self) -> None:
        """Main autonomous control loop."""
        logger.info("Autonomous control loop started")

        while self.running and not self.shutdown_event.is_set():
            try:
                # State machine
                if self.state == AutonomousState.EXPLORING:
                    self._exploration_behavior()
                elif self.state == AutonomousState.PATROL:
                    self._patrol_behavior()
                elif self.state == AutonomousState.INVESTIGATING:
                    self._investigation_behavior()
                elif self.state == AutonomousState.ALERT:
                    self._alert_behavior()
                elif self.state == AutonomousState.OBSTACLE_AVOIDANCE:
                    self._obstacle_avoidance_behavior()
                elif self.state == AutonomousState.IDLE:
                    time.sleep(0.5)

                # Check for faces periodically
                if time.time() - self.last_face_check >= self.face_check_interval:
                    self._check_for_faces()
                    self.last_face_check = time.time()

                time.sleep(0.1)  # Small sleep to prevent CPU spinning

            except Exception as e:
                logger.error(f"Error in autonomous loop: {e}", exc_info=True)
                time.sleep(1.0)

        logger.info("Autonomous control loop stopped")

    def _exploration_behavior(self) -> None:
        """
        Exploration mode: Wander around like a curious kid.
        Randomly walk, turn, pause to scan environment.
        """
        # Check for obstacles
        distance = self._get_distance()
        if distance < self.obstacle_distance_threshold:
            logger.debug(f"Obstacle detected at {distance:.1f}cm during exploration")
            self.set_state(AutonomousState.OBSTACLE_AVOIDANCE)
            return

        # Random behavior selection
        action = random.random()

        if action < self.exploration_turn_probability:
            # Random turn
            direction = random.choice(['left', 'right'])
            duration = random.uniform(0.5, 1.5)
            logger.debug(f"Exploration: Turning {direction} for {duration:.1f}s")

            if direction == 'left':
                self.control.command_input("CMD_MOVE#0#-30")
            else:
                self.control.command_input("CMD_MOVE#0#30")

            time.sleep(duration)
            self.control.command_input("CMD_MOVE#0#0")  # Stop

        else:
            # Move forward
            speed = random.randint(30, 60)
            logger.debug(f"Exploration: Moving forward at speed {speed}")
            self.control.command_input(f"CMD_MOVE#{speed}#0")
            time.sleep(self.exploration_move_duration)
            self.control.command_input("CMD_MOVE#0#0")  # Stop

        # Pause and scan
        logger.debug("Exploration: Pausing to scan environment")
        time.sleep(self.exploration_pause_duration)

        # Update spatial memory (simple: mark current position as visited)
        # Position estimation from IMU heading and movement
        self.spatial_memory.mark_visited(0, 0)  # Simplified for now

    def _patrol_behavior(self) -> None:
        """
        Patrol mode: Follow predefined waypoints, alert on unknown faces.
        More methodical than exploration.
        """
        if not self.patrol_waypoints:
            logger.warning("No patrol waypoints defined, switching to exploration")
            self.set_state(AutonomousState.EXPLORING)
            return

        # Check for obstacles
        distance = self._get_distance()
        if distance < self.obstacle_distance_threshold:
            logger.debug(f"Obstacle detected at {distance:.1f}cm during patrol")
            self.set_state(AutonomousState.OBSTACLE_AVOIDANCE)
            return

        # Execute current waypoint command
        waypoint_cmd = self.patrol_waypoints[self.current_waypoint_index]
        logger.debug(f"Patrol: Waypoint {self.current_waypoint_index}: {waypoint_cmd}")
        self.control.command_input(waypoint_cmd)

        # Wait at waypoint
        time.sleep(self.patrol_pause_duration)

        # Stop movement
        self.control.command_input("CMD_MOVE#0#0")

        # Move to next waypoint
        self.current_waypoint_index = (
            (self.current_waypoint_index + 1) % len(self.patrol_waypoints)
        )

    def _investigation_behavior(self) -> None:
        """
        Investigation mode: Face detected, move closer for better recognition.
        Cautious approach to verify identity.
        """
        # Get current frame and check for faces
        frame = self._get_camera_frame()
        if frame is None:
            logger.warning("Cannot get camera frame for investigation")
            self.set_state(self.previous_state)
            return

        results = self.face_rec.detect_and_recognize(frame)

        if not results:
            # Lost the face
            logger.info("Investigation: Lost face, returning to previous state")
            self.set_state(self.previous_state)
            return

        # Focus on largest face (closest)
        face = max(results, key=lambda f: f['rect'][2] * f['rect'][3])

        # Check distance
        distance = self._get_distance()

        if distance > self.investigation_distance:
            # Move closer
            logger.debug(f"Investigation: Moving closer (current distance: {distance:.1f}cm)")
            self.control.command_input("CMD_MOVE#20#0")  # Slow approach
            time.sleep(0.5)
        else:
            # Close enough, make determination
            self.control.command_input("CMD_MOVE#0#0")  # Stop

            if face['is_known']:
                logger.info(f"Investigation: Identified as {face['name']}")
                self._trigger_known_face_event(face)
                self.set_state(self.previous_state)
            else:
                logger.warning(f"Investigation: Unknown person detected!")
                self._trigger_unknown_face_event(face)
                self.set_state(AutonomousState.ALERT)

    def _alert_behavior(self) -> None:
        """
        Alert mode: Unknown person detected, sound alarm.
        """
        if self.alert_start_time is None:
            self.alert_start_time = time.time()
            logger.warning("ALERT: Unknown person detected!")

        # Sound buzzer alarm pattern
        self._sound_alarm_pattern()

        # Flash LEDs red
        self.control.command_input("CMD_LED#255#0#0")

        # Check if alert duration exceeded
        if time.time() - self.alert_start_time >= self.unknown_face_alert_duration:
            logger.info("Alert duration complete, returning to patrol")
            self.alert_start_time = None
            self.control.command_input("CMD_LED#0#0#0")  # Turn off LEDs
            self.set_state(AutonomousState.PATROL)

    def _obstacle_avoidance_behavior(self) -> None:
        """
        Obstacle avoidance: Detected obstacle, maneuver around it.
        Simple reactive behavior.
        """
        distance = self._get_distance()

        if distance >= self.safe_distance:
            # Clear, return to previous state
            logger.debug("Obstacle cleared, resuming previous state")
            self.set_state(self.previous_state)
            return

        # Stop
        self.control.command_input("CMD_MOVE#0#0")
        time.sleep(0.3)

        # Back up
        logger.debug("Obstacle avoidance: Backing up")
        self.control.command_input("CMD_MOVE#-30#0")
        time.sleep(1.0)
        self.control.command_input("CMD_MOVE#0#0")

        # Turn random direction
        direction = random.choice(['left', 'right'])
        logger.debug(f"Obstacle avoidance: Turning {direction}")

        if direction == 'left':
            self.control.command_input("CMD_MOVE#0#-40")
        else:
            self.control.command_input("CMD_MOVE#0#40")

        time.sleep(1.5)
        self.control.command_input("CMD_MOVE#0#0")

        # Mark obstacle in spatial memory
        self.spatial_memory.mark_obstacle(0, 0)  # Simplified

    def _check_for_faces(self) -> None:
        """Periodically check camera feed for faces."""
        if self.state in [AutonomousState.INVESTIGATING, AutonomousState.ALERT]:
            return  # Already handling faces

        frame = self._get_camera_frame()
        if frame is None:
            return

        results = self.face_rec.detect_and_recognize(frame)

        if results:
            face = results[0]  # Focus on first detected face

            if face['is_known']:
                logger.info(f"Detected known face: {face['name']}")
                self._trigger_known_face_event(face)
                # Brief acknowledgment chirp
                self._sound_chirp_pattern()
            else:
                logger.warning("Detected unknown face, investigating")
                self.set_state(AutonomousState.INVESTIGATING)

    def _get_distance(self) -> float:
        """Get distance from ultrasonic sensor."""
        try:
            # Request sonic reading
            distance = self.sonic.get_distance()
            return distance if distance > 0 else 200.0  # Max range if invalid
        except Exception as e:
            logger.error(f"Error reading distance sensor: {e}")
            return 200.0  # Assume clear

    def _get_camera_frame(self) -> Optional[np.ndarray]:
        """Get current camera frame."""
        try:
            # Get frame from camera as numpy array (BGR format)
            frame = self.camera.capture_frame()
            return frame
        except Exception as e:
            logger.error(f"Error getting camera frame: {e}")
            return None

    def _sound_alarm_pattern(self) -> None:
        """Sound buzzer alarm pattern for unknown person."""
        # Rapid beeping pattern
        self.control.command_input("CMD_BUZZER#1")
        time.sleep(0.2)
        self.control.command_input("CMD_BUZZER#0")
        time.sleep(0.2)

    def _sound_chirp_pattern(self) -> None:
        """Sound friendly chirp for known person."""
        # Short single beep
        self.control.command_input("CMD_BUZZER#1")
        time.sleep(0.1)
        self.control.command_input("CMD_BUZZER#0")

    def _trigger_unknown_face_event(self, face: Dict) -> None:
        """Trigger callback for unknown face detection."""
        if self.on_unknown_face_callback:
            try:
                self.on_unknown_face_callback(face)
            except Exception as e:
                logger.error(f"Error in unknown face callback: {e}")

    def _trigger_known_face_event(self, face: Dict) -> None:
        """Trigger callback for known face detection."""
        if self.on_known_face_callback:
            try:
                self.on_known_face_callback(face)
            except Exception as e:
                logger.error(f"Error in known face callback: {e}")

    def set_patrol_route(self, waypoints: List[str]) -> None:
        """
        Set patrol route waypoints.

        Args:
            waypoints: List of movement commands for patrol route
        """
        self.patrol_waypoints = waypoints
        self.current_waypoint_index = 0
        logger.info(f"Patrol route set: {len(waypoints)} waypoints")

    def add_patrol_waypoint(self, command: str) -> None:
        """Add a waypoint to patrol route."""
        self.patrol_waypoints.append(command)
        logger.info(f"Added patrol waypoint: {command}")


class SpatialMemory:
    """
    Simple spatial memory using occupancy grid.
    Tracks visited locations and obstacles.
    """

    def __init__(self, grid_size: int = 20, cell_size: float = 50.0):
        """
        Initialize spatial memory.

        Args:
            grid_size: Size of grid (grid_size x grid_size)
            cell_size: Size of each cell in cm
        """
        self.grid_size = grid_size
        self.cell_size = cell_size

        # Grid: 0 = unknown, 1 = free/visited, -1 = obstacle
        self.grid = np.zeros((grid_size, grid_size), dtype=np.int8)

        # Center position (robot starts here)
        self.center = (grid_size // 2, grid_size // 2)

        # Current position estimate (grid coordinates)
        self.current_pos = list(self.center)

        logger.debug(f"Spatial memory initialized: {grid_size}x{grid_size} grid, "
                    f"{cell_size}cm cells")

    def mark_visited(self, x: int, y: int) -> None:
        """Mark a grid cell as visited."""
        gx, gy = self._world_to_grid(x, y)
        if self._in_bounds(gx, gy):
            self.grid[gy, gx] = 1

    def mark_obstacle(self, x: int, y: int) -> None:
        """Mark a grid cell as obstacle."""
        gx, gy = self._world_to_grid(x, y)
        if self._in_bounds(gx, gy):
            self.grid[gy, gx] = -1

    def is_obstacle(self, x: int, y: int) -> bool:
        """Check if location has obstacle."""
        gx, gy = self._world_to_grid(x, y)
        if self._in_bounds(gx, gy):
            return self.grid[gy, gx] == -1
        return False

    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (cm) to grid coordinates."""
        gx = self.center[0] + int(x / self.cell_size)
        gy = self.center[1] + int(y / self.cell_size)
        return gx, gy

    def _in_bounds(self, gx: int, gy: int) -> bool:
        """Check if grid coordinates are in bounds."""
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size

    def get_exploration_score(self) -> float:
        """Get exploration completion percentage."""
        visited = np.sum(self.grid == 1)
        total = self.grid_size * self.grid_size
        return (visited / total) * 100.0


if __name__ == '__main__':
    """Test autonomous controller"""
    from logger import setup_logger

    setup_logger(log_level='INFO')
    logger.info("Autonomous controller test - placeholder mode")
