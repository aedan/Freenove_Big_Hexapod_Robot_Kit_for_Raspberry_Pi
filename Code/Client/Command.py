class COMMAND:
    CMD_MOVE = "CMD_MOVE"
    CMD_LED_MOD = "CMD_LED_MOD"
    CMD_LED = "CMD_LED"
    CMD_SONIC = "CMD_SONIC"
    CMD_BUZZER = "CMD_BUZZER"
    CMD_HEAD = "CMD_HEAD"
    CMD_BALANCE = "CMD_BALANCE"
    CMD_ATTITUDE = "CMD_ATTITUDE"
    CMD_POSITION = "CMD_POSITION"
    CMD_RELAX = "CMD_RELAX"
    CMD_POWER = "CMD_POWER"
    CMD_CALIBRATION = "CMD_CALIBRATION"
    CMD_CAMERA = "CMD_CAMERA"
    CMD_SERVOPOWER = "CMD_SERVOPOWER"
    CMD_AUTO_CALIBRATE = "CMD_AUTO_CALIBRATE"  # Auto-calibrate with IMU
    CMD_SET_HEIGHT = "CMD_SET_HEIGHT"  # Set standing height

    # Autonomous mode commands
    CMD_START_EXPLORING = "CMD_START_EXPLORING"  # Start exploration mode
    CMD_START_PATROL = "CMD_START_PATROL"  # Start patrol mode
    CMD_STOP_AUTONOMOUS = "CMD_STOP_AUTONOMOUS"  # Stop autonomous operation
    CMD_ADD_PATROL_WAYPOINT = "CMD_ADD_PATROL_WAYPOINT"  # Add patrol waypoint
    CMD_LABEL_FACE = "CMD_LABEL_FACE"  # Label detected face with name
    CMD_CAPTURE_FACE_SAMPLE = "CMD_CAPTURE_FACE_SAMPLE"  # Capture face sample
    CMD_TRAIN_FACES = "CMD_TRAIN_FACES"  # Train face recognizer
    CMD_LIST_KNOWN_FACES = "CMD_LIST_KNOWN_FACES"  # Get list of known faces
    CMD_DELETE_FACE = "CMD_DELETE_FACE"  # Delete a known face

    def __init__(self):
        pass
