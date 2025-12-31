# Client Autonomous Control Guide

## Overview

The client interface now has full support for autonomous features! A new "🤖 Autonomous" button opens a dedicated control window for all autonomous robot functions.

## What's New

### Autonomous Control Window

When you click the **🤖 Autonomous** button on the main client window, you'll see a new control panel with:

#### 1. **Autonomous Modes** 🎮
- **🔍 Start Exploring** - Robot wanders randomly, avoids obstacles, scans for faces
- **🚨 Start Patrol** - Robot follows predefined patrol route, watches for intruders
- **🛑 Stop Autonomous** - Stop all autonomous operation

#### 2. **Face Recognition Training** 👤
- **📷 Capture Face** - Capture a face sample for training
  - Prompts for person's name
  - Tip: Capture 5-10 samples from different angles for best results
- **🎓 Train Recognizer** - Train the face recognizer with all captured samples
- **📋 List Known Faces** - Show all known faces in database

#### 3. **Patrol Waypoints** 🗺️
Quick buttons to build patrol routes:
- **➡️ Add Forward** - Add forward movement to patrol
- **↪️ Add Turn Left** - Add left turn to patrol
- **↩️ Add Turn Right** - Add right turn to patrol

#### 4. **Auto-Calibration** 🎯
- **🎯 Auto-Calibrate & Stand** - Level robot using IMU and lift to standing height
- **Set Height** - Adjust standing height (1.0 - 5.0 inches)

#### 5. **Activity Log** 📝
Real-time log of autonomous activities and command feedback

## Quick Start Tutorial

### 1. Connect to Robot
```
1. Enter robot IP address (already saved in IP.txt)
2. Click "Connect"
3. Click "Video" to start video stream
```

### 2. Train Some Faces
```
1. Click "🤖 Autonomous" button
2. Position yourself in front of camera
3. Click "📷 Capture Face"
4. Enter your name
5. Move slightly (different angle)
6. Repeat steps 3-5 about 8-10 times
7. Click "🎓 Train Recognizer"
8. Wait for training to complete
```

### 3. Start Exploring
```
1. In autonomous window, click "🔍 Start Exploring"
2. Robot will now:
   - Wander around randomly
   - Avoid obstacles automatically
   - Scan for faces
   - Chirp when it sees you (known face)
   - Alert (buzzer) when unknown person detected
```

### 4. Set Up Patrol (Optional)
```
1. Click "🛑 Stop Autonomous" first
2. Build patrol route:
   - Click "➡️ Add Forward"
   - Click "↩️ Add Turn Right"
   - Click "➡️ Add Forward"
   - Click "↩️ Add Turn Right"
   - Click "➡️ Add Forward"
   - Click "↩️ Add Turn Right"
   - Click "➡️ Add Forward"
   - Click "↩️ Add Turn Right"
   (Makes a square patrol route)
3. Click "🚨 Start Patrol"
4. Robot patrols and alerts on unknown faces!
```

## Understanding the Interface

### Status Display
At the top of the autonomous window, you'll see current status:
- **Status: Idle** - Robot waiting for commands
- **Status: Exploring 🔍** - Exploration mode active
- **Status: Patrolling 🚨** - Patrol mode active

### Activity Log
The bottom text area shows:
- Commands sent to robot
- Confirmation of actions
- Tips and reminders
- Auto-scrolls to show latest activity

### Button States
- All buttons remain enabled during operation
- You can switch modes at any time
- Stopping autonomous mode returns to manual control

## Tips & Best Practices

### Face Recognition
1. **Multiple Samples**: Capture 8-12 samples per person
2. **Different Angles**: Front, slight left, slight right, up, down
3. **Different Lighting**: If possible, capture in various lighting
4. **Distance**: Capture from the distance you expect robot to see people
5. **Retrain**: After adding more samples, click Train again

### Exploration Mode
- Best for: Initial learning, general autonomous operation
- Robot will: Wander, avoid obstacles, learn faces
- Known faces: Short chirp
- Unknown faces: Moves closer to investigate, then alerts

### Patrol Mode
- Best for: Security, monitoring specific area
- Setup: Define route with waypoint buttons first
- Robot will: Follow route, pause at waypoints, scan for intruders
- Known faces: Quick chirp, continues patrol
- Unknown faces: Buzzer alarm for 5 seconds, red LEDs

### Patrol Waypoint Building
- **Forward**: Moves ~2 seconds forward
- **Turn Left/Right**: Rotates ~90 degrees
- Combine to create your patrol route
- Robot loops back to start automatically

### Auto-Calibration
- Use when: Robot is unbalanced, legs uneven
- Requirements: Flat ground
- Process: Takes ~30 seconds
- Result: Level stance, proper standing height

## Keyboard Shortcuts (Optional)

You can also send commands via terminal/code:

```python
# In Client.py or via Python console
client.send_data("CMD_START_EXPLORING")
client.send_data("CMD_START_PATROL")
client.send_data("CMD_STOP_AUTONOMOUS")
client.send_data("CMD_CAPTURE_FACE_SAMPLE#Alice")
client.send_data("CMD_TRAIN_FACES")
client.send_data("CMD_ADD_PATROL_WAYPOINT#CMD_MOVE#50#0#0#0#tripod")
```

## Troubleshooting

### "Button '🤖 Autonomous' not appearing"
The button is added programmatically. If you don't see it:
- Check console for errors
- Ensure `Autonomous.py` is in Client folder
- Try calling `showAutonomousWindow()` manually

### "Commands not working"
- Ensure robot is connected (check connection status)
- Check video stream is active
- Verify server has opencv-contrib-python installed

### "Face recognition not accurate"
- Capture more samples (15-20 recommended)
- Retrain after adding samples
- Ensure good lighting
- Check camera is focused

### "Patrol route not working as expected"
- Verify waypoints were added (check log)
- Start patrol mode after adding waypoints
- Test individual movements first

### "Autonomous window crashes"
- Check Python console for error details
- Ensure PyQt5 is installed: `pip install PyQt5`
- Verify `Command.py` has autonomous commands

## File Structure

```
Code/Client/
├── Main.py                    # Main GUI (updated)
├── Command.py                 # Command definitions (updated)
├── Autonomous.py              # New autonomous control window
├── Client.py                  # Client communication
└── AUTONOMOUS_CLIENT_GUIDE.md # This guide
```

## Advanced Usage

### Custom Patrol Routes

Create complex routes programmatically:

```python
# Example: Figure-8 patrol
waypoints = [
    "CMD_MOVE#50#0#0#0#tripod",    # Forward
    "CMD_MOVE#0#40#0#0#tripod",     # Turn right
    "CMD_MOVE#50#0#0#0#tripod",    # Forward
    "CMD_MOVE#0#40#0#0#tripod",     # Turn right
    "CMD_MOVE#50#0#0#0#tripod",    # Forward
    "CMD_MOVE#0#-40#0#0#tripod",    # Turn left
    "CMD_MOVE#50#0#0#0#tripod",    # Forward
    "CMD_MOVE#0#-40#0#0#tripod",    # Turn left
]

for wp in waypoints:
    client.send_data(f"CMD_ADD_PATROL_WAYPOINT#{wp}")
```

### Batch Face Training

Train multiple people at once:

```python
people = ["Alice", "Bob", "Charlie"]

for person in people:
    print(f"Position {person} in front of camera...")
    input("Press Enter when ready...")

    for i in range(10):
        client.send_data(f"CMD_CAPTURE_FACE_SAMPLE#{person}")
        print(f"Captured sample {i+1}/10 for {person}")
        time.sleep(1)

print("Training recognizer...")
client.send_data("CMD_TRAIN_FACES")
```

### Responding to Events

Monitor the server logs to see detection events:
- Known face detected: Shows name
- Unknown face: Shows "Unknown person detected!"
- Obstacle avoidance: Shows distance

## What Happens on the Server

When you click buttons in the autonomous window:

1. **Start Exploring**
   - Server initializes camera, ultrasonic, face recognition
   - Autonomous controller starts exploration state
   - Robot begins wandering behavior

2. **Capture Face**
   - Camera captures current frame
   - Face detection runs (Haar Cascade)
   - Face image saved to database
   - Confirmation logged

3. **Train Faces**
   - All face samples loaded
   - LBPH recognizer trained
   - Model saved to disk
   - Ready for recognition

4. **Start Patrol**
   - Loads patrol waypoints
   - Starts patrol loop
   - Monitors for faces at each waypoint

## Feature Summary

| Feature | GUI Control | Server-Side | Status |
|---------|-------------|-------------|--------|
| Exploration Mode | ✅ | ✅ | Ready |
| Patrol Mode | ✅ | ✅ | Ready |
| Face Capture | ✅ | ✅ | Ready |
| Face Training | ✅ | ✅ | Ready |
| Waypoint Creation | ✅ | ✅ | Ready |
| Auto-Calibration | ✅ | ✅ | Ready |
| Height Adjustment | ✅ | ✅ | Ready |
| Activity Logging | ✅ | N/A | Ready |

## Next Steps

1. **Test Basic Features**
   - Connect to robot
   - Open autonomous window
   - Train a face
   - Try exploration mode

2. **Set Up Patrol**
   - Define a simple route
   - Test patrol behavior
   - Train family/friends

3. **Fine-Tune Behaviors**
   - Adjust parameters in `autonomous.py` (server)
   - Customize patrol routes
   - Add more faces

4. **Enjoy Your Autonomous Robot!** 🎉

---

**Need Help?** Check the server-side guide: `Code/Server/AUTONOMOUS_README.md`

**Have Fun!** Your robot is now a curious explorer and protective guard dog! 🤖🐕
