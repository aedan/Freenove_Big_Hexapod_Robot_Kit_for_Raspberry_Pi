# Network Discovery System - User Guide

## Overview

Your hexapod robot client now features **automatic network discovery**! No more manually entering IP addresses - the client can automatically find all robots on your local network and let you select which one to connect to.

## Features

### 🔍 Automatic Network Scanning
- Scans your entire local subnet (e.g., 192.168.1.0/24)
- Parallel scanning for speed (~5-15 seconds for 254 addresses)
- Discovers all hexapod robots automatically

### 🤖 Robot Identification
- Shows robot name, IP address, type, and version
- Distinguishes between different robot models
- Saves discovered robots for quick access later

### 💾 Smart Caching
- Remembers previously discovered robots
- Quick "Refresh" to check if cached robots are still online
- No need to re-scan every time

### ✋ Manual Override
- Can still manually enter IP addresses
- Useful for robots on different subnets
- Fallback when auto-discovery doesn't work

## How to Use

### Quick Start

#### 1. **Launch the Client**
```bash
cd Code/Client
python3 Main.py
```

#### 2. **Click "🔍 Find Robots" Button**
- Located on the main window (auto-added if not in UI)
- Opens the Robot Selector dialog

#### 3. **Scan Network**
- Click "🔍 Scan Network" in the dialog
- Wait 5-15 seconds for scan to complete
- See list of discovered robots

#### 4. **Select Robot**
- Double-click a robot in the list, OR
- Single-click to select, then click "✓ Connect to Selected Robot"
- Robot IP is automatically filled into the main window

#### 5. **Connect**
- Click "Connect" on the main window
- Start using your robot!

### Alternative: Quick Refresh

If you've scanned before:
1. Click "🔍 Find Robots"
2. Click "🔄 Refresh" (faster - only checks cached robots)
3. Select robot and connect

### Manual IP Entry

If auto-discovery doesn't work:
1. Click "🔍 Find Robots"
2. Enter IP in "Manual Connection" section
3. Click "Connect"

## User Interface Guide

### Robot Selector Dialog

```
┌────────────────────────────────────────┐
│  🤖 Hexapod Robot Network Discovery   │
├────────────────────────────────────────┤
│  Network Scan                          │
│  [🔍 Scan Network] [🔄 Refresh]        │
│  ▓▓▓▓▓▓▓▓▓▓▓▓▓░░░░░░░ 65%             │
│  Status: Scanning... 165/254 (65%)     │
├────────────────────────────────────────┤
│  Discovered Robots                     │
│  ┌──────────────────────────────────┐ │
│  │ 🤖 MyHexapod                     │ │
│  │     IP: 192.168.1.100            │ │
│  │     Type: Freenove Hexapod       │ │
│  │     Version: 2.0                 │ │
│  │                                  │ │
│  │ 🤖 Workshop-Robot                │ │
│  │     IP: 192.168.1.105            │ │
│  │     Type: Freenove Hexapod       │ │
│  │     Version: 2.0                 │ │
│  └──────────────────────────────────┘ │
├────────────────────────────────────────┤
│  Manual Connection                     │
│  IP Address: [192.168.1.___] [Connect]│
├────────────────────────────────────────┤
│  [✓ Connect to Selected Robot]        │
│  [✗ Cancel]                            │
└────────────────────────────────────────┘
```

### Button Descriptions

| Button | Function |
|--------|----------|
| **🔍 Scan Network** | Scan entire local subnet for robots (5-15 seconds) |
| **🔄 Refresh** | Quick check of previously discovered robots (1-2 seconds) |
| **✓ Connect to Selected Robot** | Use the selected robot from the list |
| **Manual Connect** | Connect to manually entered IP address |
| **✗ Cancel** | Close dialog without selecting |

### Status Messages

| Status | Meaning |
|--------|---------|
| "Ready to scan" | Initial state, ready to start scanning |
| "Scanning... 165/254 (65%)" | Scan in progress |
| "Found 2 robot(s)" | Scan complete, robots discovered |
| "No robots found" | Scan complete, no robots on network |
| "2/5 robot(s) online" | Refresh complete, some cached robots reachable |
| "Loaded 3 cached robot(s)" | Previously discovered robots loaded from file |

## How It Works

### Network Discovery Protocol

1. **Client Side** (NetworkScanner.py):
   - Detects local subnet (e.g., 192.168.1.0/24)
   - Scans all 254 possible IP addresses in parallel
   - Tries to connect to command port (5002)
   - Sends `CMD_DISCOVER` command

2. **Server Side** (Server responds):
   - Receives `CMD_DISCOVER` command
   - Responds with: `HEXAPOD:robot_name:version`
   - Example: `HEXAPOD:MyHexapod:2.0`

3. **Client Side** (Collects results):
   - Parses server responses
   - Builds list of discovered robots
   - Displays in GUI
   - Saves to `discovered_robots.json`

### Caching System

Discovered robots are saved to `Code/Client/discovered_robots.json`:

```json
[
  {
    "ip": "192.168.1.100",
    "name": "MyHexapod",
    "version": "2.0",
    "type": "Freenove Hexapod",
    "last_seen": 1234567890.12
  },
  {
    "ip": "192.168.1.105",
    "name": "Workshop-Robot",
    "version": "2.0",
    "type": "Freenove Hexapod",
    "last_seen": 1234567895.34
  }
]
```

This file is automatically loaded next time you open the selector!

## Server Configuration

### Setting Robot Name

On the server, you can customize the robot name:

**Option 1: Edit `Code/Server/params.json`**
```json
{
  "network": {
    "robot_name": "MyHexapod"
  }
}
```

**Option 2: Code (config.py)**
```python
@dataclass
class NetworkConfig:
    robot_name: str = "MyCustomName"
```

The name will appear in the client's robot list!

### Default Name

If not configured, defaults to: `"HexapodRobot"`

## Troubleshooting

### "No robots found"

**Possible causes:**
1. Robot not powered on
2. Robot on different network/subnet
3. Firewall blocking port 5002
4. Robot server not running

**Solutions:**
- Check robot is powered and server is running
- Verify both devices on same WiFi network
- Check IP range: run `ifconfig` or `ip addr` on robot
- Try manual IP entry as fallback
- Check firewall settings

### "Scan takes too long"

**Normal behavior:**
- Full scan: 5-15 seconds (scans 254 addresses)
- Refresh: 1-2 seconds (checks cached only)

**To speed up:**
- Use "Refresh" instead of full scan
- Manually enter IP if you know it

### "Robot not responding after selection"

**Check:**
1. Robot still powered on?
2. Network connection stable?
3. Try manual connection to verify IP

### "Discovered wrong device"

**Explanation:**
- Scanner finds any device responding on port 5002
- Non-hexapod devices might respond differently
- Look for "Type: Freenove Hexapod" in the list

**Solution:**
- Only select robots showing "Freenove Hexapod" type
- Unknown devices show as "Legacy" type

## Technical Details

### Performance

- **Scan Speed**: 50 parallel connections
- **Timeout**: 0.5 seconds per IP
- **Total Time**: ~5-15 seconds for /24 subnet
- **Caching**: Instant load of previous results

### Compatibility

- **Client**: Python 3.7+, PyQt5
- **Server**: Must support `CMD_DISCOVER` (v2.0+)
- **Legacy Servers**: Will be detected but show as "Unknown" version
- **Network**: Works on any IPv4 network

### Supported Subnets

- Automatically detects: `/24` networks (192.168.1.0/24)
- **Custom subnets**: Edit `NetworkScanner.py` if needed

```python
# For custom subnet:
scanner = RobotScanner()
robots = scanner.scan_network(subnet="10.0.1.0/24")
```

### Files Created

| File | Purpose | Location |
|------|---------|----------|
| `NetworkScanner.py` | Scanner logic | `Code/Client/` |
| `RobotSelector.py` | GUI dialog | `Code/Client/` |
| `discovered_robots.json` | Cache file | `Code/Client/` |

## Advanced Usage

### Programmatic Discovery

You can use the scanner from Python code:

```python
from NetworkScanner import RobotScanner

# Create scanner
scanner = RobotScanner()

# Scan network
robots = scanner.scan_network()

# Print results
for robot in robots:
    print(f"{robot['name']}: {robot['ip']}")

# Save results
scanner.save_discovered_robots()

# Load cached
cached = scanner.load_discovered_robots()
```

### Asynchronous Scanning

```python
def on_scan_complete(robots):
    print(f"Found {len(robots)} robots")

scanner.scan_network_async(
    completion_callback=on_scan_complete
)
```

### Quick Availability Check

```python
is_online = scanner.quick_check_robot("192.168.1.100")
print(f"Robot online: {is_online}")
```

## Future Enhancements

Potential additions:

- **mDNS/Bonjour**: Automatic discovery without scanning
- **Robot grouping**: Organize multiple robots
- **Connection history**: Remember last connected robot
- **Network profiles**: Save different network configurations
- **Multi-robot control**: Connect to multiple robots simultaneously

## Command Line Testing

Test the scanner from command line:

```bash
cd Code/Client
python3 NetworkScanner.py
```

This will:
1. Scan your local network
2. Display all found robots
3. Save results to `discovered_robots.json`

## Summary

The network discovery system makes connecting to your hexapod robot effortless:

1. **One Click**: Open robot selector
2. **Auto-Scan**: Find all robots on network
3. **Select**: Choose your robot from the list
4. **Connect**: Start controlling immediately

No more IP address hunting! 🎉

---

**Questions?** Check the autonomous guides or server documentation for more features.
