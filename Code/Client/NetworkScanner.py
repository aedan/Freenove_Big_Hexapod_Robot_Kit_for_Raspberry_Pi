# -*- coding: utf-8 -*-
"""
Network Scanner for Hexapod Robot Discovery
Automatically finds robots on the local network.
"""

import socket
import threading
import time
import json
from typing import List, Dict, Optional
import ipaddress
import concurrent.futures


class RobotScanner:
    """
    Scans local network for hexapod robots.
    Uses parallel scanning for speed.
    """

    def __init__(self):
        self.discovered_robots: List[Dict] = []
        self.scan_in_progress = False
        self.discovery_port = 5002  # Command port
        self.timeout = 0.5  # Socket timeout in seconds

    def get_local_subnet(self) -> Optional[str]:
        """
        Detect local subnet for scanning.

        Returns:
            Subnet in CIDR notation (e.g., "192.168.1.0/24") or None
        """
        try:
            # Create a socket to determine local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            # Connect to external address (doesn't actually send data)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()

            # Convert to network address
            # Assume /24 subnet (most common for home networks)
            network = ipaddress.IPv4Network(f"{local_ip}/24", strict=False)
            return str(network)

        except Exception as e:
            print(f"Error detecting subnet: {e}")
            return None

    def check_robot_at_ip(self, ip: str) -> Optional[Dict]:
        """
        Check if a hexapod robot is at the given IP.

        Args:
            ip: IP address to check

        Returns:
            Robot info dict or None if not a robot
        """
        try:
            # Try to connect to command port
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)

            result = sock.connect_ex((ip, self.discovery_port))

            if result == 0:
                # Connection successful, send discovery request
                try:
                    sock.send(b"CMD_DISCOVER\n")
                    sock.settimeout(1.0)  # Wait longer for response
                    response = sock.recv(1024).decode('utf-8').strip()

                    # Check if response indicates it's a hexapod robot
                    if response.startswith("HEXAPOD:"):
                        # Parse response: HEXAPOD:name:version
                        parts = response.split(":")
                        robot_info = {
                            "ip": ip,
                            "name": parts[1] if len(parts) > 1 else "Unknown",
                            "version": parts[2] if len(parts) > 2 else "1.0",
                            "type": "Freenove Hexapod",
                            "last_seen": time.time()
                        }
                        sock.close()
                        return robot_info
                    else:
                        # Not a hexapod, but something responded
                        sock.close()
                        return None

                except socket.timeout:
                    # No response to discovery, might be robot without discovery support
                    # Try old method - assume it's a robot if port is open
                    sock.close()
                    robot_info = {
                        "ip": ip,
                        "name": f"Robot-{ip.split('.')[-1]}",
                        "version": "Unknown",
                        "type": "Freenove Hexapod (Legacy)",
                        "last_seen": time.time()
                    }
                    return robot_info

                except Exception as e:
                    sock.close()
                    return None
            else:
                sock.close()
                return None

        except Exception as e:
            return None

    def scan_network(self, subnet: Optional[str] = None,
                     progress_callback=None) -> List[Dict]:
        """
        Scan network for hexapod robots.

        Args:
            subnet: Subnet to scan in CIDR notation, or None to auto-detect
            progress_callback: Optional callback function(current, total)

        Returns:
            List of discovered robot info dicts
        """
        if self.scan_in_progress:
            print("Scan already in progress")
            return self.discovered_robots

        self.scan_in_progress = True
        self.discovered_robots = []

        # Auto-detect subnet if not provided
        if subnet is None:
            subnet = self.get_local_subnet()
            if subnet is None:
                print("Could not detect local subnet")
                self.scan_in_progress = False
                return []

        print(f"Scanning subnet: {subnet}")

        try:
            network = ipaddress.IPv4Network(subnet, strict=False)
            hosts = list(network.hosts())
            total_hosts = len(hosts)

            print(f"Scanning {total_hosts} IP addresses...")

            # Use thread pool for parallel scanning
            with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
                futures = {}
                for idx, host in enumerate(hosts):
                    ip = str(host)
                    future = executor.submit(self.check_robot_at_ip, ip)
                    futures[future] = (ip, idx)

                # Collect results as they complete
                for future in concurrent.futures.as_completed(futures):
                    ip, idx = futures[future]

                    try:
                        robot_info = future.result()
                        if robot_info:
                            self.discovered_robots.append(robot_info)
                            print(f"Found robot at {ip}: {robot_info['name']}")

                    except Exception as e:
                        pass

                    # Update progress
                    if progress_callback:
                        progress_callback(idx + 1, total_hosts)

        except Exception as e:
            print(f"Scan error: {e}")

        finally:
            self.scan_in_progress = False

        print(f"Scan complete. Found {len(self.discovered_robots)} robot(s)")
        return self.discovered_robots

    def scan_network_async(self, subnet: Optional[str] = None,
                           completion_callback=None,
                           progress_callback=None):
        """
        Scan network asynchronously in a separate thread.

        Args:
            subnet: Subnet to scan
            completion_callback: Called when scan completes with robot list
            progress_callback: Called periodically with (current, total)
        """
        def scan_thread():
            robots = self.scan_network(subnet, progress_callback)
            if completion_callback:
                completion_callback(robots)

        thread = threading.Thread(target=scan_thread, daemon=True)
        thread.start()

    def save_discovered_robots(self, filename: str = "discovered_robots.json"):
        """Save discovered robots to file."""
        try:
            with open(filename, 'w') as f:
                json.dump(self.discovered_robots, f, indent=2)
            print(f"Saved {len(self.discovered_robots)} robots to {filename}")
        except Exception as e:
            print(f"Error saving robots: {e}")

    def load_discovered_robots(self, filename: str = "discovered_robots.json") -> List[Dict]:
        """
        Load previously discovered robots from file.

        Returns:
            List of robot info dicts
        """
        try:
            with open(filename, 'r') as f:
                robots = json.load(f)
                print(f"Loaded {len(robots)} robots from {filename}")
                return robots
        except FileNotFoundError:
            return []
        except Exception as e:
            print(f"Error loading robots: {e}")
            return []

    def quick_check_robot(self, ip: str) -> bool:
        """
        Quick check if robot is still reachable.

        Args:
            ip: IP address to check

        Returns:
            True if robot is reachable
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.3)
            result = sock.connect_ex((ip, self.discovery_port))
            sock.close()
            return result == 0
        except:
            return False


if __name__ == '__main__':
    """Test the network scanner."""
    print("=" * 50)
    print("Hexapod Robot Network Scanner - Test Mode")
    print("=" * 50)

    scanner = RobotScanner()

    # Try to load previously discovered robots
    cached_robots = scanner.load_discovered_robots()
    if cached_robots:
        print(f"\nPreviously discovered robots:")
        for robot in cached_robots:
            print(f"  - {robot['name']} at {robot['ip']}")

    print("\nStarting network scan...")

    def progress(current, total):
        if current % 10 == 0:  # Print every 10 IPs
            percent = (current / total) * 100
            print(f"Progress: {current}/{total} ({percent:.1f}%)")

    # Scan network
    robots = scanner.scan_network(progress_callback=progress)

    print("\n" + "=" * 50)
    print(f"Scan Results: {len(robots)} robot(s) found")
    print("=" * 50)

    if robots:
        for robot in robots:
            print(f"\nRobot: {robot['name']}")
            print(f"  IP: {robot['ip']}")
            print(f"  Type: {robot['type']}")
            print(f"  Version: {robot['version']}")

        # Save results
        scanner.save_discovered_robots()
    else:
        print("No robots found on the network.")
        print("\nTroubleshooting:")
        print("  1. Ensure robot is powered on")
        print("  2. Check robot is connected to same network")
        print("  3. Verify firewall settings")
        print("  4. Try manual IP entry")
