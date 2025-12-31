"""
Setup script for Freenove Hexapod Robot on Raspberry Pi.
Installs dependencies and configures system settings, with optimizations for Pi 4.
"""

import subprocess
import re


def check_and_install(package):
    """Check if a Python package is installed, and install it if not."""
    try:
        __import__(package)
        return True
    except ImportError:
        install_command = f"sudo pip3 install {package}"
        try:
            subprocess.run(install_command, shell=True, check=True)
            return True
        except subprocess.CalledProcessError:
            print(f"Failed to install {package}.")
            return False


def apt_install(package):
    """Install a package using apt-get."""
    install_command = f"sudo apt-get install -y {package}"
    try:
        subprocess.run(install_command, shell=True, check=True)
        return True
    except subprocess.CalledProcessError:
        print(f"Failed to install {package} via apt-get.")
        return False


def custom_install(command):
    """Execute a custom installation command."""
    try:
        subprocess.run(command, shell=True, check=True)
        return True
    except subprocess.CalledProcessError:
        print(f"Failed to execute custom command: {command}")
        return False


def get_raspberry_pi_version():
    """
    Detect Raspberry Pi model.

    Returns:
        int: 5 for Pi 5, 4 for Pi 4, 3 for Pi 3, 1 for other models, 0 on error
    """
    print("Detecting Raspberry Pi model...")
    try:
        result = subprocess.run(
            ['cat', '/sys/firmware/devicetree/base/model'],
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            model = result.stdout.strip()
            print(f"Detected: {model}")

            # Extract version number using regex
            if "Raspberry Pi 5" in model:
                print("→ Raspberry Pi 5")
                return 5
            elif "Raspberry Pi 4" in model:
                print("→ Raspberry Pi 4 (optimizations will be applied)")
                return 4
            elif "Raspberry Pi 3" in model:
                print("→ Raspberry Pi 3")
                return 3
            else:
                print(f"→ Raspberry Pi (model not specifically recognized)")
                return 1
        else:
            print("Failed to get Raspberry Pi model information.")
            return 0
    except Exception as e:
        print(f"Error detecting Raspberry Pi version: {e}")
        return 0


def update_config_file(file_path, command, value):
    """
    Update or add a configuration parameter in config.txt.

    Args:
        file_path: Path to config.txt
        command: Configuration parameter name
        value: Value to set
    """
    new_content = []
    command_found = False

    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()

        for line in lines:
            stripped_line = line.strip()
            # Check if line contains the command (commented or not)
            if stripped_line.startswith(command) or stripped_line.startswith(f'#{command}'):
                command_found = True
                new_content.append(f'{command}={value}\n')
            else:
                new_content.append(line)

        # Add parameter if not found
        if not command_found:
            new_content.append(f'\n{command}={value}\n')

        with open(file_path, 'w') as f:
            f.writelines(new_content)

        print(f"✓ Updated {file_path}: {command}={value}")
        return True

    except Exception as e:
        print(f"✗ Error updating {file_path}: {e}")
        return False


def config_camera_to_config_txt(file_path, command, value=None):
    """
    Configure camera in config.txt using dtoverlay.

    Args:
        file_path: Path to config.txt
        command: Camera model (ov5647, imx219, etc.)
        value: Optional value (e.g., cam0, cam1 for Pi 5)
    """
    new_content = []
    command_found = False

    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()

        # Remove old camera configurations
        for line in lines:
            stripped_line = line.strip()
            # Skip old camera overlay lines
            if 'ov5647' in stripped_line or 'imx219' in stripped_line:
                continue
            # Update or keep line
            if stripped_line.startswith(f'dtoverlay={command}') or \
               stripped_line.startswith(f'#dtoverlay={command}'):
                command_found = True
                if value:
                    new_content.append(f'dtoverlay={command},{value}\n')
                else:
                    new_content.append(f'dtoverlay={command}\n')
            else:
                new_content.append(line)

        # Add if not found
        if not command_found:
            if value:
                new_content.append(f'\ndtoverlay={command},{value}\n')
            else:
                new_content.append(f'\ndtoverlay={command}\n')

        with open(file_path, 'w') as f:
            f.writelines(new_content)

        value_str = f",{value}" if value else ""
        print(f"✓ Camera configured: dtoverlay={command}{value_str}")
        return True

    except Exception as e:
        print(f"✗ Error configuring camera: {e}")
        return False


def backup_file(file_path):
    """Create a backup of a file."""
    backup_path = file_path + '.bak'
    print(f"Creating backup: {backup_path}")
    try:
        with open(file_path, 'rb') as src_file:
            with open(backup_path, 'wb') as dst_file:
                dst_file.write(src_file.read())
        print(f"✓ Backup created: {backup_path}")
        return True
    except Exception as e:
        print(f"✗ Error creating backup: {e}")
        return False


def config_i2c_for_pi4(file_path):
    """
    Configure I2C at 400kHz for Pi 4.
    Enables I2C and sets clock speed to 400kHz (Fast Mode).

    Args:
        file_path: Path to config.txt
    """
    print("\n=== Configuring I2C for Pi 4 ===")
    update_config_file(file_path, 'dtparam=i2c_arm', 'on')
    update_config_file(file_path, 'dtparam=i2c_arm_baudrate', '400000')
    print("✓ I2C configured at 400kHz (Fast Mode)")


def config_gpu_memory(file_path):
    """
    Allocate GPU memory for camera/encoding on Pi 4.
    256MB is sufficient for H264 hardware encoding.

    Args:
        file_path: Path to config.txt
    """
    print("\n=== Configuring GPU Memory ===")
    update_config_file(file_path, 'gpu_mem', '256')
    print("✓ GPU memory set to 256MB for camera encoding")


def config_file():
    """Configure system settings in config.txt."""
    pi_version = get_raspberry_pi_version()
    file_path = '/boot/firmware/config.txt'

    # Create backup
    backup_file(file_path)

    # Basic configuration
    print("\n=== Basic Configuration ===")
    update_config_file(file_path, 'dtparam=spi', 'on')
    update_config_file(file_path, 'camera_auto_detect', '0')

    # Camera configuration
    print("\n=== Camera Configuration ===")
    while True:
        camera_model = input(
            "Enter camera model (ov5647 or imx219): "
        ).strip().lower()
        if camera_model not in ['ov5647', 'imx219']:
            print("Invalid input. Please enter either ov5647 or imx219.")
        else:
            break

    if pi_version == 5:
        print("Configuring for Raspberry Pi 5...")
        while True:
            camera_port = input(
                "Which camera port is connected? (cam0 or cam1): "
            ).strip().lower()
            if camera_port not in ['cam0', 'cam1']:
                print("Invalid input. Please enter either cam0 or cam1.")
            else:
                break
        config_camera_to_config_txt(file_path, camera_model, camera_port)

    elif pi_version == 4:
        print("Configuring for Raspberry Pi 4...")
        config_camera_to_config_txt(file_path, camera_model)
        # Apply Pi 4 specific optimizations
        config_i2c_for_pi4(file_path)
        config_gpu_memory(file_path)

    elif pi_version == 3:
        print("Configuring for Raspberry Pi 3...")
        update_config_file(file_path, 'dtparam=audio', 'off')
        config_camera_to_config_txt(file_path, camera_model)

    else:
        print("Configuring with default settings...")
        config_camera_to_config_txt(file_path, camera_model)

    print("\n=== Configuration Complete ===")


def main():
    """Main setup routine."""
    print("=" * 60)
    print("Freenove Hexapod Robot - Setup Script")
    print("=" * 60)

    install_status = {
        "rpi-ws281x-python": False,
        "mpu6050": False,
        "libqt5gui5 python3-dev python3-pyqt5": False,
        "python3-opencv": False
    }

    print("\n=== Updating Package List ===")
    try:
        subprocess.run("sudo apt-get update", shell=True, check=True)
        print("✓ Package list updated")
    except subprocess.CalledProcessError:
        print("✗ Failed to update package list")

    print("\n=== Installing Dependencies ===")

    # Install rpi-ws281x-python
    print("\n1. Installing rpi-ws281x-python (WS2812B LED driver)...")
    install_status["rpi-ws281x-python"] = custom_install(
        "cd ./Libs/rpi-ws281x-python/library && sudo python3 setup.py install"
    )

    # Install mpu6050
    print("\n2. Installing mpu6050 (IMU sensor library)...")
    install_status["mpu6050"] = custom_install(
        "cd ./Libs/mpu6050 && sudo python3 setup.py install"
    )

    # Install Qt5 and PyQt5
    print("\n3. Installing Qt5 GUI libraries...")
    install_status["libqt5gui5 python3-dev python3-pyqt5"] = apt_install(
        "libqt5gui5 python3-dev python3-pyqt5"
    )

    # Install OpenCV
    print("\n4. Installing OpenCV (python3-opencv)...")
    install_status["python3-opencv"] = apt_install("python3-opencv")

    # Summary
    print("\n" + "=" * 60)
    print("Installation Summary:")
    print("=" * 60)
    for lib, status in install_status.items():
        status_icon = "✓" if status else "✗"
        print(f"{status_icon} {lib}")

    if all(install_status.values()):
        print("\n✓ All dependencies installed successfully")
        config_file()
        print("\n" + "=" * 60)
        print("IMPORTANT: Please reboot your Raspberry Pi to apply changes")
        print("Run: sudo reboot")
        print("=" * 60)
    else:
        missing_libraries = [
            lib for lib, status in install_status.items() if not status
        ]
        print(f"\n✗ Some dependencies failed to install:")
        for lib in missing_libraries:
            print(f"  - {lib}")
        print("\nPlease run the script again or install manually.")


if __name__ == "__main__":
    main()
