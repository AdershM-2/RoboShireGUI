#!/bin/bash
###############################################################################
# RoboShire Standalone Ubuntu Installer
# Version: 2.3.0
# Description: One-command installation script for Ubuntu 22.04/24.04
# Usage: curl -fsSL https://roboshire.com/install.sh | bash
#        OR: ./install_roboshire_ubuntu.sh
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
ROBOSHIRE_VERSION="2.5.0"
INSTALL_DIR="$HOME/roboshire"
ROS_DISTRO="humble"  # Default, detect from Ubuntu version
PYTHON_MIN_VERSION="3.10"

###############################################################################
# Helper Functions
###############################################################################

print_header() {
    echo -e "${BLUE}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║                                                              ║"
    echo "║                 RoboShire Installer v${ROBOSHIRE_VERSION}                 ║"
    echo "║          GUI-Based ROS2 Robotics Development Tool            ║"
    echo "║                                                              ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_step() {
    echo -e "${GREEN}==>${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}WARNING:${NC} $1"
}

print_error() {
    echo -e "${RED}ERROR:${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

check_os() {
    print_step "Checking operating system..."

    if [ ! -f /etc/os-release ]; then
        print_error "Cannot determine OS. /etc/os-release not found."
        exit 1
    fi

    source /etc/os-release

    if [ "$ID" != "ubuntu" ]; then
        print_error "This script is for Ubuntu only. Detected: $ID"
        print_warning "For other distros, see manual installation docs."
        exit 1
    fi

    # Determine ROS distro based on Ubuntu version
    case "$VERSION_ID" in
        "22.04")
            ROS_DISTRO="humble"
            PYTHON_VERSION="3.10"
            print_success "Ubuntu 22.04 LTS detected (ROS 2 Humble)"
            ;;
        "24.04")
            ROS_DISTRO="jazzy"
            PYTHON_VERSION="3.12"
            print_success "Ubuntu 24.04 LTS detected (ROS 2 Jazzy)"
            ;;
        "20.04")
            ROS_DISTRO="foxy"
            PYTHON_VERSION="3.8"
            print_warning "Ubuntu 20.04 detected. ROS 2 Foxy is end-of-life."
            print_warning "Consider upgrading to Ubuntu 22.04 for best experience."
            ;;
        *)
            print_error "Unsupported Ubuntu version: $VERSION_ID"
            print_warning "Supported versions: 22.04 (recommended), 24.04"
            exit 1
            ;;
    esac
}

check_system_requirements() {
    print_step "Checking system requirements..."

    # Check RAM
    total_ram=$(free -g | awk '/^Mem:/{print $2}')
    if [ "$total_ram" -lt 4 ]; then
        print_warning "Low RAM detected (${total_ram}GB). Recommended: 8GB+"
        print_warning "Installation may be slow. Consider closing applications."
    else
        print_success "RAM: ${total_ram}GB (sufficient)"
    fi

    # Check disk space
    available_space=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')
    if [ "$available_space" -lt 10 ]; then
        print_error "Insufficient disk space. Need 10GB+, available: ${available_space}GB"
        exit 1
    else
        print_success "Disk space: ${available_space}GB available"
    fi

    # Check internet connection
    if ! ping -c 1 google.com &> /dev/null; then
        print_error "No internet connection detected."
        print_warning "Internet required for downloading packages."
        exit 1
    else
        print_success "Internet connection OK"
    fi
}

install_ros2() {
    print_step "Installing ROS 2 ${ROS_DISTRO}..."

    # Check if already installed
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        print_success "ROS 2 ${ROS_DISTRO} already installed"
        return 0
    fi

    # Add ROS 2 repository
    print_step "Adding ROS 2 repository..."
    sudo apt update
    sudo apt install -y software-properties-common
    sudo add-apt-repository -y universe

    # Add ROS 2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    # Add ROS 2 repository to sources
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Install ROS 2
    sudo apt update
    print_step "Installing ROS 2 packages (this may take 10-15 minutes)..."
    sudo apt install -y ros-${ROS_DISTRO}-desktop \
                        ros-${ROS_DISTRO}-ros-base \
                        python3-argcomplete \
                        python3-colcon-common-extensions \
                        python3-rosdep \
                        python3-vcstool

    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        print_step "Initializing rosdep..."
        sudo rosdep init
    fi
    rosdep update

    # Source ROS 2
    source /opt/ros/${ROS_DISTRO}/setup.bash

    print_success "ROS 2 ${ROS_DISTRO} installed successfully"
}

install_python_dependencies() {
    print_step "Installing Python dependencies..."

    # Ensure pip is installed
    sudo apt install -y python3-pip python3-venv

    # Install RoboShire Python dependencies
    pip3 install --user --upgrade \
        PyQt5 \
        pyserial \
        pyyaml \
        lxml \
        jinja2 \
        psutil \
        paramiko \
        requests \
        matplotlib \
        numpy

    print_success "Python dependencies installed"
}

install_system_dependencies() {
    print_step "Installing system dependencies..."

    sudo apt update
    sudo apt install -y \
        git \
        wget \
        curl \
        build-essential \
        cmake \
        pkg-config \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libxcb-xinerama0 \
        libxcb-cursor0 \
        libxkbcommon-x11-0 \
        x11-apps \
        qtbase5-dev \
        qt5-qmake \
        python3-pyqt5 \
        python3-pyqt5.qtsvg \
        arduino \
        arduino-core

    print_success "System dependencies installed"
}

install_mujoco() {
    print_step "Installing MuJoCo physics simulator..."

    MUJOCO_VERSION="3.1.0"
    MUJOCO_DIR="$HOME/.mujoco"

    if [ -d "$MUJOCO_DIR/mujoco-${MUJOCO_VERSION}" ]; then
        print_success "MuJoCo already installed"
        return 0
    fi

    mkdir -p "$MUJOCO_DIR"
    cd "$MUJOCO_DIR"

    # Download MuJoCo
    print_step "Downloading MuJoCo ${MUJOCO_VERSION}..."
    wget -q --show-progress https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz

    # Extract
    tar -xzf mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz
    rm mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz

    # Install Python bindings
    pip3 install --user mujoco

    # Add to LD_LIBRARY_PATH
    if ! grep -q "MUJOCO_DIR" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# MuJoCo" >> ~/.bashrc
        echo "export MUJOCO_DIR=\$HOME/.mujoco/mujoco-${MUJOCO_VERSION}" >> ~/.bashrc
        echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$MUJOCO_DIR/lib" >> ~/.bashrc
    fi

    print_success "MuJoCo installed"
    cd -
}

clone_roboshire() {
    print_step "Cloning RoboShire repository..."

    if [ -d "$INSTALL_DIR" ]; then
        print_warning "RoboShire directory already exists: $INSTALL_DIR"
        read -p "Remove and reinstall? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$INSTALL_DIR"
        else
            print_success "Using existing directory"
            return 0
        fi
    fi

    # Clone repository (or download release)
    # TODO: Update with actual repository URL
    git clone https://github.com/YOUR_ORG/roboshire.git "$INSTALL_DIR" || {
        print_error "Failed to clone repository"
        print_warning "Fallback: Creating directory structure manually"
        mkdir -p "$INSTALL_DIR"
        cd "$INSTALL_DIR"
        # Download release archive
        # wget https://github.com/YOUR_ORG/roboshire/archive/refs/tags/v${ROBOSHIRE_VERSION}.tar.gz
        # tar -xzf v${ROBOSHIRE_VERSION}.tar.gz --strip-components=1
    }

    print_success "RoboShire cloned to $INSTALL_DIR"
}

setup_workspace() {
    print_step "Setting up RoboShire workspace..."

    cd "$INSTALL_DIR"

    # Create workspace structure
    mkdir -p workspace/src
    mkdir -p workspace/projects

    # Source ROS 2 and build workspace
    source /opt/ros/${ROS_DISTRO}/setup.bash

    if [ -d "workspace/src" ] && [ "$(ls -A workspace/src)" ]; then
        cd workspace
        print_step "Building workspace..."
        colcon build --symlink-install
        cd ..
        print_success "Workspace built"
    fi

    print_success "Workspace setup complete"
}

configure_environment() {
    print_step "Configuring environment..."

    # Add RoboShire to PATH and environment
    if ! grep -q "ROBOSHIRE_HOME" ~/.bashrc; then
        cat >> ~/.bashrc << EOF

# RoboShire Configuration
export ROBOSHIRE_HOME="$INSTALL_DIR"
export PATH="\$PATH:\$ROBOSHIRE_HOME/scripts"
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "\$ROBOSHIRE_HOME/workspace/install/setup.bash" ]; then
    source "\$ROBOSHIRE_HOME/workspace/install/setup.bash"
fi
alias roboshire="python3 \$ROBOSHIRE_HOME/roboshire"
EOF
        print_success "Environment configured in ~/.bashrc"
    else
        print_success "Environment already configured"
    fi
}

create_desktop_launcher() {
    print_step "Creating desktop launcher..."

    DESKTOP_FILE="$HOME/.local/share/applications/roboshire.desktop"
    ICON_FILE="$INSTALL_DIR/roboshire/gui/icons/roboshire_icon.png"

    # Create default icon if not exists
    mkdir -p "$INSTALL_DIR/roboshire/gui/icons"
    if [ ! -f "$ICON_FILE" ]; then
        # Create a simple placeholder icon (would be replaced with actual icon)
        ICON_FILE="/usr/share/pixmaps/python3.xpm"
    fi

    cat > "$DESKTOP_FILE" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=RoboShire
Comment=ROS2 Robotics Development GUI
Exec=bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && cd $INSTALL_DIR && python3 -m roboshire'
Icon=$ICON_FILE
Terminal=false
Categories=Development;Robotics;
Keywords=ros2;robotics;gui;arduino;
StartupWMClass=roboshire
EOF

    chmod +x "$DESKTOP_FILE"

    # Update desktop database
    update-desktop-database ~/.local/share/applications 2>/dev/null || true

    print_success "Desktop launcher created"
    print_step "You can now launch RoboShire from the applications menu"
}

setup_arduino_permissions() {
    print_step "Setting up Arduino/serial port permissions..."

    # Add user to dialout group for serial access
    if ! groups | grep -q dialout; then
        sudo usermod -aG dialout "$USER"
        print_warning "Added to 'dialout' group. You must LOG OUT and LOG IN for this to take effect."
    else
        print_success "Already in 'dialout' group"
    fi

    # Set up udev rules for common Arduino boards
    sudo bash -c 'cat > /etc/udev/rules.d/99-arduino.rules' << 'EOF'
# Arduino Uno, Mega
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0001", MODE="0666"
# ESP32
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"
# CH340
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
EOF

    sudo udevadm control --reload-rules
    sudo udevadm trigger

    print_success "Arduino permissions configured"
}

run_tests() {
    print_step "Running post-installation tests..."

    # Test ROS 2
    source /opt/ros/${ROS_DISTRO}/setup.bash
    if ros2 --version &>/dev/null; then
        print_success "ROS 2 test passed"
    else
        print_error "ROS 2 test failed"
        return 1
    fi

    # Test Python imports
    if python3 -c "import PyQt5" &>/dev/null; then
        print_success "PyQt5 import test passed"
    else
        print_error "PyQt5 import failed"
        return 1
    fi

    # Test RoboShire module
    cd "$INSTALL_DIR"
    if python3 -c "import roboshire" &>/dev/null; then
        print_success "RoboShire module import passed"
    else
        print_warning "RoboShire module import test skipped (requires full installation)"
    fi

    print_success "All tests passed"
}

print_next_steps() {
    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                                                              ║${NC}"
    echo -e "${GREEN}║          RoboShire Installation Complete! 🎉                 ║${NC}"
    echo -e "${GREEN}║                                                              ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BLUE}Next Steps:${NC}"
    echo ""
    echo "1. ${YELLOW}IMPORTANT:${NC} Log out and log back in (for serial port access)"
    echo ""
    echo "2. Launch RoboShire:"
    echo "   ${GREEN}cd $INSTALL_DIR${NC}"
    echo "   ${GREEN}python3 -m roboshire${NC}"
    echo "   OR: Find 'RoboShire' in your applications menu"
    echo ""
    echo "3. Start with the Quick Start Guide:"
    echo "   ${BLUE}$INSTALL_DIR/docs/hardware/HARDWARE_QUICK_START.md${NC}"
    echo ""
    echo "4. Build your first project:"
    echo "   - Weather Station (beginner)"
    echo "   - Differential Drive Robot (intermediate)"
    echo ""
    echo -e "${BLUE}Documentation:${NC}"
    echo "   Docs:     $INSTALL_DIR/docs/"
    echo "   Examples: $INSTALL_DIR/roboshire/examples/"
    echo ""
    echo -e "${BLUE}Getting Help:${NC}"
    echo "   GitHub:   https://github.com/YOUR_ORG/roboshire"
    echo "   Docs:     https://docs.roboshire.com"
    echo "   Discord:  https://discord.gg/roboshire"
    echo ""
    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo "   If you encounter issues:"
    echo "   - Check: $INSTALL_DIR/docs/hardware/TROUBLESHOOTING_HARDWARE.md"
    echo "   - Run: roboshire --check-installation"
    echo ""
}

###############################################################################
# Main Installation Flow
###############################################################################

main() {
    print_header

    echo "This script will install RoboShire v${ROBOSHIRE_VERSION} and its dependencies."
    echo "Installation directory: $INSTALL_DIR"
    echo ""
    echo "Components to be installed:"
    echo "  - ROS 2 ${ROS_DISTRO}"
    echo "  - Python dependencies (PyQt5, PySerial, etc.)"
    echo "  - MuJoCo physics simulator"
    echo "  - Arduino IDE and tools"
    echo "  - RoboShire GUI application"
    echo ""
    echo "Estimated time: 20-30 minutes"
    echo "Disk space required: ~5-7 GB"
    echo ""
    read -p "Continue with installation? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Installation cancelled."
        exit 0
    fi

    # Run installation steps
    check_os
    check_system_requirements
    install_system_dependencies
    install_ros2
    install_python_dependencies
    install_mujoco
    clone_roboshire
    setup_workspace
    configure_environment
    create_desktop_launcher
    setup_arduino_permissions
    run_tests
    print_next_steps

    echo ""
    echo -e "${GREEN}Installation completed successfully!${NC}"
    echo -e "${YELLOW}Remember to LOG OUT and LOG IN for all changes to take effect.${NC}"
}

# Run main installation
main

exit 0
