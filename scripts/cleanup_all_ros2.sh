#!/bin/bash
###############################################################################
# RobotStudio - Complete ROS2 Cleanup Script
#
# This script performs a complete cleanup of:
# - All ROS2 workspace build artifacts (build/, install/, log/)
# - All running ROS2 nodes and processes
# - ROS2 daemon and discovery processes
#
# Usage: Run this on Ubuntu VM to completely reset ROS2 environment
#
# WARNING: This will stop ALL ROS2 processes and delete ALL build artifacts!
#
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_PATH="/mnt/hgfs/ROS2_PROJECT/workspace"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  RobotStudio - Complete ROS2 Cleanup${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if workspace exists
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo -e "${RED}ERROR: Workspace not found at $WORKSPACE_PATH${NC}"
    exit 1
fi

echo -e "${YELLOW}WARNING: This will:${NC}"
echo "  1. Kill all running ROS2 nodes"
echo "  2. Kill ROS2 daemon"
echo "  3. Delete build/, install/, log/ folders"
echo "  4. Clean up lingering ROS2 processes"
echo ""

# Check for packages in src/
if [ -d "$WORKSPACE_PATH/src" ]; then
    PACKAGE_COUNT=$(find "$WORKSPACE_PATH/src" -maxdepth 1 -type d | tail -n +2 | wc -l)
    if [ "$PACKAGE_COUNT" -gt 0 ]; then
        echo -e "${YELLOW}Found $PACKAGE_COUNT package(s) in src/:${NC}"
        ls -1 "$WORKSPACE_PATH/src"
        echo ""
        read -p "Do you want to delete ALL packages in src/? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            DELETE_SRC=true
            echo -e "${YELLOW}  Will delete src/ folder and all packages${NC}"
        else
            DELETE_SRC=false
            echo -e "${BLUE}  Will keep src/ folder${NC}"
        fi
    else
        DELETE_SRC=false
    fi
fi

echo ""
read -p "Continue with cleanup? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Aborted by user${NC}"
    exit 0
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Step 1: Killing All ROS2 Nodes${NC}"
echo -e "${BLUE}========================================${NC}"

# Kill all ROS2 nodes (from workspace packages)
echo "Killing workspace nodes..."
pkill -f "ros2 run" || echo "  No 'ros2 run' processes found"
pkill -f "python3.*my_weather" || echo "  No weather nodes found"
pkill -f "light_sensor" || echo "  No light_sensor found"
pkill -f "humidity_sensor" || echo "  No humidity_sensor found"
pkill -f "pressure_sensor" || echo "  No pressure_sensor found"
pkill -f "temperature_sensor" || echo "  No temperature_sensor found"

sleep 1  # Give processes time to die

# Force kill if still running
echo "Force killing any remaining nodes..."
pkill -9 -f "ros2 run" 2>/dev/null || true
pkill -9 -f "python3.*my_weather" 2>/dev/null || true

echo -e "${GREEN}✓ All ROS2 nodes stopped${NC}"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Step 2: Killing ROS2 Daemon${NC}"
echo -e "${BLUE}========================================${NC}"

# Stop ROS2 daemon
echo "Stopping ROS2 daemon..."
ros2 daemon stop 2>/dev/null || echo "  Daemon not running"

# Kill daemon processes
pkill -f "ros2 daemon" 2>/dev/null || echo "  No daemon processes found"
pkill -f "_ros2_daemon" 2>/dev/null || true

echo -e "${GREEN}✓ ROS2 daemon stopped${NC}"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Step 3: Cleaning Up Discovery Processes${NC}"
echo -e "${BLUE}========================================${NC}"

# Kill DDS/Discovery processes
echo "Killing DDS discovery processes..."
pkill -f "discovery" 2>/dev/null || echo "  No discovery processes found"
pkill -f "dds" 2>/dev/null || echo "  No DDS processes found"

echo -e "${GREEN}✓ Discovery processes stopped${NC}"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Step 4: Deleting Build Artifacts${NC}"
echo -e "${BLUE}========================================${NC}"

cd "$WORKSPACE_PATH"

# Delete build artifacts
if [ -d "build" ]; then
    echo "Deleting build/..."
    rm -rf build
    echo -e "${GREEN}✓ build/ deleted${NC}"
else
    echo "  build/ does not exist"
fi

if [ -d "install" ]; then
    echo "Deleting install/..."
    rm -rf install
    echo -e "${GREEN}✓ install/ deleted${NC}"
else
    echo "  install/ does not exist"
fi

if [ -d "log" ]; then
    echo "Deleting log/..."
    rm -rf log
    echo -e "${GREEN}✓ log/ deleted${NC}"
else
    echo "  log/ does not exist"
fi

# Delete src/ if user requested
if [ "$DELETE_SRC" = true ] && [ -d "src" ]; then
    echo "Deleting src/ and all packages..."
    rm -rf src
    echo -e "${GREEN}✓ src/ deleted (all packages removed)${NC}"
fi

echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Step 5: Cleaning Temporary Files${NC}"
echo -e "${BLUE}========================================${NC}"

# Clean up log files in home directory
echo "Cleaning log files in home directory..."
rm -f ~/robotstudio_*.log 2>/dev/null && echo -e "${GREEN}✓ Log files deleted${NC}" || echo "  No log files found"

# Clean up Python cache
echo "Cleaning Python cache..."
find "$WORKSPACE_PATH/src" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null && echo -e "${GREEN}✓ Python cache cleaned${NC}" || echo "  No cache found"
find "$WORKSPACE_PATH/src" -type f -name "*.pyc" -delete 2>/dev/null || true

echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Step 6: Verifying Cleanup${NC}"
echo -e "${BLUE}========================================${NC}"

# Check for remaining ROS2 processes
echo "Checking for remaining ROS2 processes..."
REMAINING=$(pgrep -f "ros2" | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo -e "${GREEN}✓ No ROS2 processes running${NC}"
else
    echo -e "${YELLOW}⚠ Warning: $REMAINING ROS2 process(es) still running${NC}"
    echo "  Run 'ps aux | grep ros2' to see details"
fi

# Check workspace is clean
echo "Checking workspace..."
if [ ! -d "$WORKSPACE_PATH/build" ] && [ ! -d "$WORKSPACE_PATH/install" ] && [ ! -d "$WORKSPACE_PATH/log" ]; then
    echo -e "${GREEN}✓ Workspace is clean${NC}"
else
    echo -e "${YELLOW}⚠ Warning: Some build artifacts may remain${NC}"
fi

echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✓ Cleanup Complete!${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Generate code in RobotStudio (Ctrl+G)"
echo "  2. Build workspace (Ctrl+B)"
echo "  3. Run nodes (Ctrl+R)"
echo ""
echo -e "${GREEN}Your ROS2 environment is now clean and ready!${NC}"
