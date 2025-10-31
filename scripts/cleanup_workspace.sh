#!/bin/bash
# Cleanup ROS2 Workspace - Remove all build artifacts
# Run this on Ubuntu VM

echo "========================================"
echo "ROS2 Workspace Cleanup Script"
echo "========================================"
echo ""
echo "This will remove all build artifacts from:"
echo "  - /mnt/hgfs/ROS2_PROJECT/workspace/build/"
echo "  - /mnt/hgfs/ROS2_PROJECT/workspace/install/"
echo "  - /mnt/hgfs/ROS2_PROJECT/workspace/log/"
echo ""
echo "WARNING: This cannot be undone!"
echo ""
read -p "Press Enter to continue or Ctrl+C to cancel..."

cd /mnt/hgfs/ROS2_PROJECT/workspace

echo ""
echo "[1/2] Removing build artifacts..."
echo ""

# Remove directories
rm -rf build install log

if [ $? -eq 0 ]; then
    echo "  ✓ build/ removed"
    echo "  ✓ install/ removed"
    echo "  ✓ log/ removed"
else
    echo "  ✗ Failed to remove directories"
    exit 1
fi

echo ""
echo "[2/2] Killing any running ROS2 nodes..."
echo ""

# Kill running nodes
pkill -f "ros2 run" 2>/dev/null || true
pkill -f light_sensor 2>/dev/null || true
pkill -f humidity_sensor 2>/dev/null || true
pkill -f weather 2>/dev/null || true

echo "  ✓ All ROS2 nodes killed"

echo ""
echo "========================================"
echo "Cleanup Complete!"
echo "========================================"
echo ""
echo "Your workspace is now clean. Next steps:"
echo "  1. Build workspace: colcon build"
echo "  2. Source workspace: source install/setup.bash"
echo "  3. Run nodes: ros2 run <package> <node>"
echo ""
