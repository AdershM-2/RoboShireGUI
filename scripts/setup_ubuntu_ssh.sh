#!/bin/bash
# Setup SSH on Ubuntu VM for RobotStudio Integration
# Run this script in your Ubuntu VM terminal

echo "=== Setting up SSH on Ubuntu VM ==="
echo ""

# Install SSH server if not already installed
echo "1. Installing OpenSSH server..."
sudo apt update
sudo apt install openssh-server -y

# Start SSH service
echo ""
echo "2. Starting SSH service..."
sudo systemctl start ssh
sudo systemctl enable ssh

# Check SSH status
echo ""
echo "3. Checking SSH status..."
sudo systemctl status ssh --no-pager

# Show connection information
echo ""
echo "=== SSH Setup Complete ==="
echo ""
echo "Connection details:"
echo "  Username: $(whoami)"
echo "  Hostname: $(hostname)"
echo "  IP Address: $(hostname -I | awk '{print $1}')"
echo ""
echo "Test from Windows PowerShell:"
echo "  ssh $(whoami)@$(hostname -I | awk '{print $1}')"
echo ""
echo "Or add to Windows hosts file (C:\\Windows\\System32\\drivers\\etc\\hosts):"
echo "  $(hostname -I | awk '{print $1}')  ubuntu"
echo ""
echo "Then you can use:"
echo "  ssh $(whoami)@ubuntu"
