#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROBOT_USER="${SUDO_USER:-$USER}"

# Ensure local and system bin are in PATH for uv
export PATH="/usr/local/bin:$HOME/.local/bin:/home/${ROBOT_USER}/.local/bin:$PATH"

echo "=== OmniDrive One-Time System Provisioning ==="

# Check root/sudo
if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  echo "Error: This script must be run with sudo: sudo ./setup_system.sh"
  exit 1
fi

# ==========================================
# 1. Install system packages (Nginx, curl)
# ==========================================
echo "Installing required system packages (nginx, curl)..."
apt-get update -qq
apt-get install -y -qq nginx curl

# ==========================================
# 2. Install uv package manager if missing
# ==========================================
if ! command -v uv >/dev/null 2>&1; then
  echo "Installing 'uv' package manager to /usr/local/bin..."
  curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR="/usr/local/bin" sh
else
  echo "'uv' is already installed."
fi

# ==========================================
# 3. Create 'guest' user if missing
# ==========================================
if ! id guest >/dev/null 2>&1; then
  echo "Creating unprivileged 'guest' user for students..."
  useradd -m -s /bin/bash guest
  passwd -d guest >/dev/null 2>&1 || true
else
  echo "User 'guest' already exists."
fi

# Ensure guest does NOT have sudo access
gpasswd -d guest sudo >/dev/null 2>&1 || true

# Allow guest to safely power off the robot without password
echo "Allowing 'guest' user to run poweroff..."
cat << 'EOF' > /etc/sudoers.d/020-guest-poweroff
guest ALL=(ALL) NOPASSWD: /sbin/poweroff, /usr/sbin/poweroff, /sbin/reboot, /usr/sbin/reboot, /sbin/shutdown, /usr/sbin/shutdown, /bin/systemctl poweroff, /usr/bin/systemctl poweroff
EOF
chmod 0440 /etc/sudoers.d/020-guest-poweroff

# ==========================================
# 4. Configure serial port permissions
# ==========================================
if id -nG "$ROBOT_USER" 2>/dev/null | grep -qw "dialout"; then
  echo "User '$ROBOT_USER' is already in 'dialout' group."
else
  echo "Adding '$ROBOT_USER' to 'dialout' group for serial port access..."
  usermod -aG dialout "$ROBOT_USER"
fi

# ==========================================
# 5. Lock down /home/$ROBOT_USER (read-only for others)
# ==========================================
echo "Securing /home/$ROBOT_USER permissions (read-only for guests)..."
chmod 755 "/home/$ROBOT_USER"
chmod -R u=rwX,go=rX "$PROJECT_ROOT"

# ==========================================
# 6. Initialize /home/guest workspace and JupyterLab configuration
# ==========================================
GUEST_HOME="/home/guest"
GUEST_LAB="$GUEST_HOME/lab"

if [ -d "$GUEST_HOME" ]; then
  echo "Setting up student workspace in $GUEST_LAB..."
  mkdir -p "$GUEST_LAB"
  if [ -f "$PROJECT_ROOT/lab/notebook.ipynb" ]; then
    cp -n "$PROJECT_ROOT/lab/notebook.ipynb" "$GUEST_LAB/"
  fi

  # Install global lab-start command
  if [ -f "$SCRIPT_DIR/lab-start" ]; then
    echo "Installing global 'lab-start' command to /usr/local/bin..."
    sed "s|{{PROJECT_ROOT}}|${PROJECT_ROOT}|g" "$SCRIPT_DIR/lab-start" > /usr/local/bin/lab-start
    chmod 755 /usr/local/bin/lab-start
  fi

  chown -R guest:guest "$GUEST_HOME"
fi

echo ""
echo "=================================================="
echo "System provisioning completed successfully!"
echo "Run 'sudo ./install.sh' to deploy services and dependencies."
echo "=================================================="
echo ""

