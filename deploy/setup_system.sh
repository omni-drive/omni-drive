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
# 3. Install code-server if missing
# ==========================================
if ! command -v code-server >/dev/null 2>&1; then
  echo "Installing code-server..."
  curl -fsSL https://code-server.dev/install.sh | sh
else
  echo "code-server is already installed."
fi

# ==========================================
# 4. Create 'guest' user if missing
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

# ==========================================
# 5. Configure serial port permissions
# ==========================================
if id -nG "$ROBOT_USER" 2>/dev/null | grep -qw "dialout"; then
  echo "User '$ROBOT_USER' is already in 'dialout' group."
else
  echo "Adding '$ROBOT_USER' to 'dialout' group for serial port access..."
  usermod -aG dialout "$ROBOT_USER"
fi

# ==========================================
# 6. Lock down /home/$ROBOT_USER (read-only for others)
# ==========================================
echo "Securing /home/$ROBOT_USER permissions (read-only for guests)..."
chmod 755 "/home/$ROBOT_USER"
chmod -R u=rwX,go=rX "$PROJECT_ROOT"

# ==========================================
# 7. Initialize /home/guest workspace and code-server
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

  # Configure code-server for guest (auth: none, port 8080)
  GUEST_CODE_CONFIG="$GUEST_HOME/.config/code-server/config.yaml"
  mkdir -p "$(dirname "$GUEST_CODE_CONFIG")"
  cat <<EOF > "$GUEST_CODE_CONFIG"
bind-addr: 127.0.0.1:8080
auth: none
cert: false
EOF
  chown -R guest:guest "$GUEST_HOME/.config"

  # Pre-install Python and Jupyter extensions for guest
  echo "Pre-installing Python and Jupyter extensions for code-server..."
  sudo -H -u guest code-server --install-extension ms-python.python --install-extension ms-toolsai.jupyter >/dev/null 2>&1 || true

  # Configure default Python interpreter, dark theme, and disable workspace trust prompt for students
  GUEST_VSCODE_SETTINGS="$GUEST_HOME/.local/share/code-server/User/settings.json"
  mkdir -p "$(dirname "$GUEST_VSCODE_SETTINGS")"
  cat <<EOF > "$GUEST_VSCODE_SETTINGS"
{
  "workbench.colorTheme": "Default Dark Modern",
  "security.workspace.trust.enabled": false,
  "python.defaultInterpreterPath": "${PROJECT_ROOT}/lab/.venv/bin/python"
}
EOF

  GUEST_WORKSPACE_SETTINGS="$GUEST_LAB/.vscode/settings.json"
  mkdir -p "$(dirname "$GUEST_WORKSPACE_SETTINGS")"
  cp "$GUEST_VSCODE_SETTINGS" "$GUEST_WORKSPACE_SETTINGS"

  chown -R guest:guest "$GUEST_HOME"

  # Configure code-server to open /home/guest/lab by default via systemd override
  echo "Configuring code-server to open $GUEST_LAB by default..."
  mkdir -p /etc/systemd/system/code-server@guest.service.d
  cat <<EOF > /etc/systemd/system/code-server@guest.service.d/workspace.conf
[Service]
ExecStart=
ExecStart=/usr/bin/code-server /home/guest/lab
EOF

  # Enable and start code-server service for guest
  echo "Enabling code-server service on boot for user 'guest'..."
  systemctl daemon-reload
  systemctl restart code-server@guest 2>/dev/null || systemctl enable --now code-server@guest 2>/dev/null || true
fi

# ==========================================
# 8. Register global Jupyter kernel for OmniDrive
# ==========================================
LAB_DIR="$PROJECT_ROOT/lab"
LAB_PYTHON="$LAB_DIR/.venv/bin/python"

# Clean up broken virtualenv if directory exists but python executable is missing
if [ -d "$LAB_DIR/.venv" ] && [ ! -f "$LAB_PYTHON" ]; then
  echo "Cleaning up broken virtual environment at $LAB_DIR/.venv..."
  rm -rf "$LAB_DIR/.venv"
fi

if command -v uv >/dev/null 2>&1; then
  echo "Synchronizing lab environment with uv..."
  if [ -n "${SUDO_USER:-}" ]; then
    sudo -u "$SUDO_USER" env PATH="$PATH" uv sync --directory "$LAB_DIR"
  else
    uv sync --directory "$LAB_DIR"
  fi
  chmod -R go=rX "$LAB_DIR/.venv" 2>/dev/null || true
fi

if [ -f "$LAB_PYTHON" ]; then
  echo "Registering global 'omnidrive' Jupyter kernel..."
  "$LAB_PYTHON" -m ipykernel install \
    --prefix /usr/local \
    --name omnidrive \
    --display-name "Python (OmniDrive)" >/dev/null 2>&1 || true
  echo "Jupyter kernel registered: Python (OmniDrive) -> $LAB_PYTHON"
else
  echo "Warning: Python executable not found at $LAB_PYTHON. Jupyter kernel not registered."
fi

echo ""
echo "=================================================="
echo "System provisioning completed successfully!"
echo "You can now run './install.sh' to deploy the robot services."
echo "=================================================="
echo ""

