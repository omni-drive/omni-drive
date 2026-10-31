#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEPLOY_DIR="$SCRIPT_DIR"
VENV_DIR="$DEPLOY_DIR/.venv"
PYTHON_BIN="$VENV_DIR/bin/python"
ROBOT_USER="${SUDO_USER:-$USER}"

# Ensure local and system bin are in PATH for uv
export PATH="/usr/local/bin:$HOME/.local/bin:/home/${ROBOT_USER}/.local/bin:$PATH"

SERVICES=(
  esp-proxy.service
  lidar-proxy.service
  joystick-server.service
)

echo "=== OmniDrive Robot Installation ==="

# Check root/sudo
if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  echo "Error: This script must be run with sudo: sudo ./install.sh"
  exit 1
fi

# ==========================================
# 1. Environment check and synchronization
# ==========================================
if ! command -v uv >/dev/null 2>&1; then
  echo "Error: 'uv' package manager is not installed."
  echo "Please run 'sudo ./setup_system.sh' first."
  exit 1
fi

echo "Synchronizing dependencies with 'uv sync'..."
if [ -n "${SUDO_USER:-}" ]; then
  sudo -u "$SUDO_USER" env PATH="$PATH" uv sync --directory "$DEPLOY_DIR"
else
  uv sync --directory "$DEPLOY_DIR"
fi

if [ ! -f "$PYTHON_BIN" ]; then
  echo "Error: Python executable not found at $PYTHON_BIN."
  exit 1
fi

if [ ! -f "$PROJECT_ROOT/remote_control/build/web/index.html" ]; then
  echo "Warning: Built web client not found at $PROJECT_ROOT/remote_control/build/web/index.html"
  echo "Route /joystick/ will not be available until remote_control is built."
fi

# ==========================================
# 2. Generate and install systemd services
# ==========================================
create_service() {
  local service_name="$1"
  local description="$2"
  local python_module="$3"

  tee "/etc/systemd/system/${service_name}.service" > /dev/null <<EOF
[Unit]
Description=${description}
After=network.target

[Service]
Type=simple
User=${ROBOT_USER}
WorkingDirectory=${DEPLOY_DIR}
Environment="PYTHONUNBUFFERED=1"
UMask=0000
ExecStart=${PYTHON_BIN} -m services.${python_module}
Restart=always
RestartSec=2
StandardOutput=journal
StandardError=journal
SyslogIdentifier=omni-${service_name}

[Install]
WantedBy=multi-user.target
EOF
}

echo "Generating and installing systemd services for user '$ROBOT_USER'..."
create_service "esp-proxy" "OmniDrive ESP32 Proxy" "esp_proxy"
create_service "lidar-proxy" "OmniDrive LiDAR Proxy" "lidar_proxy"
create_service "joystick-server" "OmniDrive Joystick WebSocket Server" "joystick_server"

systemctl daemon-reload
systemctl enable "${SERVICES[@]}"
systemctl restart "${SERVICES[@]}"

# ==========================================
# 3. Configure Nginx reverse proxy
# ==========================================
NGINX_TEMPLATE="$SCRIPT_DIR/nginx/omnidrive"
NGINX_TARGET="/etc/nginx/sites-available/omnidrive"

if command -v nginx >/dev/null 2>&1; then
  echo "Configuring Nginx reverse proxy..."
  sed "s|{{PROJECT_ROOT}}|${PROJECT_ROOT}|g" "$NGINX_TEMPLATE" | tee "$NGINX_TARGET" > /dev/null
  ln -sf "$NGINX_TARGET" /etc/nginx/sites-enabled/omnidrive
  rm -f /etc/nginx/sites-enabled/default

  if nginx -t >/dev/null 2>&1; then
    systemctl reload nginx || systemctl restart nginx
    echo "Nginx configured and reloaded successfully."
  else
    echo "Warning: Nginx configuration test failed."
  fi
else
  echo "Warning: Nginx is not installed. Please run 'sudo ./setup_system.sh' first."
fi

# ==========================================
# 4. Display service status summary
# ==========================================
CODE_SERVER_USER="guest"
if ! id guest >/dev/null 2>&1; then
  CODE_SERVER_USER="$ROBOT_USER"
fi

CHECK_SERVICES=(
  esp-proxy.service
  lidar-proxy.service
  joystick-server.service
  nginx.service
)

# Include code-server service in summary if present in systemd
if systemctl list-unit-files "code-server@${CODE_SERVER_USER}.service" 2>/dev/null | grep -q "code-server"; then
  CHECK_SERVICES+=("code-server@${CODE_SERVER_USER}.service")
elif systemctl list-unit-files "code-server.service" 2>/dev/null | grep -q "code-server"; then
  CHECK_SERVICES+=("code-server.service")
fi

# ANSI colors
GREEN="\e[32m"
YELLOW="\e[33m"
RED="\e[31m"
BOLD="\e[1m"
RESET="\e[0m"

printf "\n${BOLD}%-32s %-14s %s${RESET}\n" "SERVICE" "STATUS" "SUBSTATE"
printf "%-32s %-14s %s\n" "--------------------------------" "--------------" "--------"

for service in "${CHECK_SERVICES[@]}"; do
  status="$(systemctl is-active "$service" 2>/dev/null || echo "unknown")"
  substate="$(systemctl show -p SubState --value "$service" 2>/dev/null || echo "unknown")"

  if [[ "$status" == "active" ]]; then
    printf "%-32s ${GREEN}%-14s${RESET} (%s)\n" "$service" "● active" "$substate"
  elif [[ "$status" == "activating" ]]; then
    printf "%-32s ${YELLOW}%-14s${RESET} (%s)\n" "$service" "▲ activating" "$substate"
  else
    printf "%-32s ${RED}%-14s${RESET} (%s)\n" "$service" "○ $status" "$substate"
  fi
done

echo ""
echo "=================================================="
echo -e "${BOLD}OmniDrive robot installation completed!${RESET}"
echo ""
echo "Access URLs:"
echo "  • Web IDE (code-server):  http://omnidrive.local/"
echo "  • Remote Control:        http://omnidrive.local/joystick/"
echo "  • WebSocket Telemetry:   ws://omnidrive.local/ws"
echo "=================================================="
echo ""
