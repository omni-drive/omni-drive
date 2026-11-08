#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEPLOY_DIR="$SCRIPT_DIR"
VENV_DIR="$DEPLOY_DIR/.venv"
PYTHON_BIN="$VENV_DIR/bin/python"
LAB_DIR="$PROJECT_ROOT/lab"
LAB_PYTHON="$LAB_DIR/.venv/bin/python"
ROBOT_USER="${SUDO_USER:-$USER}"

# Ensure local and system bin are in PATH for uv
export PATH="/usr/local/bin:$HOME/.local/bin:/home/${ROBOT_USER}/.local/bin:$PATH"

SERVICES=(
  esp-proxy.service
  lidar-proxy.service
  joystick-server.service
  jupyter-lab.service
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

echo "Synchronizing backend dependencies with 'uv sync'..."
if [ -n "${SUDO_USER:-}" ]; then
  sudo -u "$SUDO_USER" env PATH="$PATH" uv sync --directory "$DEPLOY_DIR"
else
  uv sync --directory "$DEPLOY_DIR"
fi

if [ ! -f "$PYTHON_BIN" ]; then
  echo "Error: Python executable not found at $PYTHON_BIN."
  exit 1
fi

echo "Synchronizing lab dependencies with 'uv sync'..."
if [ -n "${SUDO_USER:-}" ]; then
  sudo -u "$SUDO_USER" env PATH="$PATH" uv sync --directory "$LAB_DIR"
else
  uv sync --directory "$LAB_DIR"
fi
chmod -R go=rX "$LAB_DIR/.venv" 2>/dev/null || true

if [ ! -f "$LAB_PYTHON" ]; then
  echo "Error: Lab Python executable not found at $LAB_PYTHON."
  exit 1
fi

# Register single 'python3' kernel pointing to lab virtualenv
echo "Registering 'python3' Jupyter kernel..."
"$LAB_PYTHON" -m ipykernel install \
  --prefix "$LAB_DIR/.venv" \
  --name python3 \
  --display-name "Python 3 (OmniDrive)" \
  --env BOKEH_RESOURCES inline >/dev/null 2>&1 || true

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
  local exec_start="$3"
  local user="${4:-$ROBOT_USER}"
  local work_dir="${5:-$DEPLOY_DIR}"

  tee "/etc/systemd/system/${service_name}.service" > /dev/null <<EOF
[Unit]
Description=${description}
After=network.target

[Service]
Type=simple
User=${user}
WorkingDirectory=${work_dir}
Environment="PYTHONUNBUFFERED=1"
Environment="BOKEH_RESOURCES=inline"
UMask=0000
ExecStart=${exec_start}
Restart=always
RestartSec=2
StandardOutput=journal
StandardError=journal
SyslogIdentifier=omni-${service_name}

[Install]
WantedBy=multi-user.target
EOF
}

echo "Generating and installing systemd services..."
create_service "esp-proxy" "OmniDrive ESP32 Proxy" "${PYTHON_BIN} -m services.esp_proxy"
create_service "lidar-proxy" "OmniDrive LiDAR Proxy" "${PYTHON_BIN} -m services.lidar_proxy"
create_service "joystick-server" "OmniDrive Joystick WebSocket Server" "${PYTHON_BIN} -m services.joystick_server"
create_service "jupyter-lab" "OmniDrive JupyterLab Server for Students" \
  "${LAB_PYTHON} -m jupyterlab --ip=127.0.0.1 --port=8888 --no-browser --ServerApp.token='' --ServerApp.password='' --ServerApp.allow_remote_access=True --ServerApp.root_dir=/home/guest/lab --LabApp.default_url=/lab/tree/notebook.ipynb" \
  "guest" \
  "/home/guest/lab"

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
CHECK_SERVICES=(
  esp-proxy.service
  lidar-proxy.service
  joystick-server.service
  jupyter-lab.service
  nginx.service
)

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
echo "  • Web IDE (JupyterLab):   http://omnidrive.local/"
echo "  • Remote Control:        http://omnidrive.local/joystick/"
echo "  • WebSocket Telemetry:   ws://omnidrive.local/ws"
echo "=================================================="
echo ""
