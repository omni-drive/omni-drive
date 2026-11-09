# OmniDrive Deployment & Services

Backend hardware services, Nginx reverse proxy, and systemd configurations for the OmniDrive robot.

## Services Architecture
* `esp-proxy`: Serial/UART communication with ESP32 microcontroller over ZeroMQ IPC (`/tmp/esp_*.ipc`).
* `lidar-proxy`: Serial driver for Hokuyo LiDAR publishing scans over ZeroMQ IPC (`/tmp/lidar_*.ipc`).
* `joystick-server`: WebSocket server bridging remote control web client with robot control.
* `jupyter-lab`: Browser-accessible JupyterLab notebook server running under the unprivileged `guest` user.
* `nginx`: Reverse proxy serving the Web IDE (`/`), Joystick web app (`/joystick/`), and WebSocket telemetry (`/ws`).

## Provisioning & Deployment

### 1. One-Time System Provisioning (`setup_system.sh`)
Run once when configuring a new robot or OS image:
```bash
cd deploy
sudo ./setup_system.sh
```
This script:
* Installs required OS packages (`nginx`, `curl`).
* Installs `uv` package manager system-wide (`/usr/local/bin`) if not already present.
* Creates the unprivileged student account (`guest`) with no sudo privileges.
* Adds the developer user to the `dialout` group for serial port access.
* Sets permissions on `/home/<user>` to read-only for students (`r-x`).
* Prepares `/home/guest/lab` with the self-contained `notebook.ipynb`.
* Installs the global `lab-start` system command in `/usr/local/bin` to allow students to initialize a new timestamped notebook (`lab_<timestamp>.ipynb`) from the master template.

### 2. Service Deployment & Updates (`install.sh`)
Run to deploy or update services whenever repository code changes:
```bash
cd deploy
sudo ./install.sh
```
This script:
* Synchronizes backend and lab Python dependencies via `uv sync`.
* Registers the single `python3` Jupyter kernel pointing to the lab virtual environment.
* Automatically installs and starts systemd units (`esp-proxy`, `lidar-proxy`, `joystick-server`, `jupyter-lab`) with `UMask=0000` so ZeroMQ IPC sockets are accessible to `guest`.
* Deploys and reloads the Nginx configuration.
* Displays a live status table of all services and access URLs.

### 3. Build & Remote Deployment (`send_app.sh`)
Run on your development machine to compile Protobuf, build Flutter Web offline, and sync to the robot:
```bash
./send_app.sh [ROBOT_HOST] [ROBOT_PATH]
# Example: ./send_app.sh omni@omnidrive.local
```
This script:
* Generates Protobuf bindings via `gen_proto.sh`.
* Compiles Flutter Web with `--base-href /joystick/` and `--no-web-resources-cdn`.
* Synchronizes `deploy/services/generated/` and `remote_control/build/web/` to the robot using `rsync` (or `scp` fallback).
