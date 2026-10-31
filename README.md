# OmniDrive

Public materials, laboratory notebooks, deployment configurations, and remote control client for **OmniDrive** — a three-wheel omnidirectional mobile robot designed for robotics laboratory classes.

The robot features:
* Three omni-directional wheels with kiwi drive kinematics,
* 2D Hokuyo LiDAR for planar scanning and wall positioning,
* IMU / gyroscope for angular velocity and heading estimation,
* Time-of-Flight (ToF) and ultrasonic distance sensors,
* ESP32 microcontroller handling low-level motor PID and sensor polling,
* Embedded Linux host running hardware proxies and web services, programmed in Python with the [`omni_drive`](https://github.com/omni-drive/omni-drive) SDK.

---

## Quick Start for Students

1. Connect your laptop or tablet to the robot's Wi-Fi network (e.g. `OmniDrive-XXX`).
2. Open your browser:
   * **Web IDE (Notebook):** `http://omnidrive.local/` (or the robot's IP address)
   * **Joystick Remote:** `http://omnidrive.local/joystick/`
3. In the Web IDE, open `notebook.ipynb` and follow the instructions.
4. If you ever need to reset the notebook to a clean initial state, open the terminal in the IDE and run:
   ```bash
   lab-start
   ```
   *(This automatically creates a timestamped backup of your current notebook and restores a fresh template).*

---

## System Architecture

```text
               +-----------------------------------------------------------+
               |                  Client (Laptop / Phone)                  |
               |   [ VS Code Web IDE ]      [ Flutter Web Joystick App ]   |
               +-----------------------------------------------------------+
                        | (HTTP & WebSockets)           | (WebSocket /ws)
                        v                               v
       +-------------------------------------------------------------------+
       |                       Nginx Reverse Proxy (Port 80)               |
       |  • /         -> proxy to code-server:8080                         |
       |  • /joystick -> static files (remote_control/build/web/)          |
       |  • /ws       -> proxy to joystick-server:8765                     |
       +-------------------------------------------------------------------+
                 |                                              |
                 v (:8080)                                      v (:8765)
       +-------------------+                          +--------------------+
       |    code-server    |                          |  joystick-server   |
       |   (user: guest)   |                          |    (user: omni)    |
       +-------------------+                          +--------------------+
                 | (runs Jupyter kernel)                        |
                 v                                              |
       +-------------------+                                    |
       |  notebook.ipynb   |                                    |
       | (omni_drive SDK)  |                                    |
       +-------------------+                                    |
                 |                                              |
                 +----------------------+-----------------------+
                                        | (omni_drive connects to IPC)
                                        v
     ========================================================================
                      ZeroMQ UNIX Domain Sockets (/tmp/*.ipc)
        • /tmp/esp_commands.ipc (SUB)     • /tmp/esp_telemetry.ipc (PUB)
        • /tmp/lidar_scans.ipc  (PUB)
     ========================================================================
                 ^                                              ^
                 |                                              |
       +-------------------+                          +--------------------+
       |     esp-proxy     |                          |    lidar-proxy     |
       |   (user: omni)    |                          |    (user: omni)    |
       +-------------------+                          +--------------------+
                 | (UART /dev/ttyUSB*)                          | (UART /dev/ttyUSB*)
                 v                                              v
       +-------------------+                          +--------------------+
       |   ESP32 Micro     |                          |   Hokuyo 2D LiDAR  |
       | (Motors & Sensors)|                          |   (Laser Scans)    |
       +-------------------+                          +--------------------+
```

* **Reverse Proxy (Nginx):** Unified entrypoint on port 80:
  * `/` &rarr; Reverse-proxies to `code-server` (`127.0.0.1:8080`).
  * `/joystick/` &rarr; Serves pre-built static Flutter Web assets directly from `remote_control/build/web/`.
  * `/ws` &rarr; Proxies WebSocket traffic to `joystick-server` (`127.0.0.1:8765`).
* **Decoupled IPC Bus (`/tmp/*.ipc`):**
  * Hardware daemons (`esp-proxy`, `lidar-proxy`) publish telemetry/scans and subscribe to drive commands over ZeroMQ UNIX domain sockets.
  * Generated with `UMask=0000` so that both the developer (`omni`) and student (`guest`) processes can interact with the robot without requiring root or group elevation.
* **Control Sources:**
  * Both student notebooks and the joystick server interact with the robot hardware through the same clean Python `omni_drive` SDK over ZeroMQ IPC.

---

## Deployment & Administration

### 1. One-Time System Provisioning (`setup_system.sh`)
Run once when configuring a fresh Raspberry Pi or Ubuntu/Debian OS image:
```bash
cd deploy
sudo ./setup_system.sh
```
* Installs required system packages (`nginx`, `curl`, `uv`, and `code-server`).
* Creates the unprivileged `guest` user (no sudo) and locks down repository access to read-only.
* Prepares `/home/guest/lab/` with the self-contained `notebook.ipynb` and installs `/usr/local/bin/lab-start`.
* Registers the global `Python (OmniDrive)` Jupyter kernel pointing to the master `.venv`.

### 2. Service Deployment & Updates (`install.sh`)
Run on the robot whenever services, code, or configurations are updated:
```bash
cd deploy
sudo ./install.sh
```
* Synchronizes service dependencies using `uv sync`.
* Generates and restarts systemd units (`esp-proxy`, `lidar-proxy`, `joystick-server`) with `UMask=0000` for unprivileged student IPC socket access.
* Deploys and reloads Nginx reverse proxy configuration and prints a live service status table.

### 3. Remote Build & Sync (`send_app.sh`)
Run on your development PC to build and push web/proto artifacts to the robot:
```bash
./send_app.sh [ROBOT_HOST] [ROBOT_PATH]
# Default target: omni@omnidrive.local (/home/omni/omni-drive)
```
* Compiles Protobuf definitions (`remote.proto`) for Python and Dart.
* Builds the Flutter Web client offline with local CanvasKit and fonts (`--base-href /joystick/`).
* Synchronizes `deploy/services/generated/` and `remote_control/build/web/` to the robot via `rsync`.

---

## Documentation

Library documentation is published at <https://omni-drive.readthedocs.io/>.

To preview documentation locally:
```bash
cd lab
uv run --group docs mkdocs serve -f ../docs/mkdocs.yml
```
