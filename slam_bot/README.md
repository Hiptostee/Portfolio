# Setup Local Development Server (Ubuntu 24.04 with ROS2 Jazzy)

- Install Docker

- Build Docker Image

  - > docker build -t slam-bot .

  - >docker run -d --name slam_bot --restart unless-stopped -p 5901:5901 -v {path_to_repo}:/home/slambot/ros2_ws --user 0 slam_bot

- Open TigerVNC and connect to terminal localhost:5091 with **password = "password"**
  - Or run (MacOS specific VNC viewer):
    > open vnc://localhost:5901

# 🧠 Raspberry Pi 5 ROS2 Jazzy Setup Guide (Ubuntu 24.04)

This assumes you want to develop locally on your Mac (or laptop with Docker), then deploy to your RPi5 to run natively.

---

## ⚙️ Phase 1 – Prepare the Raspberry Pi

### 1. Flash Ubuntu 24.04 Server (64-bit)
- Download **Ubuntu 24.04.1 LTS (Server for Raspberry Pi)** from Canonical.
- Use **Raspberry Pi Imager**:
  - OS → *Other general-purpose OS → Ubuntu Server 24.04 (64-bit)*
  - Storage → your SD card
  - Before writing:
    - ⚙️ Click the settings icon (advanced options)
    - Enable:
      - ✅ “Set hostname” → `rpi5`
      - ✅ “Enable SSH” → choose *Use password authentication*
      - ✅ “Set username and password” → e.g., `slambot` / `password`
      - ✅ “Configure wireless LAN” → enter SSID & password (e.g., `GTother`)
      - ✅ “Set locale settings” → your timezone and locale
  - Write and eject.

---

## 🌐 Phase 2 – Connect the Pi to the Network

### 2. Boot your Pi
- Insert the SD card and power it on.
- It’ll automatically connect to your configured Wi-Fi.

### 3. Find the Pi’s IP Address
On your Mac (same network):
```bash
arp -a | grep rpi5
```
or
```bash
nmap -sn 10.89.63.0/24
```
Find the IP (e.g., `10.89.63.120`).

---

## 🔐 Phase 3 – SSH into the Pi

```bash
ssh slambot@10.89.63.120
```
Password: whatever you set during flashing.

If you want passwordless login:
```bash
ssh-copy-id slambot@10.89.63.120
```

---

## 🧩 Phase 4 – Install ROS 2 Jazzy

### 1. Set locale
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

### 2. Add ROS 2 sources
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### 3. Install ROS 2 Jazzy
```bash
sudo apt install ros-jazzy-desktop
```

### 4. Source it automatically
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🧰 Phase 5 – Connect Development to Deployment

### 1. Add deploy function on your Mac
In your Mac’s `~/.zshrc` or `~/.bashrc`, add:

```bash
deployToPi() {
    if [ -z "$1" ]; then
        echo "Usage: deployToPi <ip_address>"
        return 1
    fi

    echo "🔄 Syncing workspace to slambot@$1..."
    rsync -avz         --exclude 'build' --exclude 'install' --exclude 'log' --exclude '.git'         "$HOME/Github/Portfolio/slam_bot/"         "slambot@$1:/home/slambot/ros2_ws_pi/"

    echo "⚙️ Building on Pi..."
    ssh slambot@$1 "cd ~/ros2_ws_pi && colcon build --symlink-install"

    echo "✅ Deploy complete! Run: ros2 run test_node test_node_node"
}
```

Usage:
```bash
deployToPi 10.89.63.120
```

---

## 🚀 Phase 6 – Run your Node on the Pi

```bash
cd ~/ros2_ws_pi
source install/setup.bash
ros2 run test_node test_node_node
```

---

## 🧭 Optional Quality-of-Life Improvements

- **Auto-sourcing**
  ```bash
  echo "source ~/ros2_ws_pi/install/setup.bash" >> ~/.bashrc
  ```

- **Headless startup**
  Add a systemd service to auto-launch your node on boot.

---

## ✅ Summary Overview

| Step | Description |
|------|--------------|
| Flash Ubuntu 24.04 | Enable SSH + Wi-Fi in Imager |
| Boot & find IP | `arp -a` or `nmap` |
| SSH into Pi | `ssh slambot@<ip>` |
| Install ROS 2 Jazzy | via apt |
| Setup workspace | `mkdir ~/ros2_ws_pi/src` |
| Sync from Mac | `deployToPi <ip>` |
| Build on Pi | `colcon build` |
| Run node | `ros2 run <pkg> <node>` |



