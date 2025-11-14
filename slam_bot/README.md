# 🧠 ROS 2 Jazzy Development & Deployment Guide

_(Cross-Platform: macOS / Windows / Linux → Raspberry Pi 5 Ubuntu 24.04 Server)_

---

## ⚙️ Phase 1 – Local Development Environment (Docker + ROS 2 Jazzy)

### 1️⃣ Install Docker

- **macOS / Windows:** Install **Docker Desktop** from [docker.com](https://www.docker.com/products/docker-desktop) and ensure it’s running.
- **Linux (Ubuntu 22.04 +)**:
  ```bash
  sudo apt update && sudo apt install -y docker.io docker-compose
  sudo systemctl enable docker
  sudo usermod -aG docker $USER
  newgrp docker
  ```

### 2️⃣ Clone and Build the Docker Image

In your project root:

```bash
docker build -t slam-bot .
```

### 3️⃣ Run the Container (on any OS)

Adjust the volume path (`-v`) to your system’s workspace location.

#### macOS / Linux

```bash
docker run -d --name slam_bot --restart unless-stopped --privileged --device /dev/net/tun -p 5901:5901 -p 2222:22 -v /Users/josephmarra/GitHub/Portfolio/slam_bot:/home/slambot/ros2_ws --user 0 slam-bot
```

#### Windows (PowerShell)

```powershell
docker run -d `
  --name slam_bot `
  --restart unless-stopped `
  -p 5901:5901 `
  -p 2222:22 `
  -v "/PATH_TO_CODE:/home/slambot/ros2_ws" `
  --user 0 `
  slam-bot
```

### 4️⃣ Connect to the Container

- Use **TigerVNC**, **Remmina**, or **VNC Viewer**.
- Connect to: `localhost:5901`
- Password: `{YOUR_PASSWORD}` (from your Dockerfile)

---

## 🧠 Phase 2 – Raspberry Pi 5 ROS 2 Jazzy Setup (Ubuntu 24.04 Server)

### 1️⃣ Flash Ubuntu 24.04 Server to SD Card

- Download: [Raspberry Pi Imager](https://www.raspberrypi.com/software/).
- Open **Raspberry Pi Imager** →
  - **OS:** “Other general-purpose OS → Ubuntu Server 24.04 (64-bit)”
  - **Storage:** your SD card
  - **⚙️ Advanced options:**
    - ✅ Set hostname: `slambot`
    - ✅ Enable SSH → Use password authentication
    - ✅ Set username / password (update [Dockerfile](/Dockerfile) lines 49-50)
    - ✅ Configure Wi-Fi (SSID + password)
    - ✅ Set locale/timezone

Eject and insert into the Pi.

---

## 🌐 Phase 3 – Connect the Pi to Your Network

1. **Boot the Pi** and wait ~1 minute for Wi-Fi to connect.
2. **Find its IP Address:**
   ```bash
   # macOS/Linux
   arp -a | grep slambot
   # or
   nmap -sn 192.168.1.0/24
   ```
   **Windows PowerShell:**
   ```powershell
   arp -a | findstr slambot
   ```

---

## 🔐 Phase 4 – SSH into the Pi

```bash
ssh slambot@<pi_ip_address>
```

Password: whatever you configured during flashing.

---

## 🧩 Phase 5 – Install ROS 2 Jazzy on the Pi

### 1️⃣ Set Locale

```bash
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

### 2️⃣ Add ROS 2 Sources

```bash
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### 3️⃣ Install ROS 2 Jazzy

```bash
sudo apt install ros-jazzy-desktop -y
```

### 4️⃣ Source Automatically

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🧰 Phase 6 – Sync and Deploy Code to the Pi

Add this function to your local shell config and source it (`~/.bashrc` or `~/.zshrc`):

```bash
uploadToPi() {
    if [ -z "$1" ]; then
        echo "Usage: uploadToPi <ip_address>"
        return 1
    fi

    echo "🔄 Syncing workspace to slambot@$1 (only changed files)..."

    rsync -avz
        --exclude 'build'
        --exclude 'install'
        --exclude 'log'
        --exclude '.git'         "
        $HOME/Github/Portfolio/slam_bot/"
        "slambot@$1:/home/slambot/ros2_ws_pi/"

    echo "✅ Sync complete."
    echo "Next steps on Pi:"
    echo "  cd ~/ros2_ws_pi"
    echo "  colcon build"
    echo "  source install/setup.bash"
}
```

Usage:

```bash
uploadToPi <pi_ip_address>
```

---

## 🚀 Phase 7 – Run Your Node on the Pi

```bash
cd ~/ros2_ws_pi
colcon build
source install/setup.bash
ros2 run <package_name> <node_executable>
```

---

## 🧭 Optional Quality-of-Life Enhancements

### Auto-source workspace

```bash
echo "source ~/ros2_ws_pi/install/setup.bash" >> ~/.bashrc
```

---

## ✅ Summary Overview

| Step                | Description                  |
| ------------------- | ---------------------------- |
| Flash Ubuntu 24.04  | Enable SSH + Wi-Fi in Imager |
| Boot & find IP      | `arp -a` or `nmap`           |
| SSH into Pi         | `ssh slambot@<ip>`           |
| Install ROS 2 Jazzy | via apt                      |
| Setup workspace     | `mkdir ~/ros2_ws_pi/src`     |
| Sync from computer  | `uploadToPi <ip>`            |
| Build on Pi         | `colcon build`               |
| Run node            | `ros2 run <pkg> <node>`      |
