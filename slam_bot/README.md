# Setup Local Development Server (Ubuntu 24.04 with ROS2 Jazzy)

- Install Docker

- Build Docker Image

  - > docker build -t slam-bot .

  - > docker run -d --name slam_bot -p 5901:5901 slam-bot

- Open TigerVNC and connect to terminal localhost:5091 with **password = "password"**
  - Or run (MacOS specific VNC viewer):
    > open vnc://localhost:5901
