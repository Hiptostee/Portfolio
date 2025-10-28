# Use the official Ubuntu 24.04 image
FROM ubuntu:24.04

# Set up environment variables for non-interactive install
ENV DEBIAN_FRONTEND=noninteractive

# --- STEP 1: Install basic dependencies, XFCE/VNC, and locales (as root) ---
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    build-essential \
    wget \
    git \
    openssh-client \
    tigervnc-standalone-server \
    tigervnc-common \
    tigervnc-tools \
    xfce4 \
    xfce4-goodies \
    xterm \
    twm \
    dbus-x11 \
    locales \
    xfonts-base \
    xfonts-75dpi \
    xfonts-100dpi \
    --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# --- STEP 2: Set up locale, add ROS 2 repo, and install ROS 2 Jazzy (as root) ---
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS 2 repository and key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update

# Install ROS 2 Jazzy (includes rviz2) and python tools
RUN apt-get install -y \
    ros-jazzy-desktop \
    python3-rosdep \
    python3-colcon-common-extensions && \
    rosdep init && \
    rosdep update

# --- STEP 3: Create non-root user (as root) ---
ENV USER_NAME=user
RUN useradd -m -s /bin/bash $USER_NAME

# --- Set up TigerVNC password and startup script ---
RUN mkdir -p /home/$USER_NAME/.vnc && \
    echo "password" | /usr/bin/vncpasswd -f > /home/$USER_NAME/.vnc/passwd && \
    chmod 600 /home/$USER_NAME/.vnc/passwd

# VNC startup script - simple window manager that works
RUN echo '#!/bin/bash' > /home/$USER_NAME/.vnc/xstartup && \
    echo 'unset SESSION_MANAGER' >> /home/$USER_NAME/.vnc/xstartup && \
    echo 'unset DBUS_SESSION_BUS_ADDRESS' >> /home/$USER_NAME/.vnc/xstartup && \
    echo 'export XKL_XMODMAP_DISABLE=1' >> /home/$USER_NAME/.vnc/xstartup && \
    echo 'xsetroot -solid grey' >> /home/$USER_NAME/.vnc/xstartup && \
    echo 'xterm -geometry 120x32+50+50 -ls -title "$VNCDESKTOP Desktop" &' >> /home/$USER_NAME/.vnc/xstartup && \    
    echo 'exec twm' >> /home/$USER_NAME/.vnc/xstartup && \
    chmod +x /home/$USER_NAME/.vnc/xstartup

# Change ownership of the entire .vnc directory to the user
RUN chown -R $USER_NAME:$USER_NAME /home/$USER_NAME/.vnc

# Copy and use the custom startup script (as root)
COPY start.sh /home/$USER_NAME/start.sh
RUN chmod +x /home/$USER_NAME/start.sh && chown $USER_NAME:$USER_NAME /home/$USER_NAME/start.sh

# --- STEP 4: Complete ROS setup (Switching to non-root user) ---
USER $USER_NAME
ENV HOME=/home/$USER_NAME

# Set up the shell for ROS 2
RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc

# Expose VNC port (default is 5901 for display :1)
EXPOSE 5901

# Start the VNC server using the custom script
CMD ["/home/user/start.sh"]