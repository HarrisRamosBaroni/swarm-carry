FROM althack/ros2:jazzy-gazebo 

# ** [Optional] Uncomment this section to install additional packages. **

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      python3-scipy python3-numpy \
      build-essential liblapack-dev libopenblas-dev python3-dev python3-venv \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-gz-ros2-control \
      # VNC server and desktop environment
      tigervnc-standalone-server tigervnc-common tigervnc-tools \
      xfce4 xfce4-goodies \
      # D-Bus and session management
      dbus-x11 \
      # noVNC for optional web access
      novnc websockify \
      # Utilities
      supervisor \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# Create virtual environment
COPY requirements.txt .
RUN python3 -m venv /opt/venv \
   && /opt/venv/bin/pip install --upgrade pip \
#    && /opt/venv/bin/pip install slycot control
   && /opt/venv/bin/pip install -r requirements.txt

# Make venv accessible to ROS
ENV PATH="/opt/venv/bin:${PATH}"
ENV PYTHONPATH="/opt/venv/lib/python3.12/site-packages:${PYTHONPATH}"

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Set up a directory for runtime files dump which is writable by the ros user
RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-ros" >> /home/ros/.bashrc && \
    echo "mkdir -p \$XDG_RUNTIME_DIR" >> /home/ros/.bashrc && \
    echo "chmod 700 \$XDG_RUNTIME_DIR" >> /home/ros/.bashrc && \
    echo "export DISPLAY=:1" >> /home/ros/.bashrc

# Set up VNC server for ros user
RUN mkdir -p /home/ros/.vnc && \
    echo "#!/bin/sh" > /home/ros/.vnc/xstartup && \
    echo "unset SESSION_MANAGER" >> /home/ros/.vnc/xstartup && \
    echo "unset DBUS_SESSION_BUS_ADDRESS" >> /home/ros/.vnc/xstartup && \
    echo "# Start D-Bus session" >> /home/ros/.vnc/xstartup && \
    echo "eval \$(dbus-launch --sh-syntax --exit-with-session)" >> /home/ros/.vnc/xstartup && \
    echo "exec startxfce4" >> /home/ros/.vnc/xstartup && \
    chmod +x /home/ros/.vnc/xstartup && \
    chown -R ros:ros /home/ros/.vnc

# VNC password (default: "vncpass" - change for production)
RUN echo "vncpass" | /usr/bin/vncpasswd -f > /home/ros/.vnc/passwd && \
    chmod 600 /home/ros/.vnc/passwd && \
    chown ros:ros /home/ros/.vnc/passwd

# Expose VNC and noVNC ports
EXPOSE 5901 6080

# Switch to ros user by default
USER ros