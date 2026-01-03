# HomeCleanerBot Docker Image
# Ubuntu 22.04 + ROS2 Humble + Gazebo Garden

FROM osrf/ros:humble-desktop-full

LABEL maintainer="homecleanerbot-team"
LABEL description="HomeCleanerBot - Autonomous Home Cleaning Robot with Gazebo Garden"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV PYTHONUNBUFFERED=1
ENV GZ_VERSION=garden

# Temel sistem güncellemesi
RUN apt-get update && apt-get upgrade -y

# Temel geliştirme araçları
RUN apt-get install -y \
    build-essential cmake git wget curl nano vim \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    xterm lsb-release gnupg \
    && rm -rf /var/lib/apt/lists/*

# ROS2 paketleri
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-rqt* \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Garden kurulumu
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y gz-garden && \
    rm -rf /var/lib/apt/lists/*

# Gazebo-ROS2 bridge (Garden için)
RUN apt-get update && \
    (apt-get install -y ros-humble-ros-gzgarden || \
     apt-get install -y ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ros-gz-interfaces) && \
    rm -rf /var/lib/apt/lists/*

# GUI için gerekli paketler
RUN apt-get update && apt-get install -y \
    python3-pyqt5 \
    python3-pyqt5.qtsvg \
    libqt5svg5-dev \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Python bağımlılıkları
RUN pip3 install --no-cache-dir numpy transforms3d PyYAML

# Workspace dizini oluştur
WORKDIR /root/HomeCleanerBot

# Tüm proje dosyalarını kopyala
COPY . /root/HomeCleanerBot/

# Script'leri executable yap
RUN find /root/HomeCleanerBot -type f -name "*.py" -exec chmod +x {} \; && \
    find /root/HomeCleanerBot -type f -name "*.sh" -exec chmod +x {} \;

# rosdep ile bağımlılıkları yükle
RUN if [ -f "simple_coverage_planner/package.xml" ]; then \
        apt-get update && \
        rosdep update && \
        rosdep install --from-paths simple_coverage_planner --ignore-src -r -y; \
        rm -rf /var/lib/apt/lists/*; \
    fi

# ROS2 workspace build
RUN if [ -d "simple_coverage_planner" ]; then \
        /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /root/HomeCleanerBot/simple_coverage_planner && \
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"; \
    fi

# Gazebo Garden environment variables
ENV GZ_SIM_RESOURCE_PATH=/root/HomeCleanerBot:/root/HomeCleanerBot/models:/usr/share/gazebo
ENV IGN_GAZEBO_RESOURCE_PATH=/root/HomeCleanerBot:/root/HomeCleanerBot/models:/usr/share/gazebo

# Environment setup script oluştur
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
if [ -f "/root/HomeCleanerBot/simple_coverage_planner/install/setup.bash" ]; then\n\
    source /root/HomeCleanerBot/simple_coverage_planner/install/setup.bash\n\
fi\n\
export GZ_SIM_RESOURCE_PATH=/root/HomeCleanerBot:/root/HomeCleanerBot/models:/usr/share/gazebo\n\
export IGN_GAZEBO_RESOURCE_PATH=/root/HomeCleanerBot:/root/HomeCleanerBot/models:/usr/share/gazebo\n\
export ROS_DOMAIN_ID=0\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

# .bashrc güncelle
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /root/HomeCleanerBot/simple_coverage_planner/install/setup.bash ]; then" >> ~/.bashrc && \
    echo "    source /root/HomeCleanerBot/simple_coverage_planner/install/setup.bash" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/HomeCleanerBot:/root/HomeCleanerBot/models:/usr/share/gazebo" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc && \
    echo "echo '🤖 HomeCleanerBot Docker Container (Gazebo Garden)'" >> ~/.bashrc && \
    echo "alias launch-nav='ros2 launch launch/full_system.launch.py'" >> ~/.bashrc && \
    echo "alias start-clean='ros2 service call /start_cleaning std_srvs/srv/Trigger \"{}\"'" >> ~/.bashrc && \
    echo "alias stop-clean='ros2 service call /stop_cleaning std_srvs/srv/Trigger \"{}\"'" >> ~/.bashrc

# Çalışma dizini
WORKDIR /root/HomeCleanerBot

# Entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command
CMD ["bash"]
