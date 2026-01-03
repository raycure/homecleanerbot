#!/bin/bash

# HomeCleanerBot Docker Setup Script
# Tüm gerekli dosyaları oluşturur

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "========================================="
echo "  HomeCleanerBot Docker Setup"
echo "========================================="
echo ""

cd ~/HomeCleanerBot

# 1. Dockerfile
echo -e "${YELLOW}[1/6]${NC} Dockerfile oluşturuluyor..."
cat > Dockerfile << 'EOF'
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

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y \
    build-essential cmake git wget curl nano vim \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    xterm lsb-release gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-robot-localization \
    ros-humble-teleop-twist-keyboard ros-humble-tf2-tools \
    ros-humble-tf-transformations ros-humble-rqt* ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y gz-garden && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-humble-ros-gzgarden && rm -rf /var/lib/apt/lists/* || \
    (apt-get update && apt-get install -y ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ros-gz-interfaces && rm -rf /var/lib/apt/lists/*)

RUN apt-get update && apt-get install -y \
    python3-pyqt5 python3-pyqt5.qtsvg libqt5svg5-dev \
    x11-apps mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy transforms3d PyYAML

WORKDIR /root/HomeCleanerBot

COPY simple_coverage_planner/package.xml /root/HomeCleanerBot/simple_coverage_planner/ 2>/dev/null || true
COPY simple_coverage_planner/setup.py /root/HomeCleanerBot/simple_coverage_planner/ 2>/dev/null || true
COPY simple_coverage_planner/setup.cfg /root/HomeCleanerBot/simple_coverage_planner/ 2>/dev/null || true

RUN apt-get update && rosdep update && \
    rosdep install --from-paths simple_coverage_planner --ignore-src -r -y 2>/dev/null || true && \
    rm -rf /var/lib/apt/lists/*

COPY . /root/HomeCleanerBot/

RUN find /root/HomeCleanerBot -type f -name "*.py" -exec chmod +x {} \; && \
    find /root/HomeCleanerBot -type f -name "*.sh" -exec chmod +x {} \;

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /root/HomeCleanerBot/simple_coverage_planner && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release" || true

ENV GZ_SIM_RESOURCE_PATH=/root/HomeCleanerBot:/root/HomeCleanerBot/models:/usr/share/gazebo
ENV IGN_GAZEBO_RESOURCE_PATH=/root/HomeCleanerBot:/root/HomeCleanerBot/models:/usr/share/gazebo

RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/HomeCleanerBot/simple_coverage_planner/install/setup.bash 2>/dev/null || true\n\
export GZ_SIM_RESOURCE_PATH=/root/HomeCleanerBot:/root/HomeCleanerBot/models:/usr/share/gazebo\n\
export ROS_DOMAIN_ID=0\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/HomeCleanerBot/simple_coverage_planner/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

WORKDIR /root/HomeCleanerBot

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
EOF
echo -e "${GREEN}✓ Dockerfile oluşturuldu${NC}"

# 2. .dockerignore
echo -e "${YELLOW}[2/6]${NC} .dockerignore oluşturuluyor..."
cat > .dockerignore << 'EOF'
simple_coverage_planner/build/
simple_coverage_planner/install/
simple_coverage_planner/log/
**/__pycache__/
**/*.pyc
.vscode/
.git/
*.log
EOF
echo -e "${GREEN}✓ .dockerignore oluşturuldu${NC}"

# 3. build_docker.sh
echo -e "${YELLOW}[3/6]${NC} build_docker.sh oluşturuluyor..."
cat > build_docker.sh << 'EOF'
#!/bin/bash
set -e
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "========================================="
echo "  HomeCleanerBot Docker Build Script"
echo "========================================="
echo ""

if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Docker kurulu değil!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Docker bulundu${NC}"
echo -e "${YELLOW}📦 Docker imajı build ediliyor...${NC}"
echo ""

docker build --tag homecleanerbot:latest --progress=plain .

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}=========================================${NC}"
    echo -e "${GREEN}✓ Build başarılı!${NC}"
    echo -e "${GREEN}=========================================${NC}"
    echo ""
    docker images homecleanerbot:latest
else
    echo -e "${RED}❌ Build başarısız!${NC}"
    exit 1
fi
EOF
chmod +x build_docker.sh
echo -e "${GREEN}✓ build_docker.sh oluşturuldu${NC}"

# 4. run_docker.sh
echo -e "${YELLOW}[4/6]${NC} run_docker.sh oluşturuluyor..."
cat > run_docker.sh << 'EOF'
#!/bin/bash
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

MODE=${1:-navigation}

case $MODE in
    mapping|map|m)
        LAUNCH_FILE="launch/mapping.launch.py"
        CONTAINER_NAME="homecleanerbot-mapping"
        echo -e "${BLUE}🗺️  Mapping Mode${NC}"
        ;;
    navigation|nav|n)
        LAUNCH_FILE="launch/full_system.launch.py"
        CONTAINER_NAME="homecleanerbot-navigation"
        echo -e "${BLUE}🤖 Navigation Mode${NC}"
        ;;
    bash|shell|terminal)
        LAUNCH_FILE=""
        CONTAINER_NAME="homecleanerbot-shell"
        echo -e "${BLUE}💻 Interactive Shell${NC}"
        ;;
    *)
        echo -e "${RED}❌ Geçersiz mode: $MODE${NC}"
        echo "Kullanım: ./run_docker.sh [mapping|navigation|bash]"
        exit 1
        ;;
esac

echo ""
echo -e "${YELLOW}🔓 X11 izinleri ayarlanıyor...${NC}"
xhost +local:docker > /dev/null 2>&1

if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    echo -e "${YELLOW}🧹 Eski container temizleniyor...${NC}"
    docker rm -f ${CONTAINER_NAME} > /dev/null 2>&1
fi

echo -e "${GREEN}✓ Hazır${NC}"
echo -e "${YELLOW}🚀 Container başlatılıyor...${NC}"
echo ""

if [ "$LAUNCH_FILE" = "" ]; then
    docker run -it --rm \
        --name ${CONTAINER_NAME} \
        --network host \
        --privileged \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume $HOME/.Xauthority:/root/.Xauthority:rw \
        --volume /dev/shm:/dev/shm \
        --device /dev/dri \
        homecleanerbot:latest \
        bash
else
    docker run -it --rm \
        --name ${CONTAINER_NAME} \
        --network host \
        --privileged \
        --env DISPLAY=$DISPLAY \
        --env QT_X11_NO_MITSHM=1 \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume $HOME/.Xauthority:/root/.Xauthority:rw \
        --volume /dev/shm:/dev/shm \
        --device /dev/dri \
        homecleanerbot:latest \
        ros2 launch ${LAUNCH_FILE}
fi

echo ""
echo -e "${YELLOW}🧹 Temizleniyor...${NC}"
xhost -local:docker > /dev/null 2>&1
echo -e "${GREEN}✓ Tamamlandı${NC}"
EOF
chmod +x run_docker.sh
echo -e "${GREEN}✓ run_docker.sh oluşturuldu${NC}"

# 5. test_docker.sh
echo -e "${YELLOW}[5/6]${NC} test_docker.sh oluşturuluyor..."
cat > test_docker.sh << 'EOF'
#!/bin/bash
set -e
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "========================================="
echo "  HomeCleanerBot Docker Test"
echo "========================================="
echo ""

echo -e "${YELLOW}[1/3]${NC} Docker kontrol ediliyor..."
if command -v docker &> /dev/null; then
    echo -e "${GREEN}✓ Docker kurulu${NC}"
else
    echo -e "${RED}✗ Docker bulunamadı!${NC}"
    exit 1
fi

echo -e "${YELLOW}[2/3]${NC} X11 display kontrol ediliyor..."
if [ -z "$DISPLAY" ]; then
    echo -e "${RED}✗ DISPLAY değişkeni ayarlı değil${NC}"
    exit 1
else
    echo -e "${GREEN}✓ DISPLAY = $DISPLAY${NC}"
fi

echo -e "${YELLOW}[3/3]${NC} Docker image kontrol ediliyor..."
if docker images | grep -q "homecleanerbot"; then
    echo -e "${GREEN}✓ homecleanerbot image mevcut${NC}"
else
    echo -e "${YELLOW}⚠ Image bulunamadı, build edilmeli${NC}"
    echo "Build için: ./build_docker.sh"
fi

echo ""
echo -e "${GREEN}✓ Test tamamlandı!${NC}"
EOF
chmod +x test_docker.sh
echo -e "${GREEN}✓ test_docker.sh oluşturuldu${NC}"

# 6. Makefile
echo -e "${YELLOW}[6/6]${NC} Makefile oluşturuluyor..."
cat > Makefile << 'EOF'
.PHONY: help build run run-mapping run-shell stop clean

help:
	@echo "HomeCleanerBot Docker Commands"
	@echo "================================"
	@echo "make build         - Build Docker image"
	@echo "make run           - Run navigation mode"
	@echo "make run-mapping   - Run mapping mode"
	@echo "make run-shell     - Open interactive shell"
	@echo "make stop          - Stop all containers"
	@echo "make clean         - Remove containers and images"

build:
	@./build_docker.sh

run:
	@./run_docker.sh navigation

run-mapping:
	@./run_docker.sh mapping

run-shell:
	@./run_docker.sh bash

stop:
	@docker stop $$(docker ps -q --filter name=homecleanerbot) 2>/dev/null || true

clean:
	@docker rm -f $$(docker ps -aq --filter name=homecleanerbot) 2>/dev/null || true
	@docker rmi homecleanerbot:latest 2>/dev/null || true
EOF
echo -e "${GREEN}✓ Makefile oluşturuldu${NC}"

echo ""
echo "========================================="
echo -e "${GREEN}✓ Tüm dosyalar oluşturuldu!${NC}"
echo "========================================="
echo ""
echo "Şimdi build edebilirsiniz:"
echo "  ./build_docker.sh"
echo ""
echo "veya:"
echo "  make build"
