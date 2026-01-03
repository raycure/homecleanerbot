#!/bin/bash

# Gazebo Garden Test Script
# Docker içinde Gazebo'yu test eder

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "========================================="
echo "  Gazebo Garden Test"
echo "========================================="
echo ""

# X11 izinlerini ayarla
xhost +local:docker > /dev/null 2>&1

echo -e "${BLUE}[1/4]${NC} Container başlatılıyor..."
docker run -d \
    --name gazebo-test \
    --network host \
    --privileged \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume /dev/shm:/dev/shm \
    --device /dev/dri \
    homecleanerbot:latest \
    sleep infinity

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Container başlatıldı${NC}"
else
    echo -e "${RED}✗ Container başlatılamadı!${NC}"
    exit 1
fi
echo ""

# Gazebo versiyonunu kontrol et
echo -e "${BLUE}[2/4]${NC} Gazebo versiyonu kontrol ediliyor..."
GZ_VERSION=$(docker exec gazebo-test gz sim --version 2>/dev/null || echo "ERROR")

if [[ "$GZ_VERSION" == *"Garden"* ]] || [[ "$GZ_VERSION" == *"7."* ]]; then
    echo -e "${GREEN}✓ Gazebo Garden kurulu${NC}"
    echo "$GZ_VERSION"
else
    echo -e "${RED}✗ Gazebo Garden bulunamadı!${NC}"
    echo "Bulunan: $GZ_VERSION"
fi
echo ""

# ROS-Gazebo bridge'i kontrol et
echo -e "${BLUE}[3/4]${NC} ros_gz_bridge kontrol ediliyor..."
BRIDGE_CHECK=$(docker exec gazebo-test bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep ros_gz" || echo "ERROR")

if [[ "$BRIDGE_CHECK" == *"ros_gz"* ]]; then
    echo -e "${GREEN}✓ ros_gz_bridge kurulu${NC}"
    echo "$BRIDGE_CHECK"
else
    echo -e "${RED}✗ ros_gz_bridge bulunamadı!${NC}"
fi
echo ""

# Gazebo'yu test et (5 saniye)
echo -e "${BLUE}[4/4]${NC} Gazebo GUI test ediliyor (5 saniye)..."
echo -e "${YELLOW}⚠ Gazebo penceresi açılacak, 5 saniye sonra kapanacak${NC}"

docker exec gazebo-test bash -c "timeout 5 gz sim empty.sdf" &
GAZEBO_PID=$!

sleep 6

if ps -p $GAZEBO_PID > /dev/null 2>&1; then
    kill $GAZEBO_PID 2>/dev/null || true
fi

echo -e "${GREEN}✓ Gazebo test tamamlandı${NC}"
echo ""

# Cleanup
echo -e "${YELLOW}🧹 Temizleniyor...${NC}"
docker stop gazebo-test > /dev/null 2>&1
docker rm gazebo-test > /dev/null 2>&1
xhost -local:docker > /dev/null 2>&1

echo ""
echo "========================================="
echo -e "${GREEN}✓ Test tamamlandı!${NC}"
echo "========================================="
echo ""
echo "Gazebo Garden ile çalıştırmak için:"
echo "  ./run_docker.sh navigation"
