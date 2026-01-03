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
