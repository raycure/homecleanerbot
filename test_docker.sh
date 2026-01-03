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
