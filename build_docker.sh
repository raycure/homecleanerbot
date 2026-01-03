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
