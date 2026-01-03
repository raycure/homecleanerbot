# HomeCleanerBot - Autonomous Home Cleaning Robot

## ROS2 Humble + Gazebo Garden + Navigation2

# Proje Ekibi

Fatma Cüre - 220601023

Yunus Emre Yıldırım - 220601014

# Proje Özeti
HomeCleanerBot, ev içi ortamlarda otonom olarak temizlik yapabilen bir robot simülasyon sistemidir. ROS2 Humble, Gazebo Garden ve Navigation2 kullanılarak geliştirilmiştir.

# Özellikler

- Gazebo Garden simülasyon ortamı

- SLAM Toolbox ile harita çıkarma

- Navigation2 (Nav2) ile otonom gezinme

- Coverage Planning ile sistematik temizlik

- AMCL ile lokalizasyon

- Docking sistemi (şarj istasyonuna dönüş)

- Docker konteynerizasyonu

- RViz2 görselleştirme

Kurulum ve Çalıştırma

# Gereksinimler
- İşletim Sistemi: Ubuntu 22.04

- Docker: 20.10+

- Docker Compose: 2.0+ (opsiyonel)

- X11 Server: GUI için

- Disk Alanı: ~10 GB

- RAM: Minimum 8 GB

# 1. Projeyi İndirin
git clone <repository-url>

cd HomeCleanerBot

# 2. Docker Dosyalarını Hazırlayın

chmod +x build_docker.sh

chmod +x run_docker.sh

chmod +x test_docker.sh

# 3. Docker İmajını Oluşturun

Build scripti ile (önerilen)

./build_docker.sh

veya manuel

docker build -t homecleanerbot:latest .

# 4. Sistemi Test Edin
./test_docker.sh

Mod 1: Mapping (Harita Çıkarma) -> ros2 launch launch/slam_mapping.launch.py

Mod 2: Navigation (Otonom Temizlik) -> # Navigation modunda başlatın
./run_docker.sh navigation

Temizliği başlatmak: ros2 service call /start_cleaning std_srvs/srv/Trigger "{}"

Temizliği durdurmak: ros2 service call /stop_cleaning std_srvs/srv/Trigger "{}"

# Teknik Detaylar
Sensörler

- LiDAR: 360° lazer tarama, 10m menzil
- Odometry: Differential drive encoder'ları
- IMU: (opsiyonel) yönelim bilgisi

Algoritmalar

- SLAM: slam_toolbox (async SLAM)
- Localization: AMCL (Adaptive Monte Carlo Localization)
- Path Planning: NavFn veya ThetaStar
- Controller: DWB (Dynamic Window Approach)
- Coverage: Boustrophedon (zigzag) pattern

ROS2 Paketleri

- ros-humble-navigation2
- ros-humble-slam-toolbox
- ros-humble-robot-localization
- ros-humble-ros-gz-bridge
- ros-humble-teleop-twist-keyboard

Gazebo Modelleri

- Ev Modeli: 2+1 daire layout'u (SDF formatında)
- Robot Modeli: Differential drive robot

# Kaynaklar

- ROS2 Humble Documentation
- Navigation2 Documentation
- Gazebo Documentation
- SLAM Toolbox

# Katkıda Bulunanlar

- Fatma Cüre (220601023)
- Yunus Emre Yıldırım (220601014)
