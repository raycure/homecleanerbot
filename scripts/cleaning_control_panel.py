#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, 
                             QVBoxLayout, QWidget, QLabel, QHBoxLayout)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

class CleaningControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # ROS2 Node
        rclpy.init()
        self.node = Node('cleaning_control_panel')
        
        # Service clients
        self.start_client = self.node.create_client(Trigger, '/start_cleaning')
        self.stop_client = self.node.create_client(Trigger, '/stop_cleaning')
        self.dock_client = self.node.create_client(Trigger, '/dock_robot')
        
        # Wait for services
        self.node.get_logger().info('Servislerin hazır olması bekleniyor...')
        
        # Status
        self.is_cleaning = False
        
        # Setup UI
        self.init_ui()
        
        # Timer for ROS2 spin
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)  # 10 Hz
    
    def init_ui(self):
        """UI elemanlarını oluştur"""
        self.setWindowTitle('HomeCleanerBot Control Panel')
        self.setGeometry(100, 100, 400, 300)
        
        # Ana widget ve layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Başlık
        title = QLabel('🤖 HomeCleanerBot')
        title.setAlignment(Qt.AlignCenter)
        title_font = QFont()
        title_font.setPointSize(20)
        title_font.setBold(True)
        title.setFont(title_font)
        main_layout.addWidget(title)
        
        # Durum etiketi
        self.status_label = QLabel('Durum: Beklemede')
        self.status_label.setAlignment(Qt.AlignCenter)
        status_font = QFont()
        status_font.setPointSize(14)
        self.status_label.setFont(status_font)
        self.status_label.setStyleSheet("color: gray; padding: 10px;")
        main_layout.addWidget(self.status_label)
        
        # Start Cleaning butonu
        self.start_btn = QPushButton('▶ Start Cleaning')
        self.start_btn.setMinimumHeight(60)
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.start_btn.clicked.connect(self.start_cleaning)
        main_layout.addWidget(self.start_btn)
        
        # Stop Cleaning butonu
        self.stop_btn = QPushButton('⏸ Stop Cleaning')
        self.stop_btn.setMinimumHeight(60)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:pressed {
                background-color: #c41408;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.stop_btn.clicked.connect(self.stop_cleaning)
        self.stop_btn.setEnabled(False)
        main_layout.addWidget(self.stop_btn)
        
        # Dock (Şarja Dön) butonu
        self.dock_btn = QPushButton('🔌 Return to Dock')
        self.dock_btn.setMinimumHeight(60)
        self.dock_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #0b7dda;
            }
            QPushButton:pressed {
                background-color: #0969c3;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.dock_btn.clicked.connect(self.dock_robot)
        main_layout.addWidget(self.dock_btn)
        
        # Alt bilgi
        info_label = QLabel('ROS2 Humble • HomeCleanerBot v1.0')
        info_label.setAlignment(Qt.AlignCenter)
        info_label.setStyleSheet("color: gray; font-size: 10px; padding: 5px;")
        main_layout.addWidget(info_label)
    
    def spin_ros(self):
        """ROS2 node'u spin et"""
        rclpy.spin_once(self.node, timeout_sec=0)
    
    def start_cleaning(self):
        """Temizliği başlat"""
        self.node.get_logger().info('Start Cleaning komutu gönderiliyor...')
        
        if not self.start_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Start cleaning servisi bulunamadı!')
            self.status_label.setText('❌ Hata: Servis bulunamadı')
            self.status_label.setStyleSheet("color: red; padding: 10px;")
            return
        
        request = Trigger.Request()
        future = self.start_client.call_async(request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.is_cleaning = True
                self.status_label.setText('🧹 Durum: Temizlik Yapılıyor')
                self.status_label.setStyleSheet("color: green; padding: 10px;")
                self.start_btn.setEnabled(False)
                self.stop_btn.setEnabled(True)
                self.node.get_logger().info(f'Başarılı: {response.message}')
            else:
                self.status_label.setText(f'❌ Hata: {response.message}')
                self.status_label.setStyleSheet("color: red; padding: 10px;")
                self.node.get_logger().error(f'Başarısız: {response.message}')
        else:
            self.status_label.setText('❌ Servis yanıt vermedi')
            self.status_label.setStyleSheet("color: red; padding: 10px;")
    
    def stop_cleaning(self):
        """Temizliği durdur"""
        self.node.get_logger().info('Stop Cleaning komutu gönderiliyor...')
        
        if not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Stop cleaning servisi bulunamadı!')
            self.status_label.setText('❌ Hata: Servis bulunamadı')
            self.status_label.setStyleSheet("color: red; padding: 10px;")
            return
        
        request = Trigger.Request()
        future = self.stop_client.call_async(request)
        
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.is_cleaning = False
                self.status_label.setText('⏸ Durum: Durduruldu')
                self.status_label.setStyleSheet("color: orange; padding: 10px;")
                self.start_btn.setEnabled(True)
                self.stop_btn.setEnabled(False)
                self.node.get_logger().info(f'Başarılı: {response.message}')
            else:
                self.status_label.setText(f'❌ Hata: {response.message}')
                self.status_label.setStyleSheet("color: red; padding: 10px;")
        else:
            self.status_label.setText('❌ Servis yanıt vermedi')
            self.status_label.setStyleSheet("color: red; padding: 10px;")
    
    def dock_robot(self):
        """Robotu şarj istasyonuna gönder"""
        self.node.get_logger().info('Dock komutu gönderiliyor...')
        
        if not self.dock_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Dock servisi bulunamadı!')
            self.status_label.setText('❌ Hata: Dock servisi bulunamadı')
            self.status_label.setStyleSheet("color: red; padding: 10px;")
            return
        
        request = Trigger.Request()
        future = self.dock_client.call_async(request)
        
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.status_label.setText('🔌 Durum: Şarja Gidiyor')
                self.status_label.setStyleSheet("color: blue; padding: 10px;")
                self.node.get_logger().info(f'Başarılı: {response.message}')
            else:
                self.status_label.setText(f'❌ Hata: {response.message}')
                self.status_label.setStyleSheet("color: red; padding: 10px;")
        else:
            self.status_label.setText('❌ Servis yanıt vermedi')
            self.status_label.setStyleSheet("color: red; padding: 10px;")
    
    def closeEvent(self, event):
        """Pencere kapanırken cleanup"""
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = CleaningControlPanel()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
