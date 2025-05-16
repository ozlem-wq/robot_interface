import sys
import os
import rospy
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QStackedWidget
)
from std_msgs.msg import String

from ui.home_page import HomePage
from ui.navigation_page import NavigationPage
from ui.notifications_page import NotificationsPage
from ui.settings_page import SettingsPage
from ui.performance_page import PerformancePage

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Otonom Yük Taşıma Robotu Arayüzü")
        self.setGeometry(100, 100, 1280, 720)

        # ROS publisher
        self.control_pub = rospy.Publisher("/robot_control", String, queue_size=10)

        # Merkez widget ve ana layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Üst menü butonları
        menu_layout = QHBoxLayout()
        self.btn_home = QPushButton("🏠 Ana Sayfa")
        self.btn_navigation = QPushButton("🗺 Navigasyon")
        self.btn_notifications = QPushButton("🔔 Bildirimler")
        self.btn_settings = QPushButton("⚙️ Ayarlar")
        self.btn_performance = QPushButton("📊 Performans")

        menu_buttons = [
            self.btn_home, self.btn_navigation,
            self.btn_notifications, self.btn_settings, self.btn_performance
        ]
        for btn in menu_buttons:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #607D8B;
                    color: white;
                    font-size: 14px;
                    padding: 10px;
                    border-radius: 8px;
                }
                QPushButton:hover {
                    background-color: #455A64;
                }
            """)
            menu_layout.addWidget(btn)

        # Sayfa stack
        self.stack = QStackedWidget()
        self.home_page = HomePage()
        self.navigation_page = NavigationPage()
        self.notifications_page = NotificationsPage()
        self.settings_page = SettingsPage()
        self.performance_page = PerformancePage()

        self.stack.addWidget(self.home_page)
        self.stack.addWidget(self.navigation_page)
        self.stack.addWidget(self.notifications_page)
        self.stack.addWidget(self.settings_page)
        self.stack.addWidget(self.performance_page)

        # Sayfa geçiş bağlantıları
        self.btn_home.clicked.connect(lambda: self.stack.setCurrentWidget(self.home_page))
        self.btn_navigation.clicked.connect(lambda: self.stack.setCurrentWidget(self.navigation_page))
        self.btn_notifications.clicked.connect(lambda: self.stack.setCurrentWidget(self.notifications_page))
        self.btn_settings.clicked.connect(lambda: self.stack.setCurrentWidget(self.settings_page))
        self.btn_performance.clicked.connect(lambda: self.stack.setCurrentWidget(self.performance_page))

        # Başlat/Durdur butonları ROS mesajlarıyla bağlantılı
        self.home_page.btn_start.clicked.connect(self.send_start_command)
        self.home_page.btn_stop.clicked.connect(self.send_stop_command)

        # Layout yerleşimi
        main_layout.addLayout(menu_layout)
        main_layout.addWidget(self.stack)

    def send_start_command(self):
        self.control_pub.publish("start")
        rospy.loginfo("▶ Robot başlatıldı.")

    def send_stop_command(self):
        self.control_pub.publish("stop")
        rospy.loginfo("■ Robot durduruldu.")

def main():
    try:
        rospy.init_node("robot_interface_ui", anonymous=True, disable_signals=True)
    except rospy.ROSException:
        pass

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

