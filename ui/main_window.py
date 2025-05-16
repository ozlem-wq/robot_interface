import sys
import rospy
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QStackedWidget
from ui.home_page import HomePage
from ui.navigation_page import NavigationPage
from ui.notifications_page import NotificationsPage
from ui.performance_page import PerformancePage
from ui.settings_page import SettingsPage

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Otonom Yük Taşıma Robotu Arayüzü")
        self.setGeometry(100, 100, 1280, 720)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        main_layout = QVBoxLayout(self.central_widget)

        # Üst menü
        self.menu_layout = QHBoxLayout()
        self.btn_home = QPushButton("Ana Sayfa")
        self.btn_navigation = QPushButton("Navigasyon")
        self.btn_notifications = QPushButton("Bildirimler")
        self.btn_performance = QPushButton("Performans")
        self.btn_settings = QPushButton("Ayarlar")

        for btn in [self.btn_home, self.btn_navigation, self.btn_notifications, self.btn_performance, self.btn_settings]:
            btn.setFixedHeight(40)
            self.menu_layout.addWidget(btn)

        main_layout.addLayout(self.menu_layout)

        # Sayfa içeriği
        self.stack = QStackedWidget()
        self.home_page = HomePage()
        self.navigation_page = NavigationPage()
        self.notifications_page = NotificationsPage()
        self.performance_page = PerformancePage()
        self.settings_page = SettingsPage()

        self.stack.addWidget(self.home_page)
        self.stack.addWidget(self.navigation_page)
        self.stack.addWidget(self.notifications_page)
        self.stack.addWidget(self.performance_page)
        self.stack.addWidget(self.settings_page)

        main_layout.addWidget(self.stack)

        # Menü buton bağlantıları
        self.btn_home.clicked.connect(lambda: self.stack.setCurrentWidget(self.home_page))
        self.btn_navigation.clicked.connect(lambda: self.stack.setCurrentWidget(self.navigation_page))
        self.btn_notifications.clicked.connect(lambda: self.stack.setCurrentWidget(self.notifications_page))
        self.btn_performance.clicked.connect(lambda: self.stack.setCurrentWidget(self.performance_page))
        self.btn_settings.clicked.connect(lambda: self.stack.setCurrentWidget(self.settings_page))

        # Tema dosyasını uygula (varsayılan)
        self.apply_theme("dark")

    def apply_theme(self, theme_name):
        if theme_name == "dark":
            qss_path = "ui/style_dark.qss"
        else:
            qss_path = "ui/style.qss"
        try:
            with open(qss_path, "r") as f:
                self.setStyleSheet(f.read())
        except FileNotFoundError:
            print(f"⚠ Stil dosyası bulunamadı: {qss_path}")

def main():
    try:
        rospy.init_node("robot_interface_ui", anonymous=True, disable_signals=True)
    except rospy.ROSException:
        pass

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

